/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "thrust_feedback_control.hpp"
#include "bet_inverse_solver.hpp"

#include <cstring>

px4::AppState ThrustFeedbackControl::appState;

namespace
{

constexpr int kMotorCount = 4;
constexpr float kGramForceToNewton = 9.80665e-3f;
constexpr float kDefaultControlDt = 0.01f;
constexpr float kMinControlDt = 0.001f;
constexpr float kMaxControlDt = 0.05f;
constexpr uint64_t kBetWarnIntervalUs = 1000000;

struct PiState {
	float integral{0.f};
	uint64_t last_timestamp{0};

	void reset()
	{
		integral = 0.f;
		last_timestamp = 0;
	}
};

float grams_to_newtons(float grams)
{
	return PX4_ISFINITE(grams) ? grams * kGramForceToNewton : 0.f;
}

float clean_normalized_control(float raw)
{
	if (!PX4_ISFINITE(raw)) {
		return 0.f;
	}

	return math::constrain(raw, 0.f, 1.f);
}

float clean_force_n(float force_n, float max_force_n)
{
	if (!PX4_ISFINITE(force_n)) {
		return 0.f;
	}

	const float upper_limit = PX4_ISFINITE(max_force_n) ? fmaxf(max_force_n, 0.f) : 0.f;
	return math::constrain(force_n, 0.f, upper_limit);
}

float rpm_to_control_ff(float rpm, int motor_index)
{
	(void)rpm;
	(void)motor_index;
	return 0.f;
}

const char *bet_status_string(thrust_feedback_control::bet::BetRpmStatus status)
{
	using thrust_feedback_control::bet::BetRpmStatus;

	switch (status) {
	case BetRpmStatus::Ok:
		return "ok";

	case BetRpmStatus::InvalidInput:
		return "invalid_input";

	case BetRpmStatus::OutOfTableRange:
		return "out_of_table_range";

	case BetRpmStatus::AboveRpmLimit:
		return "above_rpm_limit";

	case BetRpmStatus::NonMonotonicTable:
		return "non_monotonic_table";
	}

	return "unknown";
}

void warn_bet_status_once_per_second(uint64_t now, int motor_index, thrust_feedback_control::bet::BetRpmStatus status,
				     float desired_lift_n, float freestream_m_s, float alpha_deg)
{
	static uint64_t last_warn_us = 0;

	if (now - last_warn_us < kBetWarnIntervalUs) {
		return;
	}

	last_warn_us = now;
	PX4_WARN("TFC BET motor %d status=%s lift=%.3fN V=%.3fm/s alpha=%.2fdeg",
		 motor_index + 1, bet_status_string(status), (double)desired_lift_n, (double)freestream_m_s,
		 (double)alpha_deg);
}

float control_dt_s(PiState &state, uint64_t now)
{
	if (state.last_timestamp == 0) {
		state.last_timestamp = now;
		return kDefaultControlDt;
	}

	const float dt = math::constrain((now - state.last_timestamp) * 1e-6f, kMinControlDt, kMaxControlDt);
	state.last_timestamp = now;
	return dt;
}

float update_pi(PiState &state, float error_n, float kp, float ki, float lim_i, float dt_s)
{
	if (!PX4_ISFINITE(error_n) || !PX4_ISFINITE(kp) || !PX4_ISFINITE(ki) || !PX4_ISFINITE(lim_i)) {
		state.reset();
		return 0.f;
	}

	const float i_limit = fmaxf(lim_i, 0.f);
	state.integral += error_n * dt_s;

	if (fabsf(ki) > 1e-6f && i_limit > 0.f) {
		const float integral_limit = i_limit / fabsf(ki);
		state.integral = math::constrain(state.integral, -integral_limit, integral_limit);

	} else {
		state.integral = 0.f;
	}

	const float p_out = kp * error_n;
	const float i_out = math::constrain(ki * state.integral, -i_limit, i_limit);
	return p_out + i_out;
}

void reset_vector4(matrix::Vector<float, 4> &values)
{
	for (int i = 0; i < kMotorCount; ++i) {
		values(i) = 0.f;
	}
}

void reset_pi_states(PiState states[kMotorCount])
{
	for (int i = 0; i < kMotorCount; ++i) {
		states[i].reset();
	}
}

} // namespace

void ThrustFeedbackControl::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

int ThrustFeedbackControl::main()
{
	appState.setRunning(true);

	const int thrustdata_sub_fd = orb_subscribe(ORB_ID(pressure_force_sensor));
	orb_set_interval(thrustdata_sub_fd, 5);

	const int thrustdesireddata_sub_fd = orb_subscribe(ORB_ID(actuator_motors));
	orb_set_interval(thrustdesireddata_sub_fd, 5);

	px4_pollfd_struct_t fds[] = {
		{ .fd = thrustdata_sub_fd, .events = POLLIN },
		{ .fd = thrustdesireddata_sub_fd, .events = POLLIN },
	};

	int error_counter = 0;
	pressure_force_sensor_s sensordata{};
	actuator_motors_s thrustdesireddata{};
	PiState pi_states[kMotorCount]{};
	bool sensor_updated = false;
	bool desired_updated = false;

	reset_vector4(_thrust_desired);
	reset_vector4(_thrust_measure);
	reset_vector4(_control_output);
	reset_vector4(_total_output);
	reset_vector4(_iolc_u_ff);
	reset_vector4(_thrust_desired_dot);

	while (appState.isRunning()) {
		parameters_update();

		if (_vehicle_status_sub.update(&_vehicle_status)) {
			_armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}

		if (_mcs_sub.update(&_mcs)) {
			const float normalized_throttle = clean_normalized_control((_mcs.throttle + 1.f) * 0.5f);
			_force_from_rc = clean_force_n(normalized_throttle * _param_tfc_thrust_max.get(), _param_tfc_thrust_max.get());
		}

		const int poll_ret = px4_poll(fds, 2, 1000);

		if (poll_ret == 0) {
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			if (error_counter < 10 || error_counter % 50 == 0) {
				PX4_ERR("ERROR RETURN VALUE FROM POLL(): %d", poll_ret);
			}

			error_counter++;

		} else {
			if (fds[0].revents & POLLIN) {
				sensor_updated = true;
				orb_copy(ORB_ID(pressure_force_sensor), thrustdata_sub_fd, &sensordata);

				thrustdata.timestamp = hrt_absolute_time();
				static uint64_t last_sensor_timestamp = thrustdata.timestamp;

				thrustdata.thrust_raw_data_1 = grams_to_newtons(-static_cast<float>(sensordata.sensor2));
				thrustdata.thrust_raw_data_2 = grams_to_newtons(static_cast<float>(sensordata.sensor2)
							       + _param_sensor2_bias1.get() + _param_sensor2_bias2.get());
				thrustdata.thrust_raw_data_3 = grams_to_newtons(static_cast<float>(sensordata.sensor3)
							       + _param_sensor3_bias1.get() + _param_sensor3_bias2.get());
				thrustdata.thrust_raw_data_4 = grams_to_newtons(static_cast<float>(sensordata.sensor4)
							       + _param_sensor4_bias1.get() + _param_sensor4_bias2.get());

				const float max_force_n = _param_tfc_thrust_max.get();
				thrustdata.thrust_raw_data_1 = clean_force_n(thrustdata.thrust_raw_data_1, max_force_n);
				thrustdata.thrust_raw_data_2 = clean_force_n(thrustdata.thrust_raw_data_2, max_force_n);
				thrustdata.thrust_raw_data_3 = clean_force_n(thrustdata.thrust_raw_data_3, max_force_n);
				thrustdata.thrust_raw_data_4 = clean_force_n(thrustdata.thrust_raw_data_4, max_force_n);

				const float dt = math::constrain((thrustdata.timestamp - last_sensor_timestamp) * 1e-6f,
								 kMinControlDt, kMaxControlDt);

				thrustdata.thrust_kalman_filter_data_1 = clean_force_n(
						thrust_kalman_filter.thrust_kalman_filter_BFS1(dt, thrustdata.thrust_raw_data_1), max_force_n);
				thrustdata.thrust_kalman_filter_data_2 = clean_force_n(
						thrust_kalman_filter.thrust_kalman_filter_BFS2(dt, thrustdata.thrust_raw_data_2), max_force_n);
				thrustdata.thrust_kalman_filter_data_3 = clean_force_n(
						thrust_kalman_filter.thrust_kalman_filter_BFS3(dt, thrustdata.thrust_raw_data_3), max_force_n);
				thrustdata.thrust_kalman_filter_data_4 = clean_force_n(
						thrust_kalman_filter.thrust_kalman_filter_BFS4(dt, thrustdata.thrust_raw_data_4), max_force_n);

				last_sensor_timestamp = thrustdata.timestamp;
				_thrustdata_pub.publish(thrustdata);
			}

			if (fds[1].revents & POLLIN) {
				desired_updated = true;
				orb_copy(ORB_ID(actuator_motors), thrustdesireddata_sub_fd, &thrustdesireddata);

				const uint64_t now = hrt_absolute_time();
				const float max_force_n = fmaxf(_param_tfc_thrust_max.get(), 0.f);
				thrustdesireddata.timestamp = now;

				thrustcontrol.timestamp = now;
				thrustcontrol.timestamp_sample = thrustdesireddata.timestamp_sample;
				thrustcontrol.reversible_flags = thrustdesireddata.reversible_flags;
				std::memcpy(thrustcontrol.control, thrustdesireddata.control, sizeof(thrustcontrol.control));

				_thrust_desired(0) = clean_force_n(_force_from_rc, max_force_n);
				_thrust_desired(1) = clean_force_n(clean_normalized_control(thrustdesireddata.control[1]) * max_force_n, max_force_n);
				_thrust_desired(2) = clean_force_n(clean_normalized_control(thrustdesireddata.control[2]) * max_force_n, max_force_n);
				_thrust_desired(3) = clean_force_n(clean_normalized_control(thrustdesireddata.control[3]) * max_force_n, max_force_n);
			}
		}

		if (!_armed) {
			sensor_updated = false;
			desired_updated = false;
			reset_pi_states(pi_states);
			reset_vector4(_control_output);
			reset_vector4(_total_output);
			reset_vector4(_iolc_u_ff);
			px4_usleep(1000);
			continue;
		}

		if (sensor_updated && desired_updated) {
			sensor_updated = false;
			desired_updated = false;

			if (_param_tfc_use_filtered_thrust.get() > 0.5f) {
				_thrust_measure(0) = thrustdata.thrust_kalman_filter_data_1;
				_thrust_measure(1) = thrustdata.thrust_kalman_filter_data_2;
				_thrust_measure(2) = thrustdata.thrust_kalman_filter_data_3;
				_thrust_measure(3) = thrustdata.thrust_kalman_filter_data_4;

			} else {
				_thrust_measure(0) = thrustdata.thrust_raw_data_1;
				_thrust_measure(1) = thrustdata.thrust_raw_data_2;
				_thrust_measure(2) = thrustdata.thrust_raw_data_3;
				_thrust_measure(3) = thrustdata.thrust_raw_data_4;
			}

			const uint64_t now = hrt_absolute_time();
			const float bet_freestream_m_s = _param_tfc_bet_v_ms.get();
			const float bet_alpha_deg = _param_tfc_bet_a_deg.get();
			const float kp = _param_tfc_pid_kp.get();
			const float ki = _param_tfc_pid_ki.get();
			const float lim_i = _param_tfc_pid_lim_i.get();

			float rpm_ff[kMotorCount] = {};
			float dt_s[kMotorCount] = {};
			float thrust_error[kMotorCount] = {};
			float feedback_out[kMotorCount] = {};

			for (int i = 0; i < kMotorCount; ++i) {
				_thrust_desired(i) = clean_force_n(_thrust_desired(i), _param_tfc_thrust_max.get());
				_thrust_measure(i) = clean_force_n(_thrust_measure(i), _param_tfc_thrust_max.get());
				_thrust_desired_dot(i) = 0.f;

				if (_thrust_desired(i) <= 1e-5f) {
					pi_states[i].reset();
					_iolc_u_ff(i) = 0.f;
					_control_output(i) = 0.f;
					_total_output(i) = 0.f;
					thrust_error[i] = -_thrust_measure(i);
					continue;
				}

				const thrust_feedback_control::bet::BetRpmSolution bet_solution =
					thrust_feedback_control::bet::lookup_rpm_for_lift_n(_thrust_desired(i), bet_freestream_m_s, bet_alpha_deg);

				if (bet_solution.status == thrust_feedback_control::bet::BetRpmStatus::Ok) {
					rpm_ff[i] = bet_solution.rpm;

				} else if (bet_solution.status == thrust_feedback_control::bet::BetRpmStatus::AboveRpmLimit) {
					rpm_ff[i] = thrust_feedback_control::bet::kBetMaxRpm;
					warn_bet_status_once_per_second(now, i, bet_solution.status, _thrust_desired(i),
									bet_freestream_m_s, bet_alpha_deg);

				} else {
					rpm_ff[i] = 0.f;
					warn_bet_status_once_per_second(now, i, bet_solution.status, _thrust_desired(i),
									bet_freestream_m_s, bet_alpha_deg);
				}

				_iolc_u_ff(i) = math::constrain(rpm_to_control_ff(rpm_ff[i], i), 0.f, 1.f);
				thrust_error[i] = _thrust_desired(i) - _thrust_measure(i);
				dt_s[i] = control_dt_s(pi_states[i], now);
				feedback_out[i] = update_pi(pi_states[i], thrust_error[i], kp, ki, lim_i, dt_s[i]);
				_control_output(i) = feedback_out[i];
				_total_output(i) = math::constrain(_iolc_u_ff(i) + feedback_out[i], 0.f, 1.f);
			}

			thrustcontroldata.motor_speed_dot1 = 0.f;
			thrustcontroldata.motor_speed_dot2 = 0.f;
			thrustcontroldata.motor_speed_dot3 = 0.f;
			thrustcontroldata.motor_speed_dot4 = 0.f;
			thrustcontroldata.motor_speed1 = rpm_ff[0];
			thrustcontroldata.motor_speed2 = rpm_ff[1];
			thrustcontroldata.motor_speed3 = rpm_ff[2];
			thrustcontroldata.motor_speed4 = rpm_ff[3];
			thrustcontroldata.f_x1 = 0.f;
			thrustcontroldata.f_x2 = 0.f;
			thrustcontroldata.f_x3 = 0.f;
			thrustcontroldata.f_x4 = 0.f;
			thrustcontroldata.g_x1 = 0.f;
			thrustcontroldata.g_x2 = 0.f;
			thrustcontroldata.g_x3 = 0.f;
			thrustcontroldata.g_x4 = 0.f;
			thrustcontroldata.deta_t_1 = dt_s[0];
			thrustcontroldata.deta_t_2 = dt_s[1];
			thrustcontroldata.deta_t_3 = dt_s[2];
			thrustcontroldata.deta_t_4 = dt_s[3];
			thrustcontroldata.thrust_error1 = thrust_error[0];
			thrustcontroldata.thrust_error2 = thrust_error[1];
			thrustcontroldata.thrust_error3 = thrust_error[2];
			thrustcontroldata.thrust_error4 = thrust_error[3];
			thrustcontroldata.thrust_desired1 = _thrust_desired(0);
			thrustcontroldata.thrust_desired2 = _thrust_desired(1);
			thrustcontroldata.thrust_desired3 = _thrust_desired(2);
			thrustcontroldata.thrust_desired4 = _thrust_desired(3);
			thrustcontroldata.thrust_desired_dot1 = 0.f;
			thrustcontroldata.thrust_desired_dot2 = 0.f;
			thrustcontroldata.thrust_desired_dot3 = 0.f;
			thrustcontroldata.thrust_desired_dot4 = 0.f;
			thrustcontroldata.thrust_feedback_out1 = feedback_out[0];
			thrustcontroldata.thrust_feedback_out2 = feedback_out[1];
			thrustcontroldata.thrust_feedback_out3 = feedback_out[2];
			thrustcontroldata.thrust_feedback_out4 = feedback_out[3];
			thrustcontroldata.thrust_feedforward_out1 = _iolc_u_ff(0);
			thrustcontroldata.thrust_feedforward_out2 = _iolc_u_ff(1);
			thrustcontroldata.thrust_feedforward_out3 = _iolc_u_ff(2);
			thrustcontroldata.thrust_feedforward_out4 = _iolc_u_ff(3);
			thrustcontroldata.thrust_control_out1 = _total_output(0);
			thrustcontroldata.thrust_control_out2 = _total_output(1);
			thrustcontroldata.thrust_control_out3 = _total_output(2);
			thrustcontroldata.thrust_control_out4 = _total_output(3);
			thrustcontroldata.thrust_control_out_pwm1 = math::constrain((2.f * _total_output(0) - 1.f), -1.f, 1.f);
			thrustcontroldata.thrust_control_out_pwm2 = math::constrain((2.f * _total_output(1) - 1.f), -1.f, 1.f);
			thrustcontroldata.thrust_control_out_pwm3 = math::constrain((2.f * _total_output(2) - 1.f), -1.f, 1.f);
			thrustcontroldata.thrust_control_out_pwm4 = math::constrain((2.f * _total_output(3) - 1.f), -1.f, 1.f);
			thrustcontroldata.kp = kp;
			thrustcontroldata.ki = ki;
			thrustcontroldata.kd = _param_tfc_pid_kd.get();
			thrustcontroldata.thrust_start = _param_tfc_start.get();

			thrustcontroldata.timestamp = now;
			_thrustcontroldata_pub.publish(thrustcontroldata);

			if (thrustcontroldata.thrust_start > 0.5f && thrustcontroldata.thrust_start < 1.5f) {
				thrustcontrol.control[0] = thrustcontroldata.thrust_control_out1;
				thrustcontrol.control[1] = thrustcontroldata.thrust_control_out2;
				thrustcontrol.control[2] = thrustcontroldata.thrust_control_out3;
				thrustcontrol.control[3] = thrustcontroldata.thrust_control_out4;
			}

			thrustcontrol.timestamp = hrt_absolute_time();
			_thrustcontrol_pub.publish(thrustcontrol);
		}

		px4_usleep(1000);
	}

	return 0;
}
