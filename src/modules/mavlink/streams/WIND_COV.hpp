/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#ifndef WIND_COV_HPP
#define WIND_COV_HPP

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wind.h>
#include <uORB/topics/thrust_control.h>
#include <uORB/topics/thrust_data.h>
#include <uORB/topics/thrust_control_data.h>

class MavlinkStreamWindCov : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamWindCov(mavlink); }

	static constexpr const char *get_name_static() { return "WIND_COV"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_WIND_COV; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _wind_sub.advertised() ? MAVLINK_MSG_ID_WIND_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamWindCov(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _wind_sub{ORB_ID(wind)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _thrust_control_sub{ORB_ID(thrust_control)};
	uORB::Subscription _thrust_data_sub{ORB_ID(thrust_data)};
	uORB::Subscription _thrust_control_data_sub{ORB_ID(thrust_control_data)};

	bool send() override
	{
		thrust_control_s thrustcontrol = {};
		thrust_control_data_s thrustcontroldata = {};
		thrust_data_s thrustdata = {};

		if (_thrust_data_sub.update(&thrustdata)) {
			mavlink_wind_cov_t msg{};

			_thrust_control_sub.copy(&thrustcontrol);
			_thrust_control_data_sub.copy(&thrustcontroldata);
			msg.time_usec = thrustdata.timestamp;

			msg.wind_x = thrustdata.thrust_raw_data_1;
			msg.wind_y = thrustcontroldata.thrust_desired1;
			msg.wind_z = thrustcontroldata.thrust_error1;

			msg.var_horiz = thrustcontrol.control[0];
			msg.var_vert = 0.0f;

			msg.wind_alt = 0.0f;

			msg.horiz_accuracy = 0.0f;
			msg.vert_accuracy = 0.0f;

			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // WIND_COV
