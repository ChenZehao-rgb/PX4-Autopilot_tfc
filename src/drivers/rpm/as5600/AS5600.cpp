/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "AS5600.hpp"

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config)
{
}

int AS5600::init()
{
	if (I2C::init() != PX4_OK) {
		return PX4_ERROR;
	}

	PX4_DEBUG("addr: 0x%02x, pool: %" PRId32 ", magnet: %" PRId32,
		  get_device_address(),
		  _param_as5600_pool.get(),
		  _param_as5600_magnet.get());

	_last_angle = -1;
	_total_angle = 0;
	_last_measurement_time = hrt_absolute_time();

	ScheduleOnInterval(_param_as5600_pool.get());
	_rpm_pub.advertise();

	return PX4_OK;
}

int AS5600::probe()
{
	uint8_t status = readRegister(AS5600_REG_STATUS);

	if (_transfer_fail_count > 0) {
		PX4_DEBUG("AS5600 probe: I2C transfer failed");
		return PX4_ERROR;
	}

	if (status & AS5600_STATUS_MD) {
		PX4_DEBUG("AS5600 probe: magnet detected, status=0x%02x", status);
		return PX4_OK;
	}

	PX4_DEBUG("AS5600 probe: no magnet detected, status=0x%02x", status);
	return PX4_OK;
}

int32_t AS5600::readAngle()
{
	uint8_t buf[2] = {};
	uint8_t reg = AS5600_REG_RAW_ANGLE_H;

	int ret = transfer(&reg, 1, buf, 2);

	if (ret != PX4_OK) {
		PX4_DEBUG("readAngle: i2c::transfer returned %d", ret);
		_transfer_fail_count++;
		return -1;
	}

	int32_t angle = ((int32_t)(buf[0] & 0x0F) << 8) | buf[1];
	return angle;
}

uint8_t AS5600::readRegister(uint8_t reg)
{
	uint8_t rcv{};
	int ret = transfer(&reg, 1, &rcv, 1);

	if (ret != PX4_OK) {
		PX4_DEBUG("readRegister: i2c::transfer returned %d", ret);
		_transfer_fail_count++;
	}

	return rcv;
}

void AS5600::RunImpl()
{
	hrt_abstime now = hrt_absolute_time();
	int32_t diffTime = now - _last_measurement_time;

	if (diffTime < _param_as5600_pool.get() / 2) {
		return;
	}

	int32_t angle = readAngle();

	if (angle < 0) {
		PX4_ERR("AS5600: failed to read angle, transfer_fail_count=%d", _transfer_fail_count);

		if (_transfer_fail_count > 10) {
			_transfer_fail_count = 0;
			_last_angle = -1;
		}

		return;
	}

	_transfer_fail_count = 0;

	if (_last_angle < 0) {
		_last_angle = angle;
		_last_measurement_time = now;
		return;
	}

	int32_t diff = angle - _last_angle;

	if (diff > (AS5600_RESOLUTION / 2)) {
		diff -= AS5600_RESOLUTION;

	} else if (diff < -(AS5600_RESOLUTION / 2)) {
		diff += AS5600_RESOLUTION;
	}

	_total_angle += diff;

	float revolutions = (float)diff / (float)AS5600_RESOLUTION;
	float time_sec = (float)diffTime / 1e6f;
	float indicated_rpm = (revolutions / (float)_param_as5600_magnet.get() / time_sec) * 60.0f;

	rpm_s msg{};
	msg.rpm_estimate = indicated_rpm;
	msg.rpm_raw = indicated_rpm;
	msg.timestamp = hrt_absolute_time();
	_rpm_pub.publish(msg);

	_last_angle = angle;
	_last_measurement_time = now;
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_as5600_pool.get());
	PX4_INFO("last angle: %" PRId32, _last_angle);
	PX4_INFO("total accumulated angle: %" PRId64, _total_angle);
}
