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

#include <math.h>

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

	PX4_DEBUG("AS5600 addr: 0x%02x, pool: %" PRId32 " us, filter: %.2f",
		  get_device_address(),
		  _param_as5600_pool.get(),
		  (double)_param_as5600_filter.get());

	// Read initial angle to initialize state
	int32_t angle = readRawAngle();

	if (angle < 0) {
		PX4_ERR("AS5600 failed to read initial angle");
		return PX4_ERROR;
	}

	_last_raw_angle = angle;
	_last_measurement_time = hrt_absolute_time();

	ScheduleOnInterval(_param_as5600_pool.get());
	_rpm_pub.advertise();

	return PX4_OK;
}

int AS5600::probe()
{
	// Check that the magnet is detected by reading status register
	int32_t status = readStatus();

	if (status < 0) {
		PX4_DEBUG("AS5600 probe: I2C transfer failed");
		return PX4_ERROR;
	}

	if (!(status & AS5600_STATUS_MD)) {
		PX4_DEBUG("AS5600 probe: magnet not detected (status=0x%02x)", status);
		// Still allow init even without magnet — it might be placed later
	}

	// Try to read the raw angle register to verify device responds correctly
	int32_t angle = readRawAngle();

	if (angle < 0) {
		PX4_DEBUG("AS5600 probe: failed to read angle register");
		return PX4_ERROR;
	}

	PX4_DEBUG("AS5600 probe: status=0x%02x, raw_angle=%d", status, angle);
	return PX4_OK;
}

int32_t AS5600::readRawAngle()
{
	return readRegister16(AS5600_REG_RAW_ANGLE_HIGH);
}

int32_t AS5600::readStatus()
{
	return readRegister(AS5600_REG_STATUS);
}

int32_t AS5600::readRegister(uint8_t reg)
{
	uint8_t val{};
	int ret = transfer(&reg, 1, &val, 1);

	if (ret != PX4_OK) {
		PX4_DEBUG("AS5600 readRegister(0x%02x): I2C transfer failed (%d)", reg, ret);
		_transfer_fail_count++;
		return -1;
	}

	return val;
}

int32_t AS5600::readRegister16(uint8_t reg_high)
{
	uint8_t buf[2]{};
	int ret = transfer(&reg_high, 1, buf, 2);

	if (ret != PX4_OK) {
		PX4_DEBUG("AS5600 readRegister16(0x%02x): I2C transfer failed (%d)", reg_high, ret);
		_transfer_fail_count++;
		return -1;
	}

	// AS5600 raw angle: high byte [11:8] in buf[0], low byte [7:0] in buf[1]
	return ((int32_t)(buf[0] & 0x0F) << 8) | buf[1];
}

void AS5600::RunImpl()
{
	// Read current raw angle from sensor
	int32_t current_angle = readRawAngle();

	if (current_angle < 0) {
		_transfer_fail_count++;

		if (_transfer_fail_count > 10) {
			PX4_ERR("AS5600: too many I2C failures (%d), attempting reinit", _transfer_fail_count);
			_transfer_fail_count = 0;
		}

		return;
	}

	_transfer_fail_count = 0;

	hrt_abstime now = hrt_absolute_time();
	float dt = (float)(now - _last_measurement_time) / 1e6f;  // seconds

	// Avoid division by zero or impossibly short intervals
	if (dt < 1e-6f) {
		_last_raw_angle = current_angle;
		_last_measurement_time = now;
		return;
	}

	// Calculate angular difference, handling wrap-around
	int32_t diff = current_angle - _last_raw_angle;
	const int32_t dir_mode = _param_as5600_dir.get();

	if(dir_mode > 0) {
		if(diff < 0)
		{
			diff += AS5600_RESOLUTION;  // reverse direction wrap forward
		} // reverse direction
	}
	else if(dir_mode < 0) {
		if(diff > 0)
		{
			diff -= AS5600_RESOLUTION;  // reverse direction wrap backward
		} // reverse direction
	}

	// Convert angular difference to RPM
	// diff / AS5600_RESOLUTION = fraction of full rotation
	// Divide by dt (seconds) = rotations per second
	// Multiply by 60 = rotations per minute
	float rpm_raw = ((float)diff / (float)AS5600_RESOLUTION) / dt * 60.0f;

	// Suppress quantization jitter near standstill (1 LSB or less in one sample).
	if (abs(diff) <= 1) {
		rpm_raw = 0.0f;
	}

	// Low-pass filter
	float alpha = _param_as5600_filter.get();
	_rpm_filtered = alpha * rpm_raw + (1.0f - alpha) * _rpm_filtered;

	// Update state
	_last_raw_angle = current_angle;
	_last_measurement_time = now;

	// Publish RPM
	rpm_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.rpm_estimate = _rpm_filtered;
	msg.rpm_raw = rpm_raw;
	_rpm_pub.publish(msg);
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("poll interval:  %" PRId32 " us", _param_as5600_pool.get());
	PX4_INFO("filter alpha:   %.2f", (double)_param_as5600_filter.get());
	PX4_INFO("max rpm limit:  %.1f", (double)_param_as5600_max_rpm.get());
	PX4_INFO("dir mode:       %" PRId32, _param_as5600_dir.get());
	PX4_INFO("last raw angle: %" PRId32, _last_raw_angle);
	PX4_INFO("RPM (filtered): %.1f", (double)_rpm_filtered);
	PX4_INFO("I2C failures:   %d", _transfer_fail_count);
}
