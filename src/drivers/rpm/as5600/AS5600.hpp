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

/**
 * @file AS5600.hpp
 *
 * Driver for motor speed measurement using AS5600 I2C magnetic rotary position sensor.
 * The AS5600 provides 12-bit angular position output (0-4095 for 0-360 degrees).
 * RPM is calculated from the angular position difference over time.
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <drivers/device/i2c.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/rpm.h>
#include <drivers/drv_hrt.h>

/* AS5600 I2C default address */
#define AS5600_BASEADDR_DEFAULT          0x36

/* AS5600 Register Map */
#define AS5600_REG_RAW_ANGLE_HIGH        0x0C
#define AS5600_REG_RAW_ANGLE_LOW         0x0D
#define AS5600_REG_ANGLE_HIGH            0x0E
#define AS5600_REG_ANGLE_LOW             0x0F
#define AS5600_REG_STATUS                0x0B
#define AS5600_REG_AGC                   0x1A
#define AS5600_REG_MAGNITUDE_HIGH        0x1B
#define AS5600_REG_MAGNITUDE_LOW         0x1C
#define AS5600_REG_CONF_HIGH             0x07
#define AS5600_REG_CONF_LOW              0x08

/* AS5600 Status bits */
#define AS5600_STATUS_MH                 0x08  /* AGC minimum gain overflow, magnet too strong */
#define AS5600_STATUS_ML                 0x10  /* AGC maximum gain overflow, magnet too weak */
#define AS5600_STATUS_MD                 0x20  /* Magnet was detected */

/* AS5600 full rotation in raw counts (12-bit) */
#define AS5600_RESOLUTION                4096

class AS5600 : public device::I2C, public ModuleParams, public I2CSPIDriver<AS5600>
{
public:
	AS5600(const I2CSPIDriverConfig &config);
	~AS5600() override = default;

	static void print_usage();

	void		RunImpl();

	int    init() override;
	void   print_status() override;

private:

	int  probe() override;

	/**
	 * Read the raw 12-bit angle register from the AS5600.
	 * @return raw angle value (0-4095), or -1 on error
	 */
	int32_t readRawAngle();

	/**
	 * Read the status register to check magnet presence.
	 * @return status register value, or -1 on error
	 */
	int32_t readStatus();

	/**
	 * Read a single register from the AS5600.
	 * @param reg Register address
	 * @return register value, or -1 on I2C error
	 */
	int32_t readRegister(uint8_t reg);

	/**
	 * Read two consecutive registers (high + low) as a 12-bit value.
	 * @param reg_high Address of the high byte register
	 * @return 12-bit value, or -1 on I2C error
	 */
	int32_t readRegister16(uint8_t reg_high);

	int32_t        _last_raw_angle{0};
	hrt_abstime    _last_measurement_time{0};
	int            _transfer_fail_count{0};
	float          _rpm_filtered{0.0f};

	uORB::PublicationMulti<rpm_s> _rpm_pub{ORB_ID(rpm)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::AS5600_POOL>) _param_as5600_pool,
		(ParamFloat<px4::params::AS5600_FILTER>) _param_as5600_filter,
		(ParamFloat<px4::params::AS5600_MAX_RPM>) _param_as5600_max_rpm,
		(ParamInt<px4::params::AS5600_DIR>) _param_as5600_dir
	)
};
