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
* AS5600 driver enable
*
* Enable the AS5600 magnetic rotary position sensor driver for RPM measurement.
*
* @reboot_required true
* @min 0
* @max 1
* @group Sensors
* @value 0 Disabled
* @value 1 Enabled
*/
PARAM_DEFINE_INT32(SENS_EN_AS5600, 0);


/**
 * AS5600 poll interval
 *
 * Determines how often the AS5600 sensor is read out.
 * Shorter intervals give better RPM resolution but increase I2C bus load.
 *
 * @reboot_required true
 * @group Sensors
 * @unit us
 * @min 10000
 * @max 1000000
 */
PARAM_DEFINE_INT32(AS5600_POLL, 100000);

/**
 * AS5600 pole pair count
 *
 * Number of magnetic pole pairs on the motor.
 * For a direct-drive motor with one magnet on the shaft, set to 1.
 * For multi-pole motors, set to the number of pole pairs.
 *
 * @reboot_required true
 * @group Sensors
 * @min 1
 */
PARAM_DEFINE_INT32(AS5600_MAGNET, 1);
