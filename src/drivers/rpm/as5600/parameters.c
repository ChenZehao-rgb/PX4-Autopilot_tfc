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
 * AS5600 enable driver
 *
 * Run AS5600 driver automatically.
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
 *
 * @reboot_required true
 * @group Sensors
 * @unit us
 * @min 5000
 * @max 1000000
 */
PARAM_DEFINE_INT32(AS5600_POOL, 5000);

/**
 * AS5600 low-pass filter coefficient
 *
 * Coefficient for exponential moving average filter applied to RPM.
 * 0.0 = only use previous value (no update),
 * 1.0 = no filtering (use raw value directly).
 * Typical value: 0.2 to 0.5.
 *
 * @group Sensors
 * @decimal 2
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(AS5600_FILTER, 0.3f);

/**
 * AS5600 max plausible RPM
 *
 * Samples with absolute raw RPM above this limit are rejected as outliers.
 * Use this to suppress occasional wrap/aliasing glitches.
 *
 * @group Sensors
 * @unit rpm
 * @decimal 1
 * @min 10.0
 * @max 30000.0
 */
PARAM_DEFINE_FLOAT(AS5600_MAX_RPM, 5000.0f);

/**
 * AS5600 direction mode
 *
 * Optional direction constraint:
 * 0 = both directions allowed,
 * 1 = only positive RPM accepted,
 * -1 = only negative RPM accepted.
 *
 * @group Sensors
 * @min -1
 * @max 1
 */
PARAM_DEFINE_INT32(AS5600_DIR, 0);
