/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

namespace thrust_feedback_control
{
namespace bet
{

enum class BetRpmStatus {
	Ok = 0,
	InvalidInput,
	OutOfTableRange,
	AboveRpmLimit,
	NonMonotonicTable,
};

struct BetRpmSolution {
	float rpm{0.f};
	float achieved_lift_n{0.f};
	int iterations{0};
	BetRpmStatus status{BetRpmStatus::InvalidInput};
};

static constexpr float kBetGravityMps2 = 9.80665f;
static constexpr float kBetMinRpm = 0.f;
static constexpr float kBetMaxRpm = 6000.f;

float compute_lift_n(float rpm, float freestream_m_s, float alpha_deg);

BetRpmSolution solve_rpm_for_lift_n(float lift_n, float freestream_m_s, float alpha_deg);
BetRpmSolution solve_rpm_for_lift_kgf(float lift_kgf, float freestream_m_s, float alpha_deg);

BetRpmSolution lookup_rpm_for_lift_n(float lift_n, float freestream_m_s, float alpha_deg);
BetRpmSolution lookup_rpm_for_lift_kgf(float lift_kgf, float freestream_m_s, float alpha_deg);

} // namespace bet
} // namespace thrust_feedback_control
