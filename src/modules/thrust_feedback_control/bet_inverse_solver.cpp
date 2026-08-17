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

#include "bet_inverse_solver.hpp"
#include "bet_aero_data.hpp"
#include "bet_lift_lookup.hpp"
#include "bet_residual_lookup.hpp"

#include <cmath>
#include <cstddef>

namespace thrust_feedback_control
{
namespace bet
{

namespace
{

static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kRadToDeg = 180.f / kPi;
static constexpr float kDegToRad = kPi / 180.f;
static constexpr float kRho = 1.225f;
static constexpr float kMu = 1.81e-5f;
static constexpr float kRotorRadiusM = 0.2286f;
static constexpr float kRotorAreaM2 = kPi * kRotorRadiusM * kRotorRadiusM;
static constexpr float kTurntableArmM = 1.5f;
static constexpr int kBladeCount = 2;
static constexpr int kNumPsi = 72;
static constexpr int kMaxInductionIterations = 80;
static constexpr float kLambdaTolerance = 1e-6f;
static constexpr float kDamping = 0.5f;
static constexpr float kCtInitialGuess = 0.011f;
static constexpr float kKxFixed = 1.2f;
static constexpr float kMuTSkewTolerance = 1e-9f;
static constexpr float kPostStallBlendDeg = 5.f;
static constexpr float kPostStallCd90 = 2.f;
static constexpr float kPostStallCdMin = 0.02f;
static constexpr float kSolveLiftToleranceN = 0.01f;
static constexpr float kSolveRpmTolerance = 1.f;
static constexpr int kMaxRpmSolveIterations = 32;

bool is_valid_common_input(float freestream_m_s, float alpha_deg)
{
	return std::isfinite(freestream_m_s) && std::isfinite(alpha_deg) && freestream_m_s >= 0.f;
}

bool is_valid_lift_input(float lift_n, float freestream_m_s, float alpha_deg)
{
	return std::isfinite(lift_n) && lift_n >= 0.f && is_valid_common_input(freestream_m_s, alpha_deg);
}

float constrain_float(float value, float min_value, float max_value)
{
	return fminf(fmaxf(value, min_value), max_value);
}

float smoothstep(float value)
{
	const float x = constrain_float(value, 0.f, 1.f);
	return x * x * (3.f - 2.f * x);
}

std::size_t lower_grid_index(const float *grid, std::size_t count, float value, float &weight)
{
	if (value <= grid[0]) {
		weight = 0.f;
		return 0;
	}

	if (value >= grid[count - 1]) {
		weight = 1.f;
		return count - 2;
	}

	for (std::size_t i = 0; i + 1 < count; ++i) {
		if (value <= grid[i + 1]) {
			const float denominator = grid[i + 1] - grid[i];
			weight = denominator > 0.f ? (value - grid[i]) / denominator : 0.f;
			return i;
		}
	}

	weight = 1.f;
	return count - 2;
}

float aero_value_at(const float *table, std::size_t airfoil_index, std::size_t alpha_index, std::size_t re_index)
{
	const std::size_t index = (airfoil_index * kNumBetAlpha + alpha_index) * kNumBetRe + re_index;
	return table[index];
}

float interpolate_aero_table(const float *table, std::size_t airfoil_index, float alpha_deg, float reynolds)
{
	const float alpha_clamped = constrain_float(alpha_deg, kBetAlphaGridDeg[0], kBetAlphaGridDeg[kNumBetAlpha - 1]);
	const float re_clamped = constrain_float(reynolds, kBetReGrid[0], kBetReGrid[kNumBetRe - 1]);

	float alpha_weight = 0.f;
	float re_weight = 0.f;
	const std::size_t alpha_index = lower_grid_index(kBetAlphaGridDeg, kNumBetAlpha, alpha_clamped, alpha_weight);
	const std::size_t re_index = lower_grid_index(kBetReGrid, kNumBetRe, re_clamped, re_weight);

	const float v00 = aero_value_at(table, airfoil_index, alpha_index, re_index);
	const float v10 = aero_value_at(table, airfoil_index, alpha_index + 1, re_index);
	const float v01 = aero_value_at(table, airfoil_index, alpha_index, re_index + 1);
	const float v11 = aero_value_at(table, airfoil_index, alpha_index + 1, re_index + 1);

	const float v0 = v00 + alpha_weight * (v10 - v00);
	const float v1 = v01 + alpha_weight * (v11 - v01);
	return v0 + re_weight * (v1 - v0);
}

void lookup_cl_cd(std::size_t airfoil_index, float alpha_deg, float reynolds, float &cl, float &cd)
{
	const float re_clamped = constrain_float(reynolds, kBetReGrid[0], kBetReGrid[kNumBetRe - 1]);
	const float alpha_min = kBetAlphaGridDeg[0];
	const float alpha_max = kBetAlphaGridDeg[kNumBetAlpha - 1];

	const float cl_table = interpolate_aero_table(kBetClTable, airfoil_index, alpha_deg, re_clamped);
	const float cd_table = interpolate_aero_table(kBetCdTable, airfoil_index, alpha_deg, re_clamped);
	const float cd0 = interpolate_aero_table(kBetCdTable, airfoil_index, 0.f, re_clamped);

	const float alpha_rad = alpha_deg * kDegToRad;
	const float sin_alpha = sinf(alpha_rad);
	const float cos_alpha = cosf(alpha_rad);
	const float cd0_effective = fmaxf(cd0, kPostStallCdMin);
	const float cl_stall = kPostStallCd90 * sin_alpha * cos_alpha;
	const float cd_stall = cd0_effective * cos_alpha * cos_alpha + kPostStallCd90 * sin_alpha * sin_alpha;

	const float low_weight = smoothstep((alpha_min - alpha_deg) / kPostStallBlendDeg);
	const float high_weight = smoothstep((alpha_deg - alpha_max) / kPostStallBlendDeg);
	const float post_stall_weight = fmaxf(low_weight, high_weight);

	cl = (1.f - post_stall_weight) * cl_table + post_stall_weight * cl_stall;
	cd = (1.f - post_stall_weight) * cd_table + post_stall_weight * cd_stall;
}

float lift_lookup_value_at(std::size_t alpha_index, std::size_t freestream_index, std::size_t rpm_index)
{
	const std::size_t index = (alpha_index * kNumBetLiftFreestream + freestream_index) * kNumBetLiftRpm + rpm_index;
	return kBetLiftLookupN[index];
}

bool table_input_in_range(float freestream_m_s, float alpha_deg)
{
	return freestream_m_s >= kBetLiftFreestreamGrid[0]
	       && freestream_m_s <= kBetLiftFreestreamGrid[kNumBetLiftFreestream - 1]
	       && alpha_deg >= kBetLiftAlphaGridDeg[0]
	       && alpha_deg <= kBetLiftAlphaGridDeg[kNumBetLiftAlpha - 1];
}

float interpolated_lift_profile_value(std::size_t rpm_index, std::size_t alpha_index, float alpha_weight,
				      std::size_t freestream_index, float freestream_weight)
{
	const float l00 = lift_lookup_value_at(alpha_index, freestream_index, rpm_index);
	const float l10 = lift_lookup_value_at(alpha_index + 1, freestream_index, rpm_index);
	const float l01 = lift_lookup_value_at(alpha_index, freestream_index + 1, rpm_index);
	const float l11 = lift_lookup_value_at(alpha_index + 1, freestream_index + 1, rpm_index);

	const float l0 = l00 + alpha_weight * (l10 - l00);
	const float l1 = l01 + alpha_weight * (l11 - l01);
	return l0 + freestream_weight * (l1 - l0);
}

float interpolated_bet_lookup_lift_n(float rpm, float freestream_m_s, float alpha_deg)
{
	rpm = constrain_float(rpm, kBetLiftRpmGrid[0], kBetLiftRpmGrid[kNumBetLiftRpm - 1]);
	freestream_m_s = constrain_float(freestream_m_s, kBetLiftFreestreamGrid[0],
				       kBetLiftFreestreamGrid[kNumBetLiftFreestream - 1]);
	alpha_deg = constrain_float(alpha_deg, kBetLiftAlphaGridDeg[0], kBetLiftAlphaGridDeg[kNumBetLiftAlpha - 1]);

	float rpm_weight = 0.f;
	float alpha_weight = 0.f;
	float freestream_weight = 0.f;
	const std::size_t rpm_index = lower_grid_index(kBetLiftRpmGrid, kNumBetLiftRpm, rpm, rpm_weight);
	const std::size_t alpha_index = lower_grid_index(kBetLiftAlphaGridDeg, kNumBetLiftAlpha, alpha_deg, alpha_weight);
	const std::size_t freestream_index = lower_grid_index(kBetLiftFreestreamGrid, kNumBetLiftFreestream,
						     freestream_m_s, freestream_weight);
	const float lift_low = interpolated_lift_profile_value(rpm_index, alpha_index, alpha_weight,
				 freestream_index, freestream_weight);
	const float lift_high = interpolated_lift_profile_value(rpm_index + 1, alpha_index, alpha_weight,
				  freestream_index, freestream_weight);
	return lift_low + rpm_weight * (lift_high - lift_low);
}

float corrected_lift_value_at(std::size_t alpha_index, std::size_t freestream_index, std::size_t rpm_index)
{
	const std::size_t index = (alpha_index * kNumBetResidualFreestream + freestream_index)
				  * kNumBetResidualRpm + rpm_index;
	return kBetCorrectedLiftLookupN[index];
}

float interpolated_corrected_lift_profile_value(std::size_t rpm_index, std::size_t alpha_index,
		float alpha_weight, std::size_t freestream_index, float freestream_weight)
{
	const float l00 = corrected_lift_value_at(alpha_index, freestream_index, rpm_index);
	const float l10 = corrected_lift_value_at(alpha_index + 1, freestream_index, rpm_index);
	const float l01 = corrected_lift_value_at(alpha_index, freestream_index + 1, rpm_index);
	const float l11 = corrected_lift_value_at(alpha_index + 1, freestream_index + 1, rpm_index);

	const float l0 = l00 + alpha_weight * (l10 - l00);
	const float l1 = l01 + alpha_weight * (l11 - l01);
	return l0 + freestream_weight * (l1 - l0);
}

float blended_corrected_lift_profile_value(std::size_t residual_rpm_index, std::size_t alpha_index,
		float alpha_weight, std::size_t freestream_index, float freestream_weight,
		float freestream_m_s, float alpha_deg, float correction_scale)
{
	const float rpm = kBetResidualRpmGrid[residual_rpm_index];
	const float bet_lift = interpolated_bet_lookup_lift_n(rpm, freestream_m_s, alpha_deg);
	const float corrected_lift = interpolated_corrected_lift_profile_value(residual_rpm_index, alpha_index,
				     alpha_weight, freestream_index, freestream_weight);
	return bet_lift + correction_scale * (corrected_lift - bet_lift);
}

} // namespace

float compute_lift_n(float rpm, float freestream_m_s, float alpha_deg)
{
	if (!std::isfinite(rpm) || rpm <= 0.f || !is_valid_common_input(freestream_m_s, alpha_deg)) {
		return 0.f;
	}

	const float omega = rpm * kPi / 30.f;
	const float alpha_rad = alpha_deg * kDegToRad;
	const float omega_a = freestream_m_s / kTurntableArmM;
	const float mu_a = freestream_m_s * cosf(alpha_rad) / (omega * kRotorRadiusM + 1e-12f);
	const float mu_t = freestream_m_s * sinf(alpha_rad) / (omega * kRotorRadiusM + 1e-12f);
	const float k_x = fabsf(mu_t) > kMuTSkewTolerance ? kKxFixed : 0.f;

	float lambda_i_0 = sqrtf(fmaxf(kCtInitialGuess / 2.f, 1e-8f));
	float thrust_total = 0.f;
	float dthrust_dr[kNumBetSections] {};

	for (int iteration = 0; iteration < kMaxInductionIterations; ++iteration) {
		for (std::size_t section_index = 0; section_index < kNumBetSections; ++section_index) {
			const BetSection &section = kBetSections[section_index];
			float dthrust_sum = 0.f;

			for (int psi_index = 0; psi_index < kNumPsi; ++psi_index) {
				const float psi = 2.f * kPi * static_cast<float>(psi_index) / static_cast<float>(kNumPsi);
				const float cos_psi = cosf(psi);
				const float sin_psi = sinf(psi);
				const float v_free = omega_a * (kTurntableArmM - section.radius_m * cos_psi);
				float lambda_i = lambda_i_0 * (1.f + k_x * section.r_over_R * sin_psi);
				lambda_i = fmaxf(lambda_i, 0.f);
				const float induced_velocity = lambda_i * omega * kRotorRadiusM;
				const float v_a = v_free * cosf(alpha_rad) + induced_velocity;
				const float v_t = omega * section.radius_m - v_free * sinf(alpha_rad) * cos_psi;
				const float v_r = sqrtf(v_a * v_a + v_t * v_t + 1e-12f);
				const float phi = atan2f(v_a, v_t);
				const float alpha_b_deg = (section.theta_rad - phi) * kRadToDeg;
				const float reynolds = kRho * v_r * section.chord_m / kMu;

				float cl = 0.f;
				float cd = 0.f;
				lookup_cl_cd(section.airfoil_index, alpha_b_deg, reynolds, cl, cd);

				dthrust_sum += 0.5f * kRho * v_r * v_r
						* (cl * cosf(phi) - cd * sinf(phi))
						* section.chord_m;
			}

			dthrust_dr[section_index] = dthrust_sum / static_cast<float>(kNumPsi);
		}

		float thrust_blade = 0.f;

		for (std::size_t i = 0; i + 1 < kNumBetSections; ++i) {
			const float dr = kBetSections[i + 1].radius_m - kBetSections[i].radius_m;
			thrust_blade += 0.5f * (dthrust_dr[i] + dthrust_dr[i + 1]) * dr;
		}

		thrust_total = static_cast<float>(kBladeCount) * thrust_blade;
		const float ct_raw = thrust_total / (kRho * kRotorAreaM2 * omega * omega * kRotorRadiusM * kRotorRadiusM);
		const float ct = fmaxf(ct_raw, 0.f);
		const float lambda_0 = mu_a + lambda_i_0;
		const float denominator = hypotf(mu_t, lambda_0);
		const float lambda_i_0_new = denominator > 1e-8f ? ct / (2.f * denominator) : sqrtf(fmaxf(ct / 2.f, 0.f));
		const float error = fabsf(lambda_i_0_new - lambda_i_0);
		lambda_i_0 = kDamping * lambda_i_0 + (1.f - kDamping) * lambda_i_0_new;

		if (error < kLambdaTolerance) {
			break;
		}
	}

	return thrust_total;
}

BetRpmSolution solve_rpm_for_lift_n(float lift_n, float freestream_m_s, float alpha_deg)
{
	BetRpmSolution solution{};

	if (!is_valid_lift_input(lift_n, freestream_m_s, alpha_deg)) {
		solution.status = BetRpmStatus::InvalidInput;
		return solution;
	}

	if (lift_n <= 0.f) {
		solution.status = BetRpmStatus::Ok;
		return solution;
	}

	const float max_lift_n = compute_lift_n(kBetMaxRpm, freestream_m_s, alpha_deg);

	if (lift_n > max_lift_n + kSolveLiftToleranceN) {
		solution.rpm = kBetMaxRpm;
		solution.achieved_lift_n = max_lift_n;
		solution.status = BetRpmStatus::AboveRpmLimit;
		return solution;
	}

	float rpm_low = kBetMinRpm;
	float rpm_high = kBetMaxRpm;
	float rpm_mid = 0.f;
	float lift_mid = 0.f;

	for (int iteration = 0; iteration < kMaxRpmSolveIterations; ++iteration) {
		rpm_mid = 0.5f * (rpm_low + rpm_high);
		lift_mid = compute_lift_n(rpm_mid, freestream_m_s, alpha_deg);
		solution.iterations = iteration + 1;

		if (fabsf(lift_mid - lift_n) <= kSolveLiftToleranceN || (rpm_high - rpm_low) <= kSolveRpmTolerance) {
			break;
		}

		if (lift_mid < lift_n) {
			rpm_low = rpm_mid;
		} else {
			rpm_high = rpm_mid;
		}
	}

	solution.rpm = rpm_mid;
	solution.achieved_lift_n = lift_mid;
	solution.status = BetRpmStatus::Ok;
	return solution;
}

BetRpmSolution solve_rpm_for_lift_kgf(float lift_kgf, float freestream_m_s, float alpha_deg)
{
	return solve_rpm_for_lift_n(lift_kgf * kBetGravityMps2, freestream_m_s, alpha_deg);
}

BetRpmSolution lookup_rpm_for_lift_n(float lift_n, float freestream_m_s, float alpha_deg)
{
	BetRpmSolution solution{};

	if (!is_valid_lift_input(lift_n, freestream_m_s, alpha_deg)) {
		solution.status = BetRpmStatus::InvalidInput;
		return solution;
	}

	const bool input_clamped = !table_input_in_range(freestream_m_s, alpha_deg);
	freestream_m_s = fminf(fmaxf(freestream_m_s, kBetLiftFreestreamGrid[0]),
				kBetLiftFreestreamGrid[kNumBetLiftFreestream - 1]);
	alpha_deg = fminf(fmaxf(alpha_deg, kBetLiftAlphaGridDeg[0]),
			  kBetLiftAlphaGridDeg[kNumBetLiftAlpha - 1]);
	const BetRpmStatus lookup_status = input_clamped ? BetRpmStatus::OutOfTableRange : BetRpmStatus::Ok;

	float alpha_weight = 0.f;
	float freestream_weight = 0.f;
	const std::size_t alpha_index = lower_grid_index(kBetLiftAlphaGridDeg, kNumBetLiftAlpha, alpha_deg, alpha_weight);
	const std::size_t freestream_index = lower_grid_index(kBetLiftFreestreamGrid, kNumBetLiftFreestream, freestream_m_s,
					     freestream_weight);

	float previous_lift = interpolated_lift_profile_value(0, alpha_index, alpha_weight, freestream_index, freestream_weight);

	if (lift_n <= previous_lift) {
		solution.rpm = kBetLiftRpmGrid[0];
		solution.achieved_lift_n = previous_lift;
		solution.status = lookup_status;
		return solution;
	}

	bool non_monotonic = false;

	for (std::size_t rpm_index = 1; rpm_index < kNumBetLiftRpm; ++rpm_index) {
		const float current_lift = interpolated_lift_profile_value(rpm_index, alpha_index, alpha_weight, freestream_index,
					   freestream_weight);

		if (current_lift + 1e-4f < previous_lift) {
			non_monotonic = true;
		}

		if (lift_n >= previous_lift && lift_n <= current_lift) {
			const float lift_span = current_lift - previous_lift;
			const float weight = lift_span > 1e-6f ? (lift_n - previous_lift) / lift_span : 0.f;
			const float rpm_low = kBetLiftRpmGrid[rpm_index - 1];
			const float rpm_high = kBetLiftRpmGrid[rpm_index];

			solution.rpm = rpm_low + weight * (rpm_high - rpm_low);
			solution.achieved_lift_n = lift_n;
			solution.iterations = static_cast<int>(rpm_index);
			solution.status = lookup_status;
			return solution;
		}

		previous_lift = current_lift;
	}

	solution.rpm = kBetLiftRpmGrid[kNumBetLiftRpm - 1];
	solution.achieved_lift_n = previous_lift;
	solution.iterations = static_cast<int>(kNumBetLiftRpm - 1);
	solution.status = non_monotonic ? BetRpmStatus::NonMonotonicTable : BetRpmStatus::AboveRpmLimit;
	return solution;
}

BetRpmSolution lookup_rpm_for_lift_kgf(float lift_kgf, float freestream_m_s, float alpha_deg)
{
	return lookup_rpm_for_lift_n(lift_kgf * kBetGravityMps2, freestream_m_s, alpha_deg);
}

BetRpmSolution lookup_corrected_rpm_for_lift_n(float lift_n, float freestream_m_s, float alpha_deg,
		float correction_scale)
{
	BetRpmSolution solution{};

	if (!is_valid_lift_input(lift_n, freestream_m_s, alpha_deg) || !std::isfinite(correction_scale)) {
		solution.status = BetRpmStatus::InvalidInput;
		return solution;
	}

	correction_scale = constrain_float(correction_scale, 0.f, 1.f);

	if (correction_scale <= 1e-6f || lift_n <= 0.f) {
		return lookup_rpm_for_lift_n(lift_n, freestream_m_s, alpha_deg);
	}

	const bool input_clamped = freestream_m_s < kBetResidualFreestreamGrid[0]
				   || freestream_m_s > kBetResidualFreestreamGrid[kNumBetResidualFreestream - 1]
				   || alpha_deg < kBetResidualAlphaGridDeg[0]
				   || alpha_deg > kBetResidualAlphaGridDeg[kNumBetResidualAlpha - 1];
	freestream_m_s = constrain_float(freestream_m_s, kBetResidualFreestreamGrid[0],
				       kBetResidualFreestreamGrid[kNumBetResidualFreestream - 1]);
	alpha_deg = constrain_float(alpha_deg, kBetResidualAlphaGridDeg[0],
				  kBetResidualAlphaGridDeg[kNumBetResidualAlpha - 1]);

	float alpha_weight = 0.f;
	float freestream_weight = 0.f;
	const std::size_t alpha_index = lower_grid_index(kBetResidualAlphaGridDeg, kNumBetResidualAlpha,
						       alpha_deg, alpha_weight);
	const std::size_t freestream_index = lower_grid_index(kBetResidualFreestreamGrid,
						    kNumBetResidualFreestream, freestream_m_s,
						    freestream_weight);

	float previous_lift = blended_corrected_lift_profile_value(0, alpha_index, alpha_weight,
			      freestream_index, freestream_weight, freestream_m_s, alpha_deg, correction_scale);

	for (std::size_t rpm_index = 1; rpm_index < kNumBetResidualRpm; ++rpm_index) {
		const float current_lift = blended_corrected_lift_profile_value(rpm_index, alpha_index, alpha_weight,
				   freestream_index, freestream_weight, freestream_m_s, alpha_deg, correction_scale);

		if (current_lift + 1e-4f < previous_lift) {
			solution.status = BetRpmStatus::NonMonotonicTable;
			return solution;
		}

		if (lift_n >= previous_lift && lift_n <= current_lift) {
			const float lift_span = current_lift - previous_lift;
			const float weight = lift_span > 1e-6f ? (lift_n - previous_lift) / lift_span : 0.f;
			const float rpm_low = kBetResidualRpmGrid[rpm_index - 1];
			const float rpm_high = kBetResidualRpmGrid[rpm_index];
			solution.rpm = rpm_low + weight * (rpm_high - rpm_low);
			solution.achieved_lift_n = lift_n;
			solution.iterations = static_cast<int>(rpm_index);
			solution.status = input_clamped ? BetRpmStatus::OutOfTableRange : BetRpmStatus::Ok;
			return solution;
		}

		previous_lift = current_lift;
	}

	// Do not extrapolate the experimental correction beyond 3000--5000 rpm.
	// Keep the original BEMT solution available as a deterministic fallback.
	solution = lookup_rpm_for_lift_n(lift_n, freestream_m_s, alpha_deg);

	if (solution.status == BetRpmStatus::Ok) {
		solution.status = input_clamped ? BetRpmStatus::OutOfTableRange : BetRpmStatus::ResidualFallback;
	}

	return solution;
}

BetRpmSolution lookup_corrected_rpm_for_lift_kgf(float lift_kgf, float freestream_m_s, float alpha_deg,
		float correction_scale)
{
	return lookup_corrected_rpm_for_lift_n(lift_kgf * kBetGravityMps2, freestream_m_s, alpha_deg,
		       correction_scale);
}

} // namespace bet
} // namespace thrust_feedback_control
