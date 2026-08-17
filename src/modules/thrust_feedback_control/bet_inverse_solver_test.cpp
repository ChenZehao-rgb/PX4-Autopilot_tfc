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
#include "bet_residual_lookup.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace thrust_feedback_control::bet;

namespace
{

struct LiftReference {
	float rpm;
	float freestream_m_s;
	float alpha_deg;
	float lift_n;
};

} // namespace

TEST(BetInverseSolver, HoverReference)
{
	EXPECT_NEAR(compute_lift_n(3000.f, 0.f, 0.f), 11.328709f, 0.08f);
}

TEST(BetInverseSolver, HoverSweepReferences)
{
	const LiftReference references[] {
		{500.f, 0.f, 0.f, 0.240587f},
		{1000.f, 0.f, 0.f, 1.213288f},
		{1500.f, 0.f, 0.f, 2.773796f},
		{2000.f, 0.f, 0.f, 5.048944f},
		{2500.f, 0.f, 0.f, 7.909288f},
		{3000.f, 0.f, 0.f, 11.328709f},
		{3500.f, 0.f, 0.f, 15.281203f},
		{4000.f, 0.f, 0.f, 19.865563f},
		{4500.f, 0.f, 0.f, 25.161694f},
		{5000.f, 0.f, 0.f, 31.092247f},
		{5500.f, 0.f, 0.f, 37.684715f},
		{6000.f, 0.f, 0.f, 44.789238f},
	};

	for (const LiftReference &reference : references) {
		EXPECT_NEAR(compute_lift_n(reference.rpm, reference.freestream_m_s, reference.alpha_deg),
			    reference.lift_n, 0.12f) << "rpm=" << reference.rpm;
	}
}

TEST(BetInverseSolver, NonzeroFreestreamReferences)
{
	const LiftReference references[] {
		{3000.f, 5.085f, 0.f, 7.162566f},
		{3000.f, 10.17f, 0.f, 1.327051f},
		{3000.f, 5.085f, 45.f, 9.011067f},
		{3000.f, 10.17f, 45.f, 5.715326f},
		{3000.f, 5.085f, 90.f, 12.834167f},
		{3000.f, 10.17f, 90.f, 15.519529f},
		{4000.f, 5.085f, 0.f, 14.731564f},
		{4000.f, 10.17f, 0.f, 7.859968f},
		{4000.f, 5.085f, 45.f, 16.985216f},
		{4000.f, 10.17f, 45.f, 13.383611f},
		{4000.f, 5.085f, 90.f, 21.589363f},
		{4000.f, 10.17f, 90.f, 25.209679f},
		{5000.f, 5.085f, 0.f, 24.858644f},
		{5000.f, 10.17f, 0.f, 16.888598f},
		{5000.f, 5.085f, 45.f, 27.496485f},
		{5000.f, 10.17f, 45.f, 23.337374f},
		{5000.f, 5.085f, 90.f, 32.864838f},
		{5000.f, 10.17f, 90.f, 36.941185f},
	};

	for (const LiftReference &reference : references) {
		EXPECT_NEAR(compute_lift_n(reference.rpm, reference.freestream_m_s, reference.alpha_deg),
			    reference.lift_n, 0.18f)
				<< "rpm=" << reference.rpm
				<< " freestream=" << reference.freestream_m_s
				<< " alpha=" << reference.alpha_deg;
	}
}

TEST(BetInverseSolver, ExactInverseRecoversRpm)
{
	const float lift_n = compute_lift_n(3500.f, 5.085f, 45.f);
	const BetRpmSolution solution = solve_rpm_for_lift_n(lift_n, 5.085f, 45.f);

	EXPECT_EQ(solution.status, BetRpmStatus::Ok);
	EXPECT_NEAR(solution.rpm, 3500.f, 5.f);
	EXPECT_NEAR(solution.achieved_lift_n, lift_n, 0.03f);
}

TEST(BetInverseSolver, KgfInverseUsesGravityConversion)
{
	const BetRpmSolution solution_n = solve_rpm_for_lift_n(kBetGravityMps2, 0.f, 0.f);
	const BetRpmSolution solution_kgf = solve_rpm_for_lift_kgf(1.f, 0.f, 0.f);

	EXPECT_EQ(solution_kgf.status, BetRpmStatus::Ok);
	EXPECT_NEAR(solution_kgf.rpm, solution_n.rpm, 1.f);
}

TEST(BetInverseSolver, ZeroAndAboveLimitTargets)
{
	const BetRpmSolution zero = solve_rpm_for_lift_n(0.f, 0.f, 0.f);
	EXPECT_EQ(zero.status, BetRpmStatus::Ok);
	EXPECT_NEAR(zero.rpm, 0.f, 1e-3f);

	const BetRpmSolution high = solve_rpm_for_lift_n(1000.f, 0.f, 0.f);
	EXPECT_EQ(high.status, BetRpmStatus::AboveRpmLimit);
	EXPECT_NEAR(high.rpm, kBetMaxRpm, 1e-3f);
}

TEST(BetInverseSolver, InvalidInputs)
{
	EXPECT_EQ(solve_rpm_for_lift_n(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f).status,
		  BetRpmStatus::InvalidInput);
	EXPECT_EQ(solve_rpm_for_lift_n(1.f, -1.f, 0.f).status, BetRpmStatus::InvalidInput);
	EXPECT_EQ(solve_rpm_for_lift_n(-1.f, 0.f, 0.f).status, BetRpmStatus::InvalidInput);
}

TEST(BetInverseSolver, TableLookup)
{
	const float lift_n = compute_lift_n(3000.f, 0.f, 0.f);
	const BetRpmSolution solution = lookup_rpm_for_lift_n(lift_n, 0.f, 0.f);

	EXPECT_EQ(solution.status, BetRpmStatus::Ok);
	EXPECT_NEAR(solution.rpm, 3000.f, 2.f);
}

TEST(BetInverseSolver, TableLookupRangeAndLimitStatus)
{
	const BetRpmSolution boundary = lookup_rpm_for_lift_n(12.485f, 10.17f, 0.f);
	const BetRpmSolution clamped = lookup_rpm_for_lift_n(12.485f, 10.177f, 0.f);

	ASSERT_EQ(boundary.status, BetRpmStatus::Ok);
	EXPECT_EQ(clamped.status, BetRpmStatus::OutOfTableRange);
	EXPECT_NEAR(clamped.rpm, boundary.rpm, 1e-3f);
	EXPECT_NEAR(clamped.achieved_lift_n, boundary.achieved_lift_n, 1e-3f);
	EXPECT_EQ(lookup_rpm_for_lift_n(1000.f, 0.f, 0.f).status, BetRpmStatus::AboveRpmLimit);
}

TEST(BetInverseSolver, CorrectedAxial15NewtonReferences)
{
	struct CorrectedReference {
		float freestream_m_s;
		float rpm;
	};

	const CorrectedReference references[] {
		{0.f, 3591.7177f},
		{5.10f, 4064.7877f},
		{10.20f, 4622.8988f},
	};

	for (const CorrectedReference &reference : references) {
		const BetRpmSolution solution = lookup_corrected_rpm_for_lift_n(
				15.f, reference.freestream_m_s, 0.f, 1.f);
		EXPECT_EQ(solution.status, BetRpmStatus::Ok);
		EXPECT_NEAR(solution.rpm, reference.rpm, 0.2f)
				<< "freestream=" << reference.freestream_m_s;
		EXPECT_NEAR(solution.achieved_lift_n, 15.f, 1e-4f);
	}
}

TEST(BetInverseSolver, CorrectedLookupZeroScaleIsExactRollback)
{
	const BetRpmSolution baseline = lookup_rpm_for_lift_n(15.f, 5.10f, 0.f);
	const BetRpmSolution rollback = lookup_corrected_rpm_for_lift_n(15.f, 5.10f, 0.f, 0.f);

	EXPECT_EQ(rollback.status, baseline.status);
	EXPECT_FLOAT_EQ(rollback.rpm, baseline.rpm);
	EXPECT_FLOAT_EQ(rollback.achieved_lift_n, baseline.achieved_lift_n);
}

TEST(BetInverseSolver, CorrectedLookupFallsBackOutsideRpmEnvelope)
{
	const BetRpmSolution solution = lookup_corrected_rpm_for_lift_n(5.f, 0.f, 0.f, 1.f);
	const BetRpmSolution baseline = lookup_rpm_for_lift_n(5.f, 0.f, 0.f);

	EXPECT_EQ(solution.status, BetRpmStatus::ResidualFallback);
	EXPECT_FLOAT_EQ(solution.rpm, baseline.rpm);
	EXPECT_FLOAT_EQ(solution.achieved_lift_n, baseline.achieved_lift_n);
}

TEST(BetInverseSolver, CorrectedLookupReportsClampedInput)
{
	const BetRpmSolution solution = lookup_corrected_rpm_for_lift_n(15.f, 10.3f, 0.f, 1.f);

	EXPECT_EQ(solution.status, BetRpmStatus::OutOfTableRange);
	EXPECT_NEAR(solution.rpm, 4622.8988f, 0.2f);
}

TEST(BetInverseSolver, CorrectedExperimentalTableIsFiniteAndRpmMonotonic)
{
	for (std::size_t alpha = 0; alpha < kNumBetResidualAlpha; ++alpha) {
		for (std::size_t freestream = 0; freestream < kNumBetResidualFreestream; ++freestream) {
			float previous = -std::numeric_limits<float>::infinity();

			for (std::size_t rpm = 0; rpm < kNumBetResidualRpm; ++rpm) {
				const std::size_t index = (alpha * kNumBetResidualFreestream + freestream)
						  * kNumBetResidualRpm + rpm;
				const float lift = kBetCorrectedLiftLookupN[index];
				EXPECT_TRUE(std::isfinite(lift));
				EXPECT_GT(lift, previous);
				previous = lift;
			}
		}
	}
}
