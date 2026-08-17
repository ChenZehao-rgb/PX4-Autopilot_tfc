/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>

namespace thrust_feedback_control
{
namespace bet
{

// Generated from the lift-calibrated 2026-08-18 turntable data. The angle is
// the BEMT disk angle: 0 deg is pure axial flow.
static constexpr std::size_t kNumBetResidualRpm = 3;
static constexpr std::size_t kNumBetResidualFreestream = 11;
static constexpr std::size_t kNumBetResidualAlpha = 7;

extern const float kBetResidualRpmGrid[kNumBetResidualRpm];
extern const float kBetResidualFreestreamGrid[kNumBetResidualFreestream];
extern const float kBetResidualAlphaGridDeg[kNumBetResidualAlpha];
extern const float kBetCorrectedLiftLookupN[
	kNumBetResidualAlpha * kNumBetResidualFreestream * kNumBetResidualRpm];

} // namespace bet
} // namespace thrust_feedback_control
