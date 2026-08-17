############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

px4_add_library(thrust_feedback_bet
        bet_aero_data.cpp
        bet_aero_data.hpp
        bet_inverse_solver.cpp
        bet_inverse_solver.hpp
        bet_lift_lookup.cpp
        bet_lift_lookup.hpp
        bet_residual_lookup.cpp
        bet_residual_lookup.hpp
)
target_include_directories(thrust_feedback_bet PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_module(
    MODULE modules__thrust_feedback_control
    MAIN thrust_feedback_control
    COMPILE_FLAGS
    SRCS
        thrust_feedback_control_bemt.cpp
        thrust_feedback_control_start.cpp
        thrust_kalman_filter.cpp
    DEPENDS
        thrust_feedback_bet
    )

px4_add_unit_gtest(SRC bet_inverse_solver_test.cpp LINKLIBS thrust_feedback_bet)
