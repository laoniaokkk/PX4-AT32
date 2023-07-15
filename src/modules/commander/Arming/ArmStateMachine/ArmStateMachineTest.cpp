/****************************************************************************
 *
 *   Copyright (C) 2022 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include "ArmStateMachine.hpp"

TEST(ArmStateMachineTest, ArmingStateTransitionTest)
{
	ArmStateMachine arm_state_machine;

	// These are the critical values from vehicle_status_s and actuator_armed_s which must be primed
	// to simulate machine state prior to testing an arming state transition. This structure is also
	// use to represent the expected machine state after the transition has been requested.
	typedef struct {
		arming_state_t  arming_state;   // vehicle_status_s.arming_state
		bool            armed;          // actuator_armed_s.armed
	} ArmingTransitionVolatileState_t;

	// This structure represents a test case for arming_state_transition. It contains the machine
	// state prior to transition, the requested state to transition to and finally the expected
	// machine state after transition.
	typedef struct {
		const char                     *assertMsg;                              // Text to show when test case fails
		ArmingTransitionVolatileState_t current_state;                          // Machine state prior to transition
		hil_state_t                     hil_state;                              // Current vehicle_status_s.hil_state
		bool                            safety_button_available;                // Current safety_s.safety_button_available
		bool                            safety_off;                             // Current safety_s.safety_off
		arming_state_t                  requested_state;                        // Requested arming state to transition to
		ArmingTransitionVolatileState_t expected_state;                         // Expected machine state after transition
		transition_result_t             expected_transition_result;             // Expected result from arming_state_transition
	} ArmingTransitionTest_t;

	// We use these defines so that our test cases are more readable
	static constexpr bool ATT_ARMED = true;
	static constexpr bool ATT_DISARMED = false;
	static constexpr bool ATT_SAFETY_AVAILABLE = true;
	static constexpr bool ATT_SAFETY_NOT_AVAILABLE = true;
	static constexpr bool ATT_SAFETY_OFF = true;
	static constexpr bool ATT_SAFETY_ON = false;

	// These are test cases for arming_state_transition
	static const ArmingTransitionTest_t rgArmingTransitionTests[] = {
		// TRANSITION_NOT_CHANGED tests

		{
			"no transition: identical states",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, TRANSITION_NOT_CHANGED
		},

		// TRANSITION_CHANGED tests

		// Check all basic valid transitions, these don't require special state in vehicle_status_t or safety_s

		{
			"transition: init to standby",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: init to standby error",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: init to reboot",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: standby to init",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: standby to standby error",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY_ERROR,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: standby to reboot",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: armed to standby",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: standby error to reboot",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED}, TRANSITION_CHANGED
		},

		{
			"transition: in air restore to armed",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, TRANSITION_CHANGED
		},

		{
			"transition: in air restore to reboot",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED}, TRANSITION_CHANGED
		},

		// hil on tests, standby error to standby not normally allowed

		{
			"transition: standby error to standby, hil on",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, vehicle_status_s::HIL_STATE_ON, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, TRANSITION_CHANGED
		},

		// Safety button arming tests

		{
			"transition: standby to armed, no safety button",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, vehicle_status_s::HIL_STATE_ON, ATT_SAFETY_NOT_AVAILABLE, ATT_SAFETY_OFF,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, TRANSITION_CHANGED
		},

		{
			"transition: standby to armed, safety button off",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, vehicle_status_s::HIL_STATE_ON, ATT_SAFETY_AVAILABLE, ATT_SAFETY_OFF,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, TRANSITION_CHANGED
		},

		// TRANSITION_DENIED tests

		// Check some important basic invalid transitions, these don't require special state in vehicle_status_t or safety_s

		{
			"no transition: init to armed",
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_INIT, ATT_DISARMED}, TRANSITION_DENIED
		},

		{
			"no transition: armed to init",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_INIT,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, TRANSITION_DENIED
		},

		{
			"no transition: armed to reboot",
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_SHUTDOWN,
			{ vehicle_status_s::ARMING_STATE_ARMED, ATT_ARMED}, TRANSITION_DENIED
		},

		{
			"no transition: standby error to armed",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, TRANSITION_DENIED
		},

		{
			"no transition: standby error to standby",
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_STANDBY_ERROR, ATT_DISARMED}, TRANSITION_DENIED
		},

		{
			"no transition: reboot to armed",
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_SHUTDOWN, ATT_DISARMED}, TRANSITION_DENIED
		},

		{
			"no transition: in air restore to standby",
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_STANDBY,
			{ vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE, ATT_DISARMED}, TRANSITION_DENIED
		},

		// Safety button arming tests

		{
			"no transition: init to armed, safety button on",
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, vehicle_status_s::HIL_STATE_OFF, ATT_SAFETY_AVAILABLE, ATT_SAFETY_ON,
			vehicle_status_s::ARMING_STATE_ARMED,
			{ vehicle_status_s::ARMING_STATE_STANDBY, ATT_DISARMED}, TRANSITION_DENIED
		},
	};

	struct vehicle_status_s status {};
	struct actuator_armed_s armed {};

	size_t cArmingTransitionTests = sizeof(rgArmingTransitionTests) / sizeof(rgArmingTransitionTests[0]);

	for (size_t i = 0; i < cArmingTransitionTests; i++) {
		const ArmingTransitionTest_t *test = &rgArmingTransitionTests[i];

		// Setup initial machine state
		arm_state_machine.forceArmState(test->current_state.arming_state);
		status.hil_state = test->hil_state;

		HealthAndArmingChecks health_and_arming_checks(nullptr, status);

		// Attempt transition
		transition_result_t result = arm_state_machine.arming_state_transition(
						     status,
						     test->requested_state,
						     armed,
						     health_and_arming_checks,
						     true /* enable pre-arm checks */,
						     nullptr /* no mavlink_log_pub */,
						     arm_disarm_reason_t::unit_test);

		// Validate result of transition
		EXPECT_EQ(result, test->expected_transition_result) << test->assertMsg;
		EXPECT_EQ(arm_state_machine.getArmState(), test->expected_state.arming_state) << test->assertMsg;
		EXPECT_EQ(arm_state_machine.isArmed(), test->expected_state.armed) << test->assertMsg;
	}
}
