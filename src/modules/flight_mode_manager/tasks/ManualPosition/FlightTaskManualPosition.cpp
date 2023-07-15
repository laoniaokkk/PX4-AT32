/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskManualPosition.cpp
 */

#include "FlightTaskManualPosition.hpp"
#include <mathlib/mathlib.h>
#include <float.h>

using namespace matrix;

bool FlightTaskManualPosition::updateInitialize()
{
	bool ret = FlightTaskManualAltitude::updateInitialize();
	// require valid position / velocity in xy
	return ret && Vector2f(_position).isAllFinite() && Vector2f(_velocity).isAllFinite();
}

bool FlightTaskManualPosition::activate(const trajectory_setpoint_s &last_setpoint)
{
	// all requirements from altitude-mode still have to hold
	bool ret = FlightTaskManualAltitude::activate(last_setpoint);

	_position_setpoint(0) = _position(0);
	_position_setpoint(1) = _position(1);
	_velocity_setpoint(0) = _velocity_setpoint(1) = 0.0f;

	// for position-controlled mode, we need a valid position and velocity state
	// in NE-direction
	return ret;
}

void FlightTaskManualPosition::_scaleSticks()
{
	/* Use same scaling as for FlightTaskManualAltitude */
	FlightTaskManualAltitude::_scaleSticks();

	Vector2f stick_xy = _sticks.getPitchRollExpo();

	Sticks::limitStickUnitLengthXY(stick_xy);

	if (_param_mpc_vel_man_side.get() >= 0.f) {
		stick_xy(1) *= _param_mpc_vel_man_side.get() / _param_mpc_vel_manual.get();
	}

	if ((_param_mpc_vel_man_back.get() >= 0.f) && (stick_xy(0) < 0.f)) {
		stick_xy(0) *= _param_mpc_vel_man_back.get() / _param_mpc_vel_manual.get();
	}

	const float max_speed_from_estimator = _sub_vehicle_local_position.get().vxy_max;

	float velocity_scale = _param_mpc_vel_manual.get();

	if (PX4_ISFINITE(max_speed_from_estimator)) {
		// Constrain with optical flow limit but leave 0.3 m/s for repositioning
		velocity_scale = math::constrain(velocity_scale, 0.3f, max_speed_from_estimator);
	}

	// scale velocity to its maximum limits
	Vector2f vel_sp_xy = stick_xy * velocity_scale;

	// Rotate setpoint into local frame
	Sticks::rotateIntoHeadingFrameXY(vel_sp_xy, _yaw, _yaw_setpoint);

	// collision prevention
	if (_collision_prevention.is_active()) {
		_collision_prevention.modifySetpoint(vel_sp_xy, velocity_scale, _position.xy(), _velocity.xy());
	}

	_velocity_setpoint.xy() = vel_sp_xy;
}

void FlightTaskManualPosition::_updateXYlock()
{
	/* If position lock is not active, position setpoint is set to NAN.*/
	const float vel_xy_norm = Vector2f(_velocity).length();
	const bool apply_brake = Vector2f(_velocity_setpoint).length() < FLT_EPSILON;
	const bool stopped = (_param_mpc_hold_max_xy.get() < FLT_EPSILON || vel_xy_norm < _param_mpc_hold_max_xy.get());

	if (apply_brake && stopped && !Vector2f(_position_setpoint).isAllFinite()) {
		_position_setpoint.xy() = _position.xy();

	} else if (Vector2f(_position_setpoint).isAllFinite() && apply_brake) {
		// Position is locked but check if a reset event has happened.
		// We will shift the setpoints.
		if (_sub_vehicle_local_position.get().xy_reset_counter != _reset_counter) {
			_position_setpoint.xy() = _position.xy();
			_reset_counter = _sub_vehicle_local_position.get().xy_reset_counter;
		}

	} else {
		/* don't lock*/
		_position_setpoint(0) = NAN;
		_position_setpoint(1) = NAN;
	}
}

void FlightTaskManualPosition::_updateSetpoints()
{
	FlightTaskManualAltitude::_updateSetpoints(); // needed to get yaw and setpoints in z-direction
	_acceleration_setpoint.setNaN(); // don't use the horizontal setpoints from FlightTaskAltitude

	_updateXYlock(); // check for position lock

	_weathervane.update();

	if (_weathervane.isActive()) {
		_yaw_setpoint = NAN;

		// only enable the weathervane to change the yawrate when position lock is active (and thus the pos. sp. aren't NAN)
		if (Vector2f(_position_setpoint).isAllFinite()) {
			// vehicle is steady
			_yawspeed_setpoint += _weathervane.getWeathervaneYawrate();
		}
	}
}
