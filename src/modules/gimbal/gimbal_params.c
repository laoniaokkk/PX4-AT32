/****************************************************************************
*
*   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
* Mount input mode
*
* This is the protocol used between the ground station and the autopilot.
*
* Recommended is Auto, RC only or MAVLink gimbal protocol v2.
* The rest will be deprecated.
*
* @value -1 DISABLED
* @value 0 Auto (RC and MAVLink gimbal protocol v2)
* @value 1 RC
* @value 2 MAVLINK_ROI (protocol v1, to be deprecated)
* @value 3 MAVLINK_DO_MOUNT (protocol v1, to be deprecated)
* @value 4 MAVlink gimbal protocol v2
* @min -1
* @max 4
* @group Mount
* @reboot_required true
*/
PARAM_DEFINE_INT32(MNT_MODE_IN, -1);

/**
* Mount output mode
*
* This is the protocol used between the autopilot and a connected gimbal.
*
* Recommended is the MAVLink gimbal protocol v2 if the gimbal supports it.
*
* @value 0 AUX
* @value 1 MAVLink gimbal protocol v1
* @value 2 MAVLink gimbal protocol v2
* @min 0
* @max 2
* @group Mount
* @reboot_required true
*/
PARAM_DEFINE_INT32(MNT_MODE_OUT, 0);

/**
* Mavlink System ID of the mount
*
* If MNT_MODE_OUT is MAVLink gimbal protocol v1, mount configure/control commands will be sent with this target ID.
*
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAV_SYSID, 1);

/**
* Mavlink Component ID of the mount
*
* If MNT_MODE_OUT is MAVLink protocol v2, mount configure/control commands will be sent with this component ID.
*
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAV_COMPID, 154);

/**
* Auxiliary channel to control roll (in AUX input or manual mode).
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @value 6 AUX6
* @min 0
* @max 6
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_ROLL, 0);

/**
* Auxiliary channel to control pitch (in AUX input or manual mode).
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @value 6 AUX6
* @min 0
* @max 6
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_PITCH, 0);

/**
* Auxiliary channel to control yaw (in AUX input or manual mode).
*
* @value 0 Disable
* @value 1 AUX1
* @value 2 AUX2
* @value 3 AUX3
* @value 4 AUX4
* @value 5 AUX5
* @value 6 AUX6
* @min 0
* @max 6
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_MAN_YAW, 0);

/**
* Stabilize the mount
*
* Set to true for servo gimbal, false for passthrough.
* This is required for a gimbal which is not capable of stabilizing itself
* and relies on the IMU's attitude estimation.
*
* @value 0 Disable
* @value 1 Stabilize all axis
* @value 2 Stabilize yaw for absolute/lock mode.
* @min 0
* @max 2
* @group Mount
*/
PARAM_DEFINE_INT32(MNT_DO_STAB, 0);

/**
* Range of pitch channel output in degrees (only in AUX output mode).
*
* @min 1.0
* @max 720.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_RANGE_PITCH, 90.0f);

/**
* Range of roll channel output in degrees (only in AUX output mode).
*
* @min 1.0
* @max 720.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_RANGE_ROLL, 90.0f);

/**
* Range of yaw channel output in degrees (only in AUX output mode).
*
* @min 1.0
* @max 720.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_RANGE_YAW, 360.0f);

/**
* Offset for pitch channel output in degrees.
*
* @min -360.0
* @max 360.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OFF_PITCH, 0.0f);

/**
* Offset for roll channel output in degrees.
*
* @min -360.0
* @max 360.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OFF_ROLL, 0.0f);

/**
* Offset for yaw channel output in degrees.
*
* @min -360.0
* @max 360.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_OFF_YAW, 0.0f);

/**
 * Angular pitch rate for manual input in degrees/second.
 *
 * Full stick input [-1..1] translats to [-pitch rate..pitch rate].
 *
 * @min 1.0
 * @max 90.0
 * @unit deg/s
 * @group Mount
 */
PARAM_DEFINE_FLOAT(MNT_RATE_PITCH, 30.0f);

/**
 * Angular yaw rate for manual input in degrees/second.
 *
 * Full stick input [-1..1] translats to [-yaw rate..yaw rate].
 *
 * @min 1.0
 * @max 90.0
 * @unit deg/s
 * @group Mount
 */
PARAM_DEFINE_FLOAT(MNT_RATE_YAW, 30.0f);

/**
 * Input mode for RC gimbal input
 *
 * @value 0 Angle
 * @value 1 Angular rate
 * @min 0
 * @max 1
 * @group Mount
 */
PARAM_DEFINE_INT32(MNT_RC_IN_MODE, 1);

/**
* Pitch minimum when landed
*
* @min -90.0
* @max 90.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_LND_P_MIN, -90.0f);

/**
* Pitch maximum when landed
*
* @min -90.0
* @max 90.0
* @unit deg
* @decimal 1
* @group Mount
*/
PARAM_DEFINE_FLOAT(MNT_LND_P_MAX, 90.0f);
