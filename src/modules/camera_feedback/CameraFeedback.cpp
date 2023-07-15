/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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

#include "CameraFeedback.hpp"

using namespace time_literals;

CameraFeedback::CameraFeedback() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_p_cam_cap_fback = param_find("CAM_CAP_FBACK");

	if (_p_cam_cap_fback != PARAM_INVALID) {
		param_get(_p_cam_cap_fback, (int32_t *)&_cam_cap_fback);
	}
}

bool
CameraFeedback::init()
{
	if (!_trigger_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_capture_pub.advertise();

	return true;
}

void
CameraFeedback::Run()
{
	if (should_exit()) {
		_trigger_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	camera_trigger_s trig{};

	while (_trigger_sub.update(&trig)) {

		// update geotagging subscriptions
		vehicle_global_position_s gpos{};
		_gpos_sub.copy(&gpos);

		vehicle_attitude_s att{};
		_att_sub.copy(&att);

		if (trig.timestamp == 0 ||
		    gpos.timestamp == 0 ||
		    att.timestamp == 0) {

			// reject until we have valid data
			continue;
		}

		if ((_cam_cap_fback >= 1) && !trig.feedback) {
			// Ignore triggers that are not feedback when camera capture feedback is enabled
			continue;
		}

		camera_capture_s capture{};

		// Fill timestamps
		capture.timestamp = trig.timestamp;
		capture.timestamp_utc = trig.timestamp_utc;

		// Fill image sequence
		capture.seq = trig.seq;

		// Fill position data
		capture.lat = gpos.lat;
		capture.lon = gpos.lon;
		capture.alt = gpos.alt;

		if (gpos.terrain_alt_valid) {
			capture.ground_distance = gpos.alt - gpos.terrain_alt;

		} else {
			capture.ground_distance = -1.0f;
		}

		// Fill attitude data
		gimbal_device_attitude_status_s gimbal{};

		if (_gimbal_sub.copy(&gimbal) && (hrt_elapsed_time(&gimbal.timestamp) < 1_s)) {
			if (gimbal.device_flags & gimbal_device_attitude_status_s::DEVICE_FLAGS_YAW_LOCK) {
				// Gimbal yaw angle is absolute angle relative to North
				capture.q[0] = gimbal.q[0];
				capture.q[1] = gimbal.q[1];
				capture.q[2] = gimbal.q[2];
				capture.q[3] = gimbal.q[3];

			} else {
				// Gimbal quaternion frame is in the Earth frame rotated so that the x-axis is pointing
				// forward (yaw relative to vehicle). Get heading from the vehicle attitude and combine it
				// with the gimbal orientation.
				const matrix::Eulerf euler_vehicle(matrix::Quatf(att.q));
				const matrix::Quatf q_heading(matrix::Eulerf(0.0f, 0.0f, euler_vehicle(2)));
				matrix::Quatf q_gimbal(gimbal.q);
				q_gimbal = q_heading * q_gimbal;

				capture.q[0] = q_gimbal(0);
				capture.q[1] = q_gimbal(1);
				capture.q[2] = q_gimbal(2);
				capture.q[3] = q_gimbal(3);
			}

		} else {
			// No gimbal orientation, use vehicle attitude
			capture.q[0] = att.q[0];
			capture.q[1] = att.q[1];
			capture.q[2] = att.q[2];
			capture.q[3] = att.q[3];
		}

		capture.result = 1;

		_capture_pub.publish(capture);
	}
}

int
CameraFeedback::task_spawn(int argc, char *argv[])
{
	CameraFeedback *instance = new CameraFeedback();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int
CameraFeedback::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
CameraFeedback::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("camera_feedback", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int camera_feedback_main(int argc, char *argv[])
{
	return CameraFeedback::main(argc, argv);
}
