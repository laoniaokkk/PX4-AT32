/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file esc_calibration.cpp
 *
 * Definition of esc calibration
 *
 * @author Roman Bapst <roman@px4.io>
 */

#include "esc_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_test.h>
#include <parameters/param.h>

using namespace time_literals;

bool check_battery_disconnected(orb_advert_t *mavlink_log_pub)
{
	uORB::SubscriptionData<battery_status_s> batt_sub{ORB_ID(battery_status)};
	const battery_status_s &battery = batt_sub.get();
	batt_sub.update();

	if (battery.timestamp == 0) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "battery unavailable");
		return false;
	}

	// Make sure battery is disconnected
	// battery is not connected if the connected flag is not set and we have a recent battery measurement
	if (!battery.connected && (hrt_elapsed_time(&battery.timestamp) < 500_ms)) {
		return true;
	}

	calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Disconnect battery and try again");
	return false;
}

static void set_motor_actuators(uORB::Publication<actuator_test_s> &publisher, float value, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = 0;

	for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) {
		actuator_test.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		publisher.publish(actuator_test);
	}
}

int do_esc_calibration(orb_advert_t *mavlink_log_pub)
{
	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, "esc");

	int	return_code = PX4_OK;
	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	// since we publish multiple at once, make sure the output driver subscribes before we publish
	actuator_test_pub.advertise();
	px4_usleep(10000);

	// set motors to high
	set_motor_actuators(actuator_test_pub, 1.f, false);
	calibration_log_info(mavlink_log_pub, "[cal] Connect battery now");


	uORB::SubscriptionData<battery_status_s> batt_sub{ORB_ID(battery_status)};
	const battery_status_s &battery = batt_sub.get();
	batt_sub.update();
	bool batt_connected = battery.connected;
	hrt_abstime timeout_start = hrt_absolute_time();

	while (true) {
		// We are either waiting for the user to connect the battery. Or we are waiting to let the PWM
		// sit high.
		static constexpr hrt_abstime battery_connect_wait_timeout{20_s};
		static constexpr hrt_abstime pwm_high_timeout{3_s};
		hrt_abstime timeout_wait = batt_connected ? pwm_high_timeout : battery_connect_wait_timeout;

		if (hrt_elapsed_time(&timeout_start) > timeout_wait) {
			if (!batt_connected) {
				calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Timeout waiting for battery");
				return_code = PX4_ERROR;
			}

			// PWM was high long enough
			break;
		}

		if (!batt_connected) {
			if (batt_sub.update()) {
				if (battery.connected) {
					// Battery is connected, signal to user and start waiting again
					batt_connected = true;
					timeout_start = hrt_absolute_time();
					calibration_log_info(mavlink_log_pub, "[cal] Battery connected");
				}
			}
		}

		px4_usleep(50000);
	}

	if (return_code == PX4_OK) {
		// set motors to low
		set_motor_actuators(actuator_test_pub, 0.f, false);
		px4_usleep(4000000);

		// release control
		set_motor_actuators(actuator_test_pub, 0.f, true);

		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "esc");
	}

	return return_code;
}
