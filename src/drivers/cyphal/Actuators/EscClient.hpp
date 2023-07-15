/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file EscClient.hpp
 *
 * Client-side implementation of UDRAL specification ESC service
 *
 * Publishes the following Cyphal messages:
 *   reg.drone.service.actuator.common.sp.Value8.0.1
 *   reg.drone.service.common.Readiness.0.1
 *
 * Subscribes to the following Cyphal messages:
 *   reg.drone.service.actuator.common.Feedback.0.1
 *   reg.drone.service.actuator.common.Status.0.1
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_test.h>
#include <uORB/topics/esc_status.h>
#include <drivers/drv_hrt.h>
#include <lib/mixer_module/mixer_module.hpp>

// UDRAL Specification Messages
using std::isfinite;
#include <reg/udral/service/actuator/common/sp/Vector31_0_1.h>
#include <reg/udral/service/common/Readiness_0_1.h>

/// TODO: Allow derived class of Subscription at same time, to handle ESC Feedback/Status
class UavcanEscController : public UavcanPublisher
{
public:
	static constexpr int MAX_ACTUATORS = MixingOutput::MAX_ACTUATORS;

	UavcanEscController(CanardHandle &handle, UavcanParamManager &pmgr) :
		UavcanPublisher(handle, pmgr, "udral.", "esc") { };

	~UavcanEscController() {};

	void update() override
	{
		actuator_test_s actuator_test;

		if (_actuator_test_sub.update(&actuator_test)) {
			_actuator_test_timestamp = actuator_test.timestamp;
		}

		if (_armed_sub.updated()) {
			actuator_armed_s new_arming;
			_armed_sub.update(&new_arming);

			if (new_arming.armed != _armed.armed) {
				_armed = new_arming;
				publish_readiness();
			}
		}

		if (hrt_absolute_time() > _previous_pub_time + READINESS_PUBLISH_PERIOD) {
			publish_readiness();
		}
	};

	void update_outputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs)
	{
		if (_port_id > 0) {
			reg_udral_service_actuator_common_sp_Vector31_0_1 msg_sp {0};
			size_t payload_size = reg_udral_service_actuator_common_sp_Vector31_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;

			for (uint8_t i = 0; i < MAX_ACTUATORS; i++) {
				if (i < num_outputs) {
					msg_sp.value[i] = static_cast<float>(outputs[i]);

				} else {
					// "unset" values published as NaN
					msg_sp.value[i] = NAN;
				}
			}

			uint8_t esc_sp_payload_buffer[reg_udral_service_actuator_common_sp_Vector31_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

			const CanardTransferMetadata transfer_metadata = {
				.priority       = CanardPriorityNominal,
				.transfer_kind  = CanardTransferKindMessage,
				.port_id        = _port_id,                // This is the subject-ID.
				.remote_node_id = CANARD_NODE_ID_UNSET,       // Messages cannot be unicast, so use UNSET.
				.transfer_id    = _transfer_id,
			};

			int result = reg_udral_service_actuator_common_sp_Vector31_0_1_serialize_(&msg_sp, esc_sp_payload_buffer,
					&payload_size);

			if (result == 0) {
				// set the data ready in the buffer and chop if needed
				++_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
				result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
							       &transfer_metadata,
							       payload_size,
							       &esc_sp_payload_buffer);
			}
		}
	};

	/**
	 * Sets the number of rotors
	 */
	void set_rotor_count(uint8_t count) { _rotor_count = count; }

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const CanardRxTransfer &msg);

	void publish_readiness()
	{
		const hrt_abstime now = hrt_absolute_time();
		_previous_pub_time = now;

		size_t payload_size = reg_udral_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_;

		// Only publish if we have a valid publication ID set
		if (_port_id == 0) {
			return;
		}

		reg_udral_service_common_Readiness_0_1 msg_arming {};

		if (_armed.armed || _actuator_test_timestamp + 100_ms > now) {
			msg_arming.value = reg_udral_service_common_Readiness_0_1_ENGAGED;

		} else if (_armed.prearmed) {
			msg_arming.value = reg_udral_service_common_Readiness_0_1_STANDBY;

		} else {
			msg_arming.value = reg_udral_service_common_Readiness_0_1_SLEEP;
		}

		uint8_t arming_payload_buffer[reg_udral_service_common_Readiness_0_1_SERIALIZATION_BUFFER_SIZE_BYTES_];

		CanardPortID arming_pid = static_cast<CanardPortID>(static_cast<uint32_t>(_port_id) + 1);
		const CanardTransferMetadata transfer_metadata = {
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindMessage,
			.port_id        = arming_pid,                // This is the subject-ID.
			.remote_node_id = CANARD_NODE_ID_UNSET,      // Messages cannot be unicast, so use UNSET.
			.transfer_id    = _arming_transfer_id,
		};

		int result = reg_udral_service_common_Readiness_0_1_serialize_(&msg_arming, arming_payload_buffer,
				&payload_size);

		if (result == 0) {
			// set the data ready in the buffer and chop if needed
			++_arming_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
			result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
						       &transfer_metadata,
						       payload_size,
						       &arming_payload_buffer);
		}
	};

	uint8_t _rotor_count {0};

	static constexpr hrt_abstime READINESS_PUBLISH_PERIOD = 100000;
	hrt_abstime _previous_pub_time = 0;

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	actuator_armed_s _armed {};

	uORB::Subscription _actuator_test_sub{ORB_ID(actuator_test)};
	uint64_t _actuator_test_timestamp{0};

	CanardTransferID _arming_transfer_id;
};
