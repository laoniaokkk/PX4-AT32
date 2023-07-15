/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file List.hpp
 *
 * Defines response to a List request
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include "../ParamManager.hpp"

#include <uavcan/_register/List_1_0.h>

#include "../Subscribers/BaseSubscriber.hpp"

class UavcanListResponse : public UavcanBaseSubscriber
{
public:
	UavcanListResponse(CanardHandle &handle, UavcanParamManager &pmgr) :
		UavcanBaseSubscriber(handle, "", "List", 0),  _param_manager(pmgr) { };

	void subscribe() override
	{
		// Subscribe to requests uavcan.pnp.NodeIDAllocationData

		_canard_handle.RxSubscribe(CanardTransferKindRequest,
					   uavcan_register_List_1_0_FIXED_PORT_ID_,
					   uavcan_register_List_Response_1_0_EXTENT_BYTES_,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);

	};

	void callback(const CanardRxTransfer &receive) override
	{
		PX4_INFO("List request");

		size_t payload_size = uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;

		uavcan_register_List_Request_1_0 msg;
		uavcan_register_List_Response_1_0 response;

		uavcan_register_List_Request_1_0_initialize_(&msg);
		uavcan_register_List_Response_1_0_initialize_(&response);

		size_t register_in_size_bits = receive.payload_size;
		uavcan_register_List_Request_1_0_deserialize_(&msg, (const uint8_t *)receive.payload, &register_in_size_bits);

		int result {0};

		if (_param_manager.GetParamName(msg.index, response.name) == 0) {
			response.name.name.count = 0;
		}

		uint8_t response_payload_buffer[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

		const CanardTransferMetadata transfer_metadata = {
			.priority       = CanardPriorityNominal,
			.transfer_kind  = CanardTransferKindResponse,
			.port_id        = uavcan_register_List_1_0_FIXED_PORT_ID_,                // This is the subject-ID.
			.remote_node_id = receive.metadata.remote_node_id,       // Messages cannot be unicast, so use UNSET.
			.transfer_id    = receive.metadata.transfer_id
		};

		result = uavcan_register_List_Response_1_0_serialize_(&response, response_payload_buffer, &payload_size);

		if (result == 0) {
			// set the data ready in the buffer and chop if needed
			result = _canard_handle.TxPush(hrt_absolute_time() + PUBLISHER_DEFAULT_TIMEOUT_USEC,
						       &transfer_metadata,
						       payload_size,
						       &response_payload_buffer);
		}

	};

private:
	UavcanParamManager &_param_manager;

};
