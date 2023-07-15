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
 * @file ServiceRequest.hpp
 *
 * Defines a Service invoker base class and process responses
 *
 * @author Peter van der Perk <peter.vanderperk@nxp.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <version/version.h>

#include <uavcan/_register/List_1_0.h>

#include "../Subscribers/BaseSubscriber.hpp"


class UavcanServiceRequestInterface
{
public:
	virtual void response_callback(const CanardRxTransfer &receive) = 0;
};

class UavcanServiceRequest : public UavcanBaseSubscriber
{
public:
	UavcanServiceRequest(CanardHandle &handle, const char *prefix_name, const char *subject_name, CanardPortID portID,
			     size_t extent) :
		UavcanBaseSubscriber(handle, prefix_name, subject_name, 0), _portID(portID), _extent(extent) { };


	void subscribe() override
	{
		// Subscribe to requests response
		_canard_handle.RxSubscribe(CanardTransferKindResponse,
					   _portID,
					   _extent,
					   CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
					   &_subj_sub._canard_sub);
	};

	bool request(const CanardMicrosecond             tx_deadline_usec,
		     const CanardTransferMetadata        *transfer_metadata,
		     const size_t                        payload_size,
		     const void *const                   payload,
		     UavcanServiceRequestInterface       *handler)
	{
		_response_callback = handler;
		remote_node_id = transfer_metadata->remote_node_id;
		++request_transfer_id;  // The transfer-ID shall be incremented after every transmission on this subject.
		return _canard_handle.TxPush(tx_deadline_usec,
					     transfer_metadata,
					     payload_size,
					     payload) > 0;
	}

	void callback(const CanardRxTransfer &receive) override
	{
		PX4_INFO("Response");

		if (_response_callback != nullptr &&
		    receive.metadata.transfer_id == (request_transfer_id - 1) &&
		    receive.metadata.remote_node_id == remote_node_id) {
			_response_callback->response_callback(receive);
		}
	};



protected:
	CanardTransferID request_transfer_id = 0;
	CanardNodeID remote_node_id = CANARD_NODE_ID_UNSET;

	const CanardPortID _portID;
	const size_t _extent;
	UavcanServiceRequestInterface *_response_callback = nullptr;

};
