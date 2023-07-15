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
 * @file BaseSubscriber.hpp
 *
 * Defines basic functionality of Cyphal subscriber class
 *
 * @author Jacob Crabill <jacob@flyvoly.com>
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <lib/parameters/param.h>

#include "../CanardHandle.hpp"
#include "../CanardInterface.hpp"
#include "../ParamManager.hpp"

class UavcanBaseSubscriber
{
public:
	UavcanBaseSubscriber(CanardHandle &handle, const char *prefix_name, const char *subject_name, uint8_t instance = 0) :
		_canard_handle(handle), _prefix_name(prefix_name), _instance(instance)
	{
		_subj_sub._subject_name = subject_name;
		_subj_sub._canard_sub.user_reference = this;
		_subj_sub._canard_sub.port_id = CANARD_PORT_ID_UNSET;
	}

	virtual ~UavcanBaseSubscriber()
	{
		unsubscribe();
	}

	bool isValidPortId(int32_t id) const { return id >= 0 && id <= CANARD_PORT_ID_MAX; }

	virtual void subscribe() = 0;
	virtual void unsubscribe()
	{
		SubjectSubscription *curSubj = &_subj_sub;

		while (curSubj != nullptr) {
			_canard_handle.RxUnsubscribe(CanardTransferKindMessage, curSubj->_canard_sub.port_id);
			curSubj = curSubj->next;
		}
	};

	virtual void callback(const CanardRxTransfer &msg) = 0;

	CanardPortID id(uint32_t instance = 0)
	{
		uint32_t i = 0;
		SubjectSubscription *curSubj = &_subj_sub;

		while (curSubj != nullptr) {
			if (instance == i) {
				return curSubj->_canard_sub.port_id;
			}

			curSubj = curSubj->next;
			i++;
		}

		return CANARD_PORT_ID_UNSET; // Wrong id return unset
	}

	bool hasPortID(CanardPortID port_id)
	{
		if (!isValidPortId((int32_t)port_id)) {
			return false;
		}

		SubjectSubscription *curSubj = &_subj_sub;

		while (curSubj != nullptr) {
			if (port_id == curSubj->_canard_sub.port_id) {
				return true;
			}

			curSubj = curSubj->next;
		}

		return false;
	}

	const char *getSubjectName()
	{
		return _subj_sub._subject_name;
	}

	const char *getSubjectPrefix()
	{
		return _prefix_name;
	}

	uint8_t getInstance()
	{
		return _instance;
	}

	void printInfo(CanardPortID port_id = CANARD_PORT_ID_UNSET)
	{
		SubjectSubscription *curSubj = &_subj_sub;

		while (curSubj != nullptr) {
			if (curSubj->_canard_sub.port_id != CANARD_PORT_ID_UNSET) {
				if (port_id == CANARD_PORT_ID_UNSET ||
				    port_id == curSubj->_canard_sub.port_id) {
					PX4_INFO("Subscribed %s.%d on port %d", curSubj->_subject_name, _instance, curSubj->_canard_sub.port_id);
				}
			}

			curSubj = curSubj->next;
		}
	}

protected:
	struct SubjectSubscription {
		CanardRxSubscription _canard_sub;
		const char *_subject_name;
		struct SubjectSubscription *next {nullptr};
	};

	CanardHandle &_canard_handle;
	const char *_prefix_name;
	SubjectSubscription _subj_sub;
	uint8_t _instance {0};
	/// TODO: 'type' parameter? uavcan.pub.PORT_NAME.type (see 384.Access.1.0.uavcan)
};
