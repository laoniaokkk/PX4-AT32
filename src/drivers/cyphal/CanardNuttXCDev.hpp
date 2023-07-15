/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/px4_config.h>

#include <canard.h>

#include "CanardInterface.hpp"

class CanardNuttXCDev : public CanardInterface
{
public:
	CanardNuttXCDev() = default;
	~CanardNuttXCDev() override = default;

	/// Creates a SocketCAN socket for corresponding iface can_iface_name
	/// Also sets up the message structures required for socketcanTransmit & socketcanReceive
	/// can_fd determines to use CAN FD frame when is 1, and classical CAN frame when is 0
	/// The return value is 0 on succes and -1 on error
	int init();

	/// Send a CanardTxQueueItem to the CanardSocketInstance socket
	/// This function is blocking
	/// The return value is number of bytes transferred, negative value on error.
	int16_t transmit(const CanardTxQueueItem &txframe, int timeout_ms = 0);

	/// Receive a CanardRxFrame from the CanardSocketInstance socket
	/// This function is blocking
	/// The return value is number of bytes received, negative value on error.
	int16_t receive(CanardRxFrame *rxf);

private:
	int _fd{-1};
	bool _can_fd{false};
};
