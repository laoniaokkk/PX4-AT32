/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *       Author: Ben Dyer <ben_dyer@mac.com>
 *               Pavel Kirienko <pavel.kirienko@zubax.com>
 *               David Sidrane <david_s5@nscdg.com>
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

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>


#include <errno.h>

#include <px4_arch/micro_hal.h>
#include "arm_internal.h"
#include "boot_app_shared.h"

#include <lib/crc/crc.h>

#define BOOTLOADER_COMMON_APP_SIGNATURE         0xB0A04150u
#define BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE  0xB0A0424Cu

#define CRC_H 1
#define CRC_L 0

inline static void read_shared(bootloader_app_shared_t *pshared)
{
	pshared->signature = getreg32(signature_LOC);
	pshared->bus_speed = getreg32(bus_speed_LOC);
	pshared->node_id = getreg32(node_id_LOC);
	pshared->crc.ul[CRC_L] = getreg32(crc_LoLOC);
	pshared->crc.ul[CRC_H] = getreg32(crc_HiLOC);
}

inline static void write_shared(bootloader_app_shared_t *pshared)
{
	putreg32(pshared->signature, signature_LOC);
	putreg32(pshared->bus_speed, bus_speed_LOC);
	putreg32(pshared->node_id, node_id_LOC);
	putreg32(pshared->crc.ul[CRC_L], crc_LoLOC);
	putreg32(pshared->crc.ul[CRC_H], crc_HiLOC);
}

static uint64_t calulate_signature(bootloader_app_shared_t *pshared)
{
	uint64_t crc;
	crc = crc64_add_word(CRC64_INITIAL, pshared->signature);
	crc = crc64_add_word(crc, pshared->bus_speed);
	crc = crc64_add_word(crc, pshared->node_id);
	crc ^= CRC64_OUTPUT_XOR;
	return crc;
}

static void bootloader_app_shared_init(bootloader_app_shared_t *pshared, eRole_t role)
{
	memset(pshared, 0, sizeof(bootloader_app_shared_t));

	if (role != Invalid) {
		pshared->signature =
			(role ==
			 App ? BOOTLOADER_COMMON_APP_SIGNATURE :
			 BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	}
}

/****************************************************************************
 * Name: bootloader_app_shared_read
 *
 * Description:
 *   Based on the role requested, this function will conditionally populate
 *   a bootloader_app_shared_t structure from the physical locations used
 *   to transfer the shared data to/from an application (internal data) .
 *
 *   The functions will only populate the structure and return a status
 *   indicating success, if the internal data has the correct signature as
 *   requested by the Role AND has a valid crc.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t return the data in if
 *   the internal data is valid for the requested Role
 *   role   - An eRole_t of App or BootLoader to validate the internal data
 *            against. For a Bootloader this would be the value of App to
 *            read the application passed data.
 *
 * Returned value:
 *   OK     - Indicates that the internal data has been copied to callers
 *            bootloader_app_shared_t structure.
 *
 *  -EBADR  - The Role or crc of the internal data was not valid. The copy
 *            did not occur.
 *
 ****************************************************************************/
__EXPORT int bootloader_app_shared_read(bootloader_app_shared_t *shared, eRole_t role)
{
	int rv = -EBADR;
	bootloader_app_shared_t working;

	read_shared(&working);

	if ((role == App ? working.signature == BOOTLOADER_COMMON_APP_SIGNATURE
	     : working.signature == BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE)
	    && (working.crc.ull == calulate_signature(&working))) {
		*shared = working;
		rv = OK;
	}

	return rv;
}

/****************************************************************************
 * Name: bootloader_app_shared_write
 *
 * Description:
 *   Based on the role, this function will commit the data passed
 *   into the physical locations used to transfer the shared data to/from
 *   an application (internal data) .
 *
 *   The functions will populate the signature and crc the data
 *   based on the provided Role.
 *
 * Input Parameters:
 *   shared - A pointer to a bootloader_app_shared_t data to commit to
 *   the internal data for passing to/from an application.
 *   role   - An eRole_t of App or BootLoader to use in the internal data
 *            to be passed to/from an application. For a Bootloader this
 *            would be the value of Bootloader to write to the passed data.
 *            to the application via the internal data.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
__EXPORT void bootloader_app_shared_write(bootloader_app_shared_t *shared, eRole_t role)
{
	bootloader_app_shared_t working = *shared;
	working.signature = (role == App ? BOOTLOADER_COMMON_APP_SIGNATURE : BOOTLOADER_COMMON_BOOTLOADER_SIGNATURE);
	working.crc.ull = calulate_signature(&working);
	write_shared(&working);
}

/****************************************************************************
 * Name: bootloader_app_shared_invalidate
 *
 * Description:
 *   Invalidates the data passed the physical locations used to transfer
 *   the shared data to/from an application (internal data) .
 *
 *   The functions will invalidate the signature and crc and shoulf be used
 *   to prevent deja vu.
 *
 * Input Parameters:
 *   None.
 *
 * Returned value:
 *   None.
 *
 ****************************************************************************/
__EXPORT void bootloader_app_shared_invalidate(void)
{
	bootloader_app_shared_t working;
	bootloader_app_shared_init(&working, Invalid);
	write_shared(&working);
}
