/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
 *   Author: @author David Sidrane <david_s5@nscdg.com>
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
 * @file board_mcu_version.c
 * Implementation of AT32 based SoC version API
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>

/* magic numbers from reference manual */

enum MCU_REV {
	MCU_REV_AT32F4_REV_A = 0x7008,
};

/* Define any issues with the Silicon as lines separated by \n
 * omitting the last \n
 */
#define AT32_F4_ERRATA "This device can only utilize a maximum of 1MB flash safely!"


//STM DocID018909 Rev 8 Sect 38.18 and DocID026670 Rev 5 40.6.1 (MCU device ID code)
# define REVID_MASK    0xFFFF0000
# define DEVID_MASK    0xFFFF


# define AT32F435ZMT7		0x4540
# define AT32F435ZGT7		0x3341
# define AT32F435ZCT7		0x3242
# define AT32F435VMT7		0x4543
# define AT32F435VGT7		0x3344
# define AT32F435VCT7		0x3245
# define AT32F435RMT7		0x4546
# define AT32F435RGT7		0x3347
# define AT32F435RCT7		0x3248
# define AT32F435CMT7		0x4549
# define AT32F435CGT7		0x334A
# define AT32F435CCT7		0x324B
# define AT32F435CMU7		0x454C
# define AT32F435CGU7		0x334D
# define AT32F435CCU7		0x324E

# define AT32F437ZMT7		0x454F
# define AT32F437ZGT7		0x3350
# define AT32F437ZCT7		0x3251
# define AT32F437VMT7		0x4552
# define AT32F437VGT7		0x3353
# define AT32F437VCT7		0x3254
# define AT32F437RMT7		0x4555
# define AT32F437RGT7		0x3356
# define AT32F437RCT7		0x3257



int board_mcu_version(char *rev, const char **revstr, const char **errata)
{
	uint32_t abc = getreg32(AT32_DEBUGMCU_BASE);

	int32_t chip_version = abc & DEVID_MASK;
	enum MCU_REV revid = (abc & REVID_MASK) >> 16;
	const char *chip_errata = NULL;

	switch (chip_version) {

	case AT32F435ZMT7:
		*revstr = "AT32F435ZMT7";
		break;

	case AT32F435ZGT7:
		*revstr = "AT32F435ZGT7";
		break;

	case AT32F435ZCT7:
		*revstr = "AT32F435ZCT7";
		break;

	case AT32F435VMT7:
		*revstr = "AT32F435VMT7";
		break;

	case AT32F435VGT7:
		*revstr = "AT32F435VGT7";
		break;

	case AT32F435VCT7:
		*revstr = "AT32F435VCT7";
		break;

	case AT32F435RMT7:
		*revstr = "AT32F435RMT7";
		break;

	case AT32F435RGT7:
		*revstr = "AT32F435RGT7";
		break;

	case AT32F435RCT7:
		*revstr = "AT32F435RCT7";
		break;

	case AT32F435CMT7:
		*revstr = "AT32F435CMT7";
		break;

	case AT32F435CGT7:
		*revstr = "AT32F435CGT7";
		break;

	case AT32F435CCT7:
		*revstr = "AT32F435CCT7";
		break;

	case AT32F435CMU7:
		*revstr = "AT32F435CMU7";
		break;

	case AT32F435CGU7:
		*revstr = "AT32F435CGU7";
		break;

	case AT32F435CCU7:
		*revstr = "AT32F435CCU7";
		break;

	case AT32F437ZMT7:
		*revstr = "AT32F437ZMT7";
		break;

	case AT32F437ZGT7:
		*revstr = "AT32F437ZGT7";
		break;

	case AT32F437ZCT7:
		*revstr = "AT32F437ZCT7";
		break;

	case AT32F437VMT7:
		*revstr = "AT32F437VMT7";
		break;

	case AT32F437VGT7:
		*revstr = "AT32F437VGT7";
		break;

	case AT32F437VCT7:
		*revstr = "AT32F437VCT7";
		break;

	case AT32F437RMT7:
		*revstr = "AT32F437RMT7";
		break;

	case AT32F437RGT7:
		*revstr = "AT32F437RGT7";
		break;

	case AT32F437RCT7:
		*revstr = "AT32F437RCT7";
		break;

	default:
		*revstr = "AT32F???";
		break;
	}

	switch (revid) {

	case MCU_REV_AT32F4_REV_A:
		*rev = 'A';
		break;

	default:
		// todo add rev for 103 - if needed
		*rev = '?';
		revid = -1;
		break;
	}

	if (errata) {
		*errata = chip_errata;
	}

	return revid;
}
