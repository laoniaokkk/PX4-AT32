/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *       Author: David Sidrane <david.sidrane@nscdg.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <px4_config.h>
#include <stdint.h>
#include "boot_config.h"
#include "board.h"
#include "led.h"

#include <debug.h>
#include <string.h>
#include <arch/board/board.h>

#include <nuttx/board.h>

#include "s32k1xx_periphclocks.h"
#include "s32k1xx_flexcan.h"

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the initialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void s32k1xx_board_initialize(void)
{
	struct peripheral_clock_config_s c;
	c.clkname = FLEXCAN0_CLK,
	c.clkgate = true,
	// CAN clocking
	s32k1xx_periphclocks(1, &c);
	// Can GPIO
	s32k1xx_pinconfig(PIN_CAN0_TX);
	s32k1xx_pinconfig(PIN_CAN0_RX);

	s32k1xx_pinconfig(BOARD_REVISION_DETECT_PIN);

	if (s32k1xx_gpioread(BOARD_REVISION_DETECT_PIN)) {
		/* Config Pins to do CAN tranceiver HW selftest */
		s32k1xx_pinconfig(PIN_CAN0_ERRN);
		s32k1xx_pinconfig(PIN_CAN0_STB);
		s32k1xx_pinconfig(PIN_CAN0_EN);

		/* EN high & STB high -> normal mode */
		s32k1xx_gpiowrite(PIN_CAN0_STB, 1);
		s32k1xx_gpiowrite(PIN_CAN0_EN, 1);
		up_udelay(3000); // Wait for startup to normal mode

		/* EN low & STB high -> listen only mode */
		s32k1xx_gpiowrite(PIN_CAN0_STB, 1);
		s32k1xx_gpiowrite(PIN_CAN0_EN, 0);
		up_udelay(100); // t moch ERRN_N

		/* Check for HW err and wait untill ERR has been cleared */
		while (!s32k1xx_gpioread(PIN_CAN0_ERRN)) {
			board_indicate(hardware_failure);
			s32k1xx_gpiowrite(PIN_CAN0_EN, 1);
			up_udelay(50);
			s32k1xx_gpiowrite(PIN_CAN0_EN, 0);
			up_udelay(50);
		}

		/* Enter normal-mode */
		s32k1xx_gpiowrite(PIN_CAN0_EN, 1);

	} else {
		/* STB low -> active CAN phy */
		s32k1xx_pinconfig(PIN_CAN0_STB  | GPIO_OUTPUT_ZERO);
	}

	//s32k1xx_gpiowrite
#if defined(OPT_WAIT_FOR_GETNODEINFO_JUMPER_GPIO)
	s32k1xx_pinconfig(GPIO_GETNODEINFO_JUMPER);
#endif
	ucans32k_timer_initialize();
}

/************************************************************************************
 * Name: board_deinitialize
 *
 * Description:
 *   This function is called by the bootloader code prior to booting
 *   the application. Is should place the HW into an benign initialized state.
 *
 ************************************************************************************/

void board_deinitialize(void)
{
	uint32_t regval  = getreg32(S32K1XX_CAN0_MCR);
	regval |= CAN_MCR_MDIS;
	putreg32(regval, S32K1XX_CAN0_MCR);


	do {
		regval  = getreg32(S32K1XX_CAN0_MCR);
		up_udelay(10);
	} while ((regval & CAN_MCR_LPMACK) == 0);


	if (s32k1xx_gpioread(BOARD_REVISION_DETECT_PIN)) {
		/* STB high -> standby CAN phy */
		s32k1xx_pinconfig(PIN_CAN0_STB  | GPIO_OUTPUT_ZERO);

	} else {
		/* STB low -> standby CAN phy */
		s32k1xx_pinconfig(PIN_CAN0_STB  | GPIO_OUTPUT_ONE);
	}
}

/****************************************************************************
 * Name: board_get_product_name
 *
 * Description:
 *   Called to retrieve the product name. The returned value is a assumed
 *   to be written to a pascal style string that will be length prefixed
 *   and not null terminated
 *
 * Input Parameters:
 *    product_name - A pointer to a buffer to write the name.
 *    maxlen       - The maximum number of charter that can be written
 *
 * Returned Value:
 *   The length of characters written to the buffer.
 *
 ****************************************************************************/

uint8_t board_get_product_name(uint8_t *product_name, size_t maxlen)
{
	DEBUGASSERT(maxlen > UAVCAN_STRLEN(HW_UAVCAN_NAME));
	memcpy(product_name, HW_UAVCAN_NAME, UAVCAN_STRLEN(HW_UAVCAN_NAME));
	return UAVCAN_STRLEN(HW_UAVCAN_NAME);
}

/****************************************************************************
 * Name: board_get_hardware_version
 *
 * Description:
 *   Called to retrieve the hardware version information. The function
 *   will first initialize the the callers struct to all zeros.
 *
 * Input Parameters:
 *    hw_version - A pointer to a uavcan_hardwareversion_t.
 *
 * Returned Value:
 *   Length of the unique_id
 *
 ****************************************************************************/

size_t board_get_hardware_version(uavcan_HardwareVersion_t *hw_version)
{
	memset(hw_version, 0, sizeof(uavcan_HardwareVersion_t));

	hw_version->major = HW_VERSION_MAJOR;
	hw_version->minor = HW_VERSION_MINOR;

	return board_get_mfguid(*(mfguid_t *) hw_version->unique_id);
}

/****************************************************************************
 * Name: board_indicate
 *
 * Description:
 *   Provides User feedback to indicate the state of the bootloader
 *   on board specific  hardware.
 *
 * Input Parameters:
 *    indication - A member of the uiindication_t
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
#define led(n, code, r , g , b, h) {.red = (r),.green = (g), .blue = (b),.hz = (h)}

typedef begin_packed_struct struct led_t {
	uint16_t red    :       5;
	uint16_t green  :       6;
	uint16_t blue   :       5;
	uint8_t hz;
} end_packed_struct led_t;

static const  led_t i2l[] = {

	led(0, off,                             0,    0,    0,     0),
	led(1, reset,                          10,   63,   31,   255),
	led(2, autobaud_start,                  0,   63,    0,     1),
	led(3, autobaud_end,                    0,   63,    0,     2),
	led(4, allocation_start,                0,    0,   31,     2),
	led(5, allocation_end,                  0,   63,   31,     3),
	led(6, fw_update_start,                15,   63,   31,     3),
	led(7, fw_update_erase_fail,           15,   63,   15,     3),
	led(8, fw_update_invalid_response,     31,    0,    0,     1),
	led(9, fw_update_timeout,              31,    0,    0,     2),
	led(a, fw_update_invalid_crc,          31,    0,    0,     4),
	led(b, jump_to_app,                     0,   63,    0,    10),
	led(c, hardware_failure,               31,    0,    0,    10),

};

void board_indicate(uiindication_t indication)
{
	rgb_led(i2l[indication].red << 3,
		i2l[indication].green << 2,
		i2l[indication].blue << 3,
		i2l[indication].hz);
}

#include <drivers/drv_watchdog.h>

// Stubs
void watchdog_init(void)
{

}
void watchdog_pet(void)
{

}
