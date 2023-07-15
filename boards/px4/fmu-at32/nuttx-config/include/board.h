/************************************************************************************
 * nuttx-configs/omnibus-f4sd/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *   Author: Nathan Tsoi <nathan@vertile.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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
 ************************************************************************************/

#ifndef __CONFIG_PX4_FMU_AT32_INCLUDE_BOARD_H
#define __CONFIG_PX4_FMU_AT32_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/
//#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "at32_rcc.h"
#include "at32_sdio.h"
#include "at32.h"

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* The AT32F435-CS board features a single 8MHz crystal.
 * Space is provided for a 32kHz RTC backup crystal, but it is not stuffed.
 *
 * This is the canonical configuration:
 *   System Clock source           : PLL (HSE)
 *   SYSCLK(Hz)                    : 288000000    Determined by PLL
 *                                                configuration
 *   HCLK(Hz)                      : 288000000    (AT32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (AT32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 2            (AT32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 2            (AT32_RCC_CFGR_PPRE2)
 *   HSE Frequency(Hz)             : 8000000      (AT32_BOARD_XTAL)
 *   PLLM                          : 1            (AT32_PLLCFG_PLLMS)
 *   PLLN                          : 144           (AT32_PLLCFG_PLLNS)
 *   PLLP                          : 4            (AT32_PLLCFG_PLLFR)
 *   Main regulator output voltage : Scale1 mode  Needed for high speed
 *                                                SYSCLK
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO clock
 */

/* HSI - 48 MHz RC factory-trimmed
 * LSI - 40 KHz RC
 * HSE - On-board crystal frequency is 8MHz
 * LSE - 32.768 kHz
 */

#define AT32_BOARD_XTAL        8000000ul

#define AT32_HSI_FREQUENCY     48000000ul
#define AT32_LSI_FREQUENCY     40000
#define AT32_HSE_FREQUENCY     AT32_BOARD_XTAL
#define AT32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE
 *
 * FREQUENCY = HSE * AT32_PLLCFG_PLLN / (AT32_PLLCFG_PLLM * AT32_PLLCFG_PLLP)
 *
 */


#define AT32_PLLCFG_PLLM       CRM_PLL_CFG_PLL_MS(1)
#define AT32_PLLCFG_PLLN       CRM_PLL_CFG_PLL_NS(144)
#define AT32_PLLCFG_PLLP       CRM_PLL_CFG_PLL_FR_4

#define AT32_SYSCLK_FREQUENCY  288000000ul

/* AHB clock (HCLK) is SYSCLK (288MHz) */

#define AT32_HCLK_FREQUENCY    AT32_SYSCLK_FREQUENCY /* HCLK  = SYSCLK / 1 */

/* APB1 clock (PCLK1) is HCLK/2 (144MHz) */

#define AT32_PCLK1_FREQUENCY   (AT32_HCLK_FREQUENCY/2) /* PCLK1 = HCLK / 2 */

/* Timers driven from APB1 will be twice PCLK1 */

#define AT32_APB1_TIM2_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM3_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM4_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM5_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM6_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM7_CLKIN   (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM12_CLKIN  (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM13_CLKIN  (2*AT32_PCLK1_FREQUENCY)
#define AT32_APB1_TIM14_CLKIN  (2*AT32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK/2 (144MHz) */

#define AT32_PCLK2_FREQUENCY   (AT32_HCLK_FREQUENCY/2) /* PCLK2 = HCLK / 2 */

/* Timers driven from APB2 will be twice PCLK2 */

#define AT32_APB2_TIM1_CLKIN   (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM8_CLKIN   (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM9_CLKIN   (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM10_CLKIN  (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM11_CLKIN  (2*AT32_PCLK2_FREQUENCY)
#define AT32_APB2_TIM20_CLKIN  (2*AT32_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM2_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (AT32_HCLK_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (AT32_HCLK_FREQUENCY)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(118+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (118 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(1+2)=16 MHz
 * DMA OFF: SDIOCLK=48MHz, SDIO_CK=SDIOCLK/(2+2)=12 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif


/* USB */
/**
 * pll clock = AT32_HCLK_FREQUENCY(288MHz)
 * usb clock use pll
 * usb_clk = 288/6 = 48MHz
 * **/
#define USB_CONFIG_USBDIV       (CRM_MISC2_USBDIV_6P0)


/* LED definitions ******************************************************************/
/* If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in any
 * way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with at32_setled() */

#define BOARD_LED1        0
//#define BOARD_LED2        1
#define BOARD_NLEDS       1

#define BOARD_LED_BLUE    BOARD_LED1
//#define BOARD_LED_RED     BOARD_LED2

/* LED bits for use with at32_setleds() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the 2 LEDs on board the
 * omnibusf4sd.  The following definitions describe how NuttX controls the LEDs:
 */

#define LED_STARTED       0  /* LED1 */
#define LED_HEAPALLOCATE  1  /* LED2 */
#define LED_IRQSENABLED   2  /* LED1 */
#define LED_STACKCREATED  3  /* LED1 + LED2 */
#define LED_INIRQ         4  /* LED1 */
#define LED_SIGNAL        5  /* LED2 */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED1 + LED2 */

/* Alternate function pin selections ************************************************/

/* UART1:
 *
 * PA10 (RX) and PA9 (TX) are broken out on J5
 */

#define GPIO_USART1_RX GPIO_USART1_RX_1
#define GPIO_USART1_TX GPIO_USART1_TX_1


/* USART2:
*
* PA2 (TX)
* PA3 (RX)
*
*/
#define GPIO_USART2_RX GPIO_USART2_RX_1
#define GPIO_USART2_TX GPIO_USART2_TX_1


/* USART3:
 *
 * PC4 (TX) and PC5 (RX)
 *
 */
#define GPIO_USART3_RX	GPIO_USART3_RX_4
#define GPIO_USART3_TX	GPIO_USART3_TX_4

/* UART4:
 *
 * PA0 (TX)
 *
 * PA1 (RX)
 */
#define GPIO_UART4_RX	GPIO_UART4_RX_1
#define GPIO_UART4_TX	GPIO_UART4_TX_1

/* UART8:
 *
 * PC2 (TX) and PC3 (RX) are broken out on J10
 */

#define GPIO_UART8_RX GPIO_UART8_RX_2
#define GPIO_UART8_TX GPIO_UART8_TX_2

/* SPI1:
 *  ICM42688P
 *  CS: PA4 -- configured in board_config.h
 *  CLK: PA5
 *  MISO: PA6
 *  MOSI: PA7
 */

#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_1
#define GPIO_SPI1_MISO GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_1

/* SPI2:
 *  AT7456E
 *  CS: PD5 -- configured in board_config.h
 *  CLK: PD1
 *  MISO: PD3
 *  MOSI: PD4
 */

#define GPIO_SPI2_SCK	 GPIO_SPI2_SCK_6
#define GPIO_SPI2_MISO	GPIO_SPI2_MISO_4
#define GPIO_SPI2_MOSI	GPIO_SPI2_MOSI_5

/* SPI3:
 *  FLASH
 *  CS: PD6 -- configured in board_config.h
 *  CLK: PC10
 *  MISO: PC11
 *  MOSI: PC12
 */

#define GPIO_SPI3_SCK  GPIO_SPI3_SCK_2
#define GPIO_SPI3_MISO GPIO_SPI3_MISO_2
#define GPIO_SPI3_MOSI GPIO_SPI3_MOSI_2

/*
 * I2C (internal)
 *
 * SCL: PC0
 * SDA: PC1
 *
 * TODO:
 *   The optional _GPIO configurations allow the I2C driver to manually
 *   reset the bus to clear stuck slaves.  They match the pin configuration,
 *   but are normally-high GPIOs.
 */
#define GPIO_I2C3_SCL		GPIO_I2C3_SCL_4
#define GPIO_I2C3_SDA		GPIO_I2C3_SDA_3



/* PWM - motor outputs, etc. are on these pins: */

#define GPIO_TIM4_CH1OUT  GPIO_TIM4_CH1OUT_1   /* S1_OUT  PB6 */
#define GPIO_TIM4_CH2OUT  GPIO_TIM4_CH2OUT_1   /* S2_OUT  PB7 */
#define GPIO_TIM4_CH3OUT  GPIO_TIM4_CH3OUT_1   /* S3_OUT  PB8 */
#define GPIO_TIM4_CH4OUT  GPIO_TIM4_CH4OUT_1   /* S4_OUT  PB9 */



#endif  /* __CONFIG_OMNIBUSF4SD_INCLUDE_BOARD_H */
