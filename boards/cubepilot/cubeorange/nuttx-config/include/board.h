/************************************************************************************
 * nuttx-config/include/board.h
 *
 *   Copyright (C) 2020 Gregory Nutt. All rights reserved.
 *   Authors: David Sidrane <david.sidrane@nscdg.com>
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
#pragma once

#include "board_dma_map.h"

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

#include "stm32_rcc.h"
#include "stm32_sdmmc.h"

/* Clocking *************************************************************************/
/* The board provides the following clock sources:
 *
 *   X1: 24 MHz crystal for HSE
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed internal oscillator
 *   HSE: 24 MHz crystal for HSE
 */
#define STM32_BOARD_XTAL        24000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL

/* Main PLL Configuration.
 *
 * PLL source is HSE = 24,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 480 MHz
 */

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (24,000,000 / 3) * 100 = 800 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 800 MHz / 2   = 400 MHz
 *   PLL1Q = PLL1_VCO/8  = 800 MHz / 8   = 100 MHz
 *   PLL1R = PLL1_VCO/2  = 800 MHz / 2   = 400 MHz
 */
#define STM32_PLLCFG_PLL1CFG    (RCC_PLLCFGR_PLL1VCOSEL_WIDE|RCC_PLLCFGR_PLL1RGE_4_8_MHZ|RCC_PLLCFGR_DIVP1EN|RCC_PLLCFGR_DIVQ1EN|RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(3)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(100)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(8)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(2)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 3) * 100)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)

/* PLL2 */
#define STM32_PLLCFG_PLL2CFG     (RCC_PLLCFGR_PLL2VCOSEL_WIDE|RCC_PLLCFGR_PLL2RGE_4_8_MHZ|RCC_PLLCFGR_DIVP2EN|RCC_PLLCFGR_DIVQ2EN|RCC_PLLCFGR_DIVR2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(30)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(4)
#define STM32_PLLCFG_PLL2Q       RCC_PLL2DIVR_Q2(5)
#define STM32_PLLCFG_PLL2R       RCC_PLL2DIVR_R2(1)

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 30)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 4)
#define STM32_PLL2Q_FREQUENCY    (STM32_VCO2_FREQUENCY / 5)
#define STM32_PLL2R_FREQUENCY    (STM32_VCO2_FREQUENCY / 1)

/* PLL3 */
#define STM32_PLLCFG_PLL3CFG    (RCC_PLLCFGR_PLL3VCOSEL_WIDE|RCC_PLLCFGR_PLL3RGE_4_8_MHZ|RCC_PLLCFGR_DIVQ3EN)
#define STM32_PLLCFG_PLL3M      RCC_PLLCKSELR_DIVM3(6)
#define STM32_PLLCFG_PLL3N      RCC_PLL3DIVR_N3(72)
#define STM32_PLLCFG_PLL3P      RCC_PLL3DIVR_P3(3)
#define STM32_PLLCFG_PLL3Q      RCC_PLL3DIVR_Q3(6)
#define STM32_PLLCFG_PLL3R      RCC_PLL3DIVR_R3(9)

#define STM32_VCO3_FREQUENCY    ((STM32_HSE_FREQUENCY / 6) * 72)
#define STM32_PLL3P_FREQUENCY   (STM32_VCO3_FREQUENCY / 3)
#define STM32_PLL3Q_FREQUENCY   (STM32_VCO3_FREQUENCY / 6)
#define STM32_PLL3R_FREQUENCY   (STM32_VCO3_FREQUENCY / 9)

/* SYSCLK = PLL1P = 400MHz
 * CPUCLK = SYSCLK / 1 = 400 MHz
 */
#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (240 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4 = 200
 */
#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY            /* same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/4 (120 MHz) */
#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd2       /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB2 clock (PCLK2) is HCLK/2 (120 MHz) */
#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd2       /* PCLK2 = HCLK / 2 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB3 clock (PCLK3) is HCLK/2 (120 MHz) */
#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd2        /* PCLK3 = HCLK / 2 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* APB4 clock (PCLK4) is HCLK/4 (120 MHz) */
#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd2       /* PCLK4 = HCLK / 2 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/2)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */
#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */
#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *  Note: look at Table 54 in ST Manual
 */
#define STM32_RCC_D1CCIPR_SDMMCSEL   RCC_D1CCIPR_SDMMC_PLL1

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI  /* I2C123 clock source */
#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2 /* SPI123 clock source */
#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_PLL2  /* SPI45 clock source */
#define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_PLL3    /* USB 1 and 2 clock source */
#define STM32_RCC_D2CCIP1R_FDCANSEL  RCC_D2CCIP1R_FDCANSEL_HSE   /* FDCAN 1 2 clock source */

#define STM32_RCC_D3CCIPR_ADCSEL     RCC_D3CCIPR_ADCSEL_PLL2     /* ADC 1 2 3 clock source */

/* FLASH wait states */
#define BOARD_FLASH_WAITSTATES 2

/* SDMMC definitions ********************************************************/
/* Init 400kHz, freq = PLL1Q/(2*div)  div =  PLL1Q/(2*freq) */
#define STM32_SDMMC_INIT_CLKDIV     (125 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* 25 MHz Max for now, 25 mHZ = PLL1Q/(2*div), div = PLL1Q/(2*freq)
 * div = 100 / (2*25)
 */
#define STM32_SDMMC_MMCXFR_CLKDIV   (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (2 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE


/* UART/USART */
#define GPIO_USART2_TX   GPIO_USART2_TX_2      /* PD5 */
#define GPIO_USART2_RX   GPIO_USART2_RX_2      /* PD6 */
#define GPIO_USART2_CTS  GPIO_USART2_CTS_NSS_2 /* PD3 */
#define GPIO_USART2_RTS  GPIO_USART2_RTS_2     /* PD4 */

#define GPIO_USART3_TX   GPIO_USART3_TX_3      /* PD8  */
#define GPIO_USART3_RX   GPIO_USART3_RX_3      /* PD9  */
#define GPIO_USART3_CTS  GPIO_USART3_CTS_NSS_2 /* PD11 */
#define GPIO_USART3_RTS  GPIO_USART3_RTS_2     /* PD12 */

#define GPIO_UART4_TX    GPIO_UART4_TX_2       /* PA0 */
#define GPIO_UART4_RX    GPIO_UART4_RX_2       /* PA1 */

#define GPIO_USART6_TX   GPIO_USART6_TX_1      /* PC6 */
#define GPIO_USART6_RX   GPIO_USART6_RX_1      /* PC7 */

#define GPIO_UART7_TX    GPIO_UART7_TX_3       /* PE8 */
#define GPIO_UART7_RX    GPIO_UART7_RX_3       /* PE7 */

#define GPIO_UART8_TX    GPIO_UART8_TX_1       /* PE1 */
#define GPIO_UART8_RX    GPIO_UART8_RX_1       /* PE0 */


/* CAN */
#define GPIO_CAN1_RX     GPIO_CAN1_RX_3        /* PD0  */
#define GPIO_CAN1_TX     GPIO_CAN1_TX_3        /* PD1  */

#define GPIO_CAN2_RX     GPIO_CAN2_RX_1        /* PB12 */
#define GPIO_CAN2_TX     GPIO_CAN2_TX_2        /* PB6  */


/* SPI */
#define ADJ_SLEW_RATE(p) (((p) & ~GPIO_SPEED_MASK) | (GPIO_SPEED_2MHz))

#define GPIO_SPI1_SCK    ADJ_SLEW_RATE(GPIO_SPI1_SCK_1) /* PA5  */
#define GPIO_SPI1_MISO   GPIO_SPI1_MISO_1               /* PA6  */
#define GPIO_SPI1_MOSI   GPIO_SPI1_MOSI_1               /* PA7  */

#define GPIO_SPI2_SCK    ADJ_SLEW_RATE(GPIO_SPI2_SCK_4) /* PB13 */
#define GPIO_SPI2_MISO   GPIO_SPI2_MISO_1               /* PB14 */
#define GPIO_SPI2_MOSI   GPIO_SPI2_MOSI_1               /* PB15 */

#define GPIO_SPI4_SCK    ADJ_SLEW_RATE(GPIO_SPI4_SCK_2) /* PE2  */
#define GPIO_SPI4_MISO   GPIO_SPI4_MISO_2               /* PE5  */
#define GPIO_SPI4_MOSI   GPIO_SPI4_MOSI_2               /* PE6  */


/* I2C */
#define GPIO_I2C1_SCL GPIO_I2C1_SCL_2       /* PB8  */
#define GPIO_I2C1_SDA GPIO_I2C1_SDA_2       /* PB9  */

#define GPIO_I2C2_SCL GPIO_I2C2_SCL_1       /* PB10 */
#define GPIO_I2C2_SDA GPIO_I2C2_SDA_1       /* PB11 */
