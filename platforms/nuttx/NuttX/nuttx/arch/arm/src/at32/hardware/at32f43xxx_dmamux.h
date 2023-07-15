/****************************************************************************
 * arch/arm/src/at32/hardware/at32f43xxx_dmamux.h
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_DMAMUX_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_DMAMUX_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMAMUX1 mapping **********************************************************/

#define DMAMUX_REQ_MEM2MEM            (0)                   /* Memory to memory transfer */
#define DMAMUX_REQ_GEN0               (1)                   /* DMAMUX Request Generator 0 */
#define DMAMUX_REQ_GEN1               (2)                   /* DMAMUX Request Generator 1 */
#define DMAMUX_REQ_GEN2               (3)                   /* DMAMUX Request Generator 2 */
#define DMAMUX_REQ_GEN3               (4)                   /* DMAMUX Request Generator 3 */
#define DMAMUX_ADC1                   (5)                   /* DMAMUX ADC1 request */
#define DMAMUX_DAC1_CH1               (6)                   /* DMAMUX DAC1 Channel 1 request */
#define DMAMUX_TIM6_UP                (8)                   /* DMAMUX TIM6 Update request */
#define DMAMUX_TIM7_UP                (9)                   /* DMAMUX TIM7 Update request */
#define DMAMUX_SPI1_RX                (10)                  /* DMAMUX SPI1 Rx request */
#define DMAMUX_SPI1_TX                (11)                  /* DMAMUX SPI1 Tx request */
#define DMAMUX_SPI2_RX                (12)                  /* DMAMUX SPI2 Rx request */
#define DMAMUX_SPI2_TX                (13)                  /* DMAMUX SPI2 Tx request */
#define DMAMUX_SPI3_RX                (14)                  /* DMAMUX SPI3 Rx request */
#define DMAMUX_SPI3_TX                (15)                  /* DMAMUX SPI3 Tx request */
#define DMAMUX_I2C1_RX                (16)                  /* DMAMUX I2C1 Rx request */
#define DMAMUX_I2C1_TX                (17)                  /* DMAMUX I2C1 Tx request */
#define DMAMUX_I2C2_RX                (18)                  /* DMAMUX I2C2 Rx request */
#define DMAMUX_I2C2_TX                (19)                  /* DMAMUX I2C2 Tx request */
#define DMAMUX_I2C3_RX                (20)                  /* DMAMUX I2C3 Rx request */
#define DMAMUX_I2C3_TX                (21)                  /* DMAMUX I2C3 Tx request */
#define DMAMUX_USART1_RX              (24)                  /* DMAMUX USART1 Rx request */
#define DMAMUX_USART1_TX              (25)                  /* DMAMUX USART1 Tx request */
#define DMAMUX_USART2_RX              (26)                  /* DMAMUX USART2 Rx request */
#define DMAMUX_USART2_TX              (27)                  /* DMAMUX USART2 Tx request */
#define DMAMUX_USART3_RX              (28)                  /* DMAMUX USART3 Rx request */
#define DMAMUX_USART3_TX              (29)                  /* DMAMUX USART3 Tx request */
#define DMAMUX_UART4_RX               (30)                  /* DMAMUX UART4 Rx request */
#define DMAMUX_UART4_TX               (31)                  /* DMAMUX UART4 Tx request */
#define DMAMUX_UART5_RX               (32)                  /* DMAMUX UART5 Rx request */
#define DMAMUX_UART5_TX               (33)                  /* DMAMUX UART5 Tx request */
#define DMAMUX_ADC2                   (36)                  /* DMAMUX ADC2 request */
#define DMAMUX_ADC3                   (37)                  /* DMAMUX ADC3 request */
#define DMAMUX_SDIO                   (39)                  /* DMAMUX SDIO1 request */
#define DMAMUX_QSPI                   (40)                  /* DMAMUX QSPI request */
#define DMAMUX_DAC2_CH1               (41)                  /* DMAMUX DAC2 Channel 1 request */
#define DMAMUX_TIM1_CH1               (42)                  /* DMAMUX TIM1 Channel 1 request */
#define DMAMUX_TIM1_CH2               (43)                  /* DMAMUX TIM1 Channel 2 request */
#define DMAMUX_TIM1_CH3               (44)                  /* DMAMUX TIM1 Channel 3 request */
#define DMAMUX_TIM1_CH4               (45)                  /* DMAMUX TIM1 Channel 4 request */
#define DMAMUX_TIM1_UP                (46)                  /* DMAMUX TIM1 Update request */
#define DMAMUX_TIM1_TRIG              (47)                  /* DMAMUX TIM1 Trigger request */
#define DMAMUX_TIM1_COM               (48)                  /* DMAMUX TIM1 Commutation request */
#define DMAMUX_TIM8_CH1               (49)                  /* DMAMUX TIM8 Channel 1 request */
#define DMAMUX_TIM8_CH2               (50)                  /* DMAMUX TIM8 Channel 2 request */
#define DMAMUX_TIM8_CH3               (51)                  /* DMAMUX TIM8 Channel 3 request */
#define DMAMUX_TIM8_CH4               (52)                  /* DMAMUX TIM8 Channel 4 request */
#define DMAMUX_TIM8_UP                (53)                  /* DMAMUX TIM8 Update request */
#define DMAMUX_TIM8_TRIG              (54)                  /* DMAMUX TIM8 Trigger request */
#define DMAMUX_TIM8_COM               (55)                  /* DMAMUX TIM8 Commutation request */
#define DMAMUX_TIM2_CH1               (56)                  /* DMAMUX TIM2 Channel 1 request */
#define DMAMUX_TIM2_CH2               (57)                  /* DMAMUX TIM2 Channel 2 request */
#define DMAMUX_TIM2_CH3               (58)                  /* DMAMUX TIM2 Channel 3 request */
#define DMAMUX_TIM2_CH4               (59)                  /* DMAMUX TIM2 Channel 4 request */
#define DMAMUX_TIM2_UP                (60)                  /* DMAMUX TIM2 Update request */
#define DMAMUX_TIM3_CH1               (61)                  /* DMAMUX TIM3 Channel 1 request */
#define DMAMUX_TIM3_CH2               (62)                  /* DMAMUX TIM3 Channel 2 request */
#define DMAMUX_TIM3_CH3               (63)                  /* DMAMUX TIM3 Channel 3 request */
#define DMAMUX_TIM3_CH4               (64)                  /* DMAMUX TIM3 Channel 4 request */
#define DMAMUX_TIM3_UP                (65)                  /* DMAMUX TIM3 Update request */
#define DMAMUX_TIM3_TRIG              (66)                  /* DMAMUX TIM3 Trigger request */
#define DMAMUX_TIM4_CH1               (67)                  /* DMAMUX TIM4 Channel 1 request */
#define DMAMUX_TIM4_CH2               (68)                  /* DMAMUX TIM4 Channel 2 request */
#define DMAMUX_TIM4_CH3               (69)                  /* DMAMUX TIM4 Channel 3 request */
#define DMAMUX_TIM4_CH4               (70)                  /* DMAMUX TIM4 Channel 4 request */
#define DMAMUX_TIM4_UP                (71)                  /* DMAMUX TIM4 Update request */
#define DMAMUX_TIM5_CH1               (72)                  /* DMAMUX TIM5 Channel 1 request */
#define DMAMUX_TIM5_CH2               (73)                  /* DMAMUX TIM5 Channel 2 request */
#define DMAMUX_TIM5_CH3               (74)                  /* DMAMUX TIM5 Channel 3 request */
#define DMAMUX_TIM5_CH4               (75)                  /* DMAMUX TIM5 Channel 4 request */
#define DMAMUX_TIM5_UP                (76)                  /* DMAMUX TIM5 Update request */
#define DMAMUX_TIM5_TRIG              (77)                  /* DMAMUX TIM5 Trigger request */
#define DMAMUX_TIM20_CH1              (86)                  /* DMAMUX TIM20 Channel 1 request */
#define DMAMUX_TIM20_CH2              (87)                  /* DMAMUX TIM20 Channel 2 request */
#define DMAMUX_TIM20_CH3              (88)                  /* DMAMUX TIM20 Channel 3 request */
#define DMAMUX_TIM20_CH4              (89)                  /* DMAMUX TIM20 Channel 4 request */
#define DMAMUX_TIM20_UP               (90)                  /* DMAMUX TIM20 Update request */
#define DMAMUX_TIM20_TRIG             (93)                  /* DMAMUX TIM20 Trigger request */
#define DMAMUX_TIM20_COM              (94)                  /* DMAMUX TIM20 Commutation request */
#define DMAMUX_SDIO2                  (103)                 /* DMAMUX DAC3 Channel 2 request */
#define DMAMUX_QSPI2                  (104)                 /* DMAMUX DAC4 Channel 1 request */
#define DMAMUX_DVP                    (105)                 /* DMAMUX DAC4 Channel 2 request */
#define DMAMUX_SPI4_RX                (106)                 /* DMAMUX SPI4 Rx request */
#define DMAMUX_SPI4_TX                (107)                 /* DMAMUX SPI4 Tx request */
#define DMAMUX_I2S2_EXT_RX            (110)                 /* DMAMUX I2S2_EXT_RX request */
#define DMAMUX_I2S2_EXT_TX            (111)                 /* DMAMUX I2S2_EXT_TX request */
#define DMAMUX_I2S3_EXT_RX            (112)                 /* DMAMUX I2S3_EXT_RX request */
#define DMAMUX_I2S3_EXT_TX            (113)                 /* DMAMUX I2S3_EXT_TX request */
#define DMAMUX_USART6_RX              (114)                 /* DMAMUX UART6 Rx request */
#define DMAMUX_USART6_TX              (115)                 /* DMAMUX UART6 Tx request */
#define DMAMUX_UART7_RX               (116)                 /* DMAMUX UART7 Rx request */
#define DMAMUX_UART7_TX               (117)                 /* DMAMUX UART7 Tx request */
#define DMAMUX_UART8_RX               (118)                 /* DMAMUX UART8 Rx request */
#define DMAMUX_UART8_TX               (119)                 /* DMAMUX UART8 Tx request */
#define DMAMUX_TIM2_TRIG              (126)                 /* DMAMUX TIM2 Trigger request */


#if 0
/* DMAMAP for DMA1 and DMA2 (DMAMUX1) */

#define DMAMAP_DMA12_MEM2MEM_0     DMAMAP_MAP(DMA1, DMAMUX_MEM2MEM)
#define DMAMAP_DMA12_MEM2MEM_1     DMAMAP_MAP(DMA2, DMAMUX_MEM2MEM)
#define DMAMAP_DMA12_REQGEN0_0     DMAMAP_MAP(DMA1, DMAMUX_REQ_GEN0)
#define DMAMAP_DMA12_REQGEN0_1     DMAMAP_MAP(DMA2, DMAMUX_REQ_GEN0)
#define DMAMAP_DMA12_REQGEN1_0     DMAMAP_MAP(DMA1, DMAMUX_REQ_GEN1)
#define DMAMAP_DMA12_REQGEN1_1     DMAMAP_MAP(DMA2, DMAMUX_REQ_GEN1)
#define DMAMAP_DMA12_REQGEN2_0     DMAMAP_MAP(DMA1, DMAMUX_REQ_GEN2)
#define DMAMAP_DMA12_REQGEN2_1     DMAMAP_MAP(DMA2, DMAMUX_REQ_GEN2)
#define DMAMAP_DMA12_REQGEN3_0     DMAMAP_MAP(DMA1, DMAMUX_REQ_GEN3)
#define DMAMAP_DMA12_REQGEN3_1     DMAMAP_MAP(DMA2, DMAMUX_REQ_GEN3)
#define DMAMAP_DMA12_ADC1_0        DMAMAP_MAP(DMA1, DMAMUX_ADC1)
#define DMAMAP_DMA12_ADC1_1        DMAMAP_MAP(DMA2, DMAMUX_ADC1)
#define DMAMAP_DMA12_DAC1CH1_0     DMAMAP_MAP(DMA1, DMAMUX_DAC1_CH1)
#define DMAMAP_DMA12_DAC1CH1_1     DMAMAP_MAP(DMA2, DMAMUX_DAC1_CH1)
#define DMAMAP_DMA12_TIM6UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM6_UP)
#define DMAMAP_DMA12_TIM6UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM6_UP)
#define DMAMAP_DMA12_TIM7UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM7_UP)
#define DMAMAP_DMA12_TIM7UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM7_UP)
#define DMAMAP_DMA12_SPI1RX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI1_RX)
#define DMAMAP_DMA12_SPI1RX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI1_RX)
#define DMAMAP_DMA12_SPI1TX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI1_TX)
#define DMAMAP_DMA12_SPI1TX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI1_TX)
#define DMAMAP_DMA12_SPI2RX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI2_RX)
#define DMAMAP_DMA12_SPI2RX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI2_RX)
#define DMAMAP_DMA12_SPI2TX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI2_TX)
#define DMAMAP_DMA12_SPI2TX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI2_TX)
#define DMAMAP_DMA12_SPI3RX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI3_RX)
#define DMAMAP_DMA12_SPI3RX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI3_RX)
#define DMAMAP_DMA12_SPI3TX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI3_TX)
#define DMAMAP_DMA12_SPI3TX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI3_TX)
#define DMAMAP_DMA12_I2C1RX_0      DMAMAP_MAP(DMA1, DMAMUX_I2C1_RX)
#define DMAMAP_DMA12_I2C1RX_1      DMAMAP_MAP(DMA2, DMAMUX_I2C1_RX)
#define DMAMAP_DMA12_I2C1TX_0      DMAMAP_MAP(DMA1, DMAMUX_I2C1_TX)
#define DMAMAP_DMA12_I2C1TX_1      DMAMAP_MAP(DMA2, DMAMUX_I2C1_TX)
#define DMAMAP_DMA12_I2C2RX_0      DMAMAP_MAP(DMA1, DMAMUX_I2C2_RX)
#define DMAMAP_DMA12_I2C2RX_1      DMAMAP_MAP(DMA2, DMAMUX_I2C2_RX)
#define DMAMAP_DMA12_I2C2TX_0      DMAMAP_MAP(DMA1, DMAMUX_I2C2_TX)
#define DMAMAP_DMA12_I2C2TX_1      DMAMAP_MAP(DMA2, DMAMUX_I2C2_TX)
#define DMAMAP_DMA12_I2C3RX_0      DMAMAP_MAP(DMA1, DMAMUX_I2C3_RX)
#define DMAMAP_DMA12_I2C3RX_1      DMAMAP_MAP(DMA2, DMAMUX_I2C3_RX)
#define DMAMAP_DMA12_I2C3TX_0      DMAMAP_MAP(DMA1, DMAMUX_I2C3_TX)
#define DMAMAP_DMA12_I2C3TX_1      DMAMAP_MAP(DMA2, DMAMUX_I2C3_TX)


#define DMAMAP_DMA12_USART1RX_0    DMAMAP_MAP(DMA1, DMAMUX_USART1_RX)
#define DMAMAP_DMA12_USART1RX_1    DMAMAP_MAP(DMA2, DMAMUX_USART1_RX)
#define DMAMAP_DMA12_USART1TX_0    DMAMAP_MAP(DMA1, DMAMUX_USART1_TX)
#define DMAMAP_DMA12_USART1TX_1    DMAMAP_MAP(DMA2, DMAMUX_USART1_TX)
#define DMAMAP_DMA12_USART2RX_0    DMAMAP_MAP(DMA1, DMAMUX_USART2_RX)
#define DMAMAP_DMA12_USART2RX_1    DMAMAP_MAP(DMA2, DMAMUX_USART2_RX)
#define DMAMAP_DMA12_USART2TX_0    DMAMAP_MAP(DMA1, DMAMUX_USART2_TX)
#define DMAMAP_DMA12_USART2TX_1    DMAMAP_MAP(DMA2, DMAMUX_USART2_TX)
#define DMAMAP_DMA12_USART3RX_0    DMAMAP_MAP(DMA1, DMAMUX_USART3_RX)
#define DMAMAP_DMA12_USART3RX_1    DMAMAP_MAP(DMA2, DMAMUX_USART3_RX)
#define DMAMAP_DMA12_USART3TX_0    DMAMAP_MAP(DMA1, DMAMUX_USART3_TX)
#define DMAMAP_DMA12_USART3TX_1    DMAMAP_MAP(DMA2, DMAMUX_USART3_TX)
#define DMAMAP_DMA12_UART4RX_0     DMAMAP_MAP(DMA1, DMAMUX_UART4_RX)
#define DMAMAP_DMA12_UART4RX_1     DMAMAP_MAP(DMA2, DMAMUX_UART4_RX)
#define DMAMAP_DMA12_UART4TX_0     DMAMAP_MAP(DMA1, DMAMUX_UART4_TX)
#define DMAMAP_DMA12_UART4TX_1     DMAMAP_MAP(DMA2, DMAMUX_UART4_TX)
#define DMAMAP_DMA12_UART5RX_0     DMAMAP_MAP(DMA1, DMAMUX_UART5_RX)
#define DMAMAP_DMA12_UART5RX_1     DMAMAP_MAP(DMA2, DMAMUX_UART5_RX)
#define DMAMAP_DMA12_UART5TX_0     DMAMAP_MAP(DMA1, DMAMUX_UART5_TX)
#define DMAMAP_DMA12_UART5TX_1     DMAMAP_MAP(DMA2, DMAMUX_UART5_TX)

#define DMAMAP_DMA12_ADC2_0        DMAMAP_MAP(DMA1, DMAMUX_ADC2)
#define DMAMAP_DMA12_ADC2_1        DMAMAP_MAP(DMA2, DMAMUX_ADC2)
#define DMAMAP_DMA12_ADC3_0        DMAMAP_MAP(DMA1, DMAMUX_ADC3)
#define DMAMAP_DMA12_ADC3_1        DMAMAP_MAP(DMA2, DMAMUX_ADC3)

#define DMAMAP_DMA12_SDIO_0        DMAMAP_MAP(DMA1, DMAMUX_SDIO)
#define DMAMAP_DMA12_SDIO_1        DMAMAP_MAP(DMA2, DMAMUX_SDIO)
#define DMAMAP_DMA12_QSPI_0        DMAMAP_MAP(DMA1, DMAMUX_QSPI)
#define DMAMAP_DMA12_QSPI_1        DMAMAP_MAP(DMA2, DMAMUX_QSPI)
#define DMAMAP_DMA12_DAC2CH1_0     DMAMAP_MAP(DMA1, DMAMUX_DAC2_CH1)
#define DMAMAP_DMA12_DAC2CH1_1     DMAMAP_MAP(DMA2, DMAMUX_DAC2_CH1)
#define DMAMAP_DMA12_TIM1CH1_0     DMAMAP_MAP(DMA1, DMAMUX_TIM1_CH1)
#define DMAMAP_DMA12_TIM1CH1_1     DMAMAP_MAP(DMA2, DMAMUX_TIM1_CH1)
#define DMAMAP_DMA12_TIM1CH2_0     DMAMAP_MAP(DMA1, DMAMUX_TIM1_CH2)
#define DMAMAP_DMA12_TIM1CH2_1     DMAMAP_MAP(DMA2, DMAMUX_TIM1_CH2)
#define DMAMAP_DMA12_TIM1CH3_0     DMAMAP_MAP(DMA1, DMAMUX_TIM1_CH3)
#define DMAMAP_DMA12_TIM1CH3_1     DMAMAP_MAP(DMA2, DMAMUX_TIM1_CH3)
#define DMAMAP_DMA12_TIM1CH4_0     DMAMAP_MAP(DMA1, DMAMUX_TIM1_CH4)
#define DMAMAP_DMA12_TIM1CH4_1     DMAMAP_MAP(DMA2, DMAMUX_TIM1_CH4)
#define DMAMAP_DMA12_TIM1UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM1_UP)
#define DMAMAP_DMA12_TIM1UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM1_UP)
#define DMAMAP_DMA12_TIM1TRIG_0    DMAMAP_MAP(DMA1, DMAMUX_TIM1_TRIG)
#define DMAMAP_DMA12_TIM1TRIG_1    DMAMAP_MAP(DMA2, DMAMUX_TIM1_TRIG)
#define DMAMAP_DMA12_TIM1COM_0     DMAMAP_MAP(DMA1, DMAMUX_TIM1_COM)
#define DMAMAP_DMA12_TIM1COM_1     DMAMAP_MAP(DMA2, DMAMUX_TIM1_COM)
#define DMAMAP_DMA12_TIM8CH1_0     DMAMAP_MAP(DMA1, DMAMUX_TIM8_CH1)
#define DMAMAP_DMA12_TIM8CH1_1     DMAMAP_MAP(DMA2, DMAMUX_TIM8_CH1)
#define DMAMAP_DMA12_TIM8CH2_0     DMAMAP_MAP(DMA1, DMAMUX_TIM8_CH2)
#define DMAMAP_DMA12_TIM8CH2_1     DMAMAP_MAP(DMA2, DMAMUX_TIM8_CH2)
#define DMAMAP_DMA12_TIM8CH3_0     DMAMAP_MAP(DMA1, DMAMUX_TIM8_CH3)
#define DMAMAP_DMA12_TIM8CH3_1     DMAMAP_MAP(DMA2, DMAMUX_TIM8_CH3)
#define DMAMAP_DMA12_TIM8CH4_0     DMAMAP_MAP(DMA1, DMAMUX_TIM8_CH4)
#define DMAMAP_DMA12_TIM8CH4_1     DMAMAP_MAP(DMA2, DMAMUX_TIM8_CH4)
#define DMAMAP_DMA12_TIM8UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM8_UP)
#define DMAMAP_DMA12_TIM8UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM8_UP)
#define DMAMAP_DMA12_TIM8TRIG_0    DMAMAP_MAP(DMA1, DMAMUX_TIM8_TRIG)
#define DMAMAP_DMA12_TIM8TRIG_1    DMAMAP_MAP(DMA2, DMAMUX_TIM8_TRIG)
#define DMAMAP_DMA12_TIM8COM_0     DMAMAP_MAP(DMA1, DMAMUX_TIM8_COM)
#define DMAMAP_DMA12_TIM8COM_1     DMAMAP_MAP(DMA2, DMAMUX_TIM8_COM)
#define DMAMAP_DMA12_TIM2CH1_0     DMAMAP_MAP(DMA1, DMAMUX_TIM2_CH1)
#define DMAMAP_DMA12_TIM2CH1_1     DMAMAP_MAP(DMA2, DMAMUX_TIM2_CH1)
#define DMAMAP_DMA12_TIM2CH2_0     DMAMAP_MAP(DMA1, DMAMUX_TIM2_CH2)
#define DMAMAP_DMA12_TIM2CH2_1     DMAMAP_MAP(DMA2, DMAMUX_TIM2_CH2)
#define DMAMAP_DMA12_TIM2CH3_0     DMAMAP_MAP(DMA1, DMAMUX_TIM2_CH3)
#define DMAMAP_DMA12_TIM2CH3_1     DMAMAP_MAP(DMA2, DMAMUX_TIM2_CH3)
#define DMAMAP_DMA12_TIM2CH4_0     DMAMAP_MAP(DMA1, DMAMUX_TIM2_CH4)
#define DMAMAP_DMA12_TIM2CH4_1     DMAMAP_MAP(DMA2, DMAMUX_TIM2_CH4)
#define DMAMAP_DMA12_TIM2UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM2_UP)
#define DMAMAP_DMA12_TIM2UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM2_UP)
#define DMAMAP_DMA12_TIM3CH1_0     DMAMAP_MAP(DMA1, DMAMUX_TIM3_CH1)
#define DMAMAP_DMA12_TIM3CH1_1     DMAMAP_MAP(DMA2, DMAMUX_TIM3_CH1)
#define DMAMAP_DMA12_TIM3CH2_0     DMAMAP_MAP(DMA1, DMAMUX_TIM3_CH2)
#define DMAMAP_DMA12_TIM3CH2_1     DMAMAP_MAP(DMA2, DMAMUX_TIM3_CH2)
#define DMAMAP_DMA12_TIM3CH3_0     DMAMAP_MAP(DMA1, DMAMUX_TIM3_CH3)
#define DMAMAP_DMA12_TIM3CH3_1     DMAMAP_MAP(DMA2, DMAMUX_TIM3_CH3)
#define DMAMAP_DMA12_TIM3CH4_0     DMAMAP_MAP(DMA1, DMAMUX_TIM3_CH4)
#define DMAMAP_DMA12_TIM3CH4_1     DMAMAP_MAP(DMA2, DMAMUX_TIM3_CH4)
#define DMAMAP_DMA12_TIM3UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM3_UP)
#define DMAMAP_DMA12_TIM3UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM3_UP)
#define DMAMAP_DMA12_TIM3TRIG_0    DMAMAP_MAP(DMA1, DMAMUX_TIM3_TRIG)
#define DMAMAP_DMA12_TIM3TRIG_1    DMAMAP_MAP(DMA2, DMAMUX_TIM3_TRIG)
#define DMAMAP_DMA12_TIM4CH1_0     DMAMAP_MAP(DMA1, DMAMUX_TIM4_CH1)
#define DMAMAP_DMA12_TIM4CH1_1     DMAMAP_MAP(DMA2, DMAMUX_TIM4_CH1)
#define DMAMAP_DMA12_TIM4CH2_0     DMAMAP_MAP(DMA1, DMAMUX_TIM4_CH2)
#define DMAMAP_DMA12_TIM4CH2_1     DMAMAP_MAP(DMA2, DMAMUX_TIM4_CH2)
#define DMAMAP_DMA12_TIM4CH3_0     DMAMAP_MAP(DMA1, DMAMUX_TIM4_CH3)
#define DMAMAP_DMA12_TIM4CH3_1     DMAMAP_MAP(DMA2, DMAMUX_TIM4_CH3)
#define DMAMAP_DMA12_TIM4CH4_0     DMAMAP_MAP(DMA1, DMAMUX_TIM4_CH4)
#define DMAMAP_DMA12_TIM4CH4_1     DMAMAP_MAP(DMA2, DMAMUX_TIM4_CH4)
#define DMAMAP_DMA12_TIM4UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM4_UP)
#define DMAMAP_DMA12_TIM4UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM4_UP)
#define DMAMAP_DMA12_TIM5CH1_0     DMAMAP_MAP(DMA1, DMAMUX_TIM5_CH1)
#define DMAMAP_DMA12_TIM5CH1_1     DMAMAP_MAP(DMA2, DMAMUX_TIM5_CH1)
#define DMAMAP_DMA12_TIM5CH2_0     DMAMAP_MAP(DMA1, DMAMUX_TIM5_CH2)
#define DMAMAP_DMA12_TIM5CH2_1     DMAMAP_MAP(DMA2, DMAMUX_TIM5_CH2)
#define DMAMAP_DMA12_TIM5CH3_0     DMAMAP_MAP(DMA1, DMAMUX_TIM5_CH3)
#define DMAMAP_DMA12_TIM5CH3_1     DMAMAP_MAP(DMA2, DMAMUX_TIM5_CH3)
#define DMAMAP_DMA12_TIM5CH4_0     DMAMAP_MAP(DMA1, DMAMUX_TIM5_CH4)
#define DMAMAP_DMA12_TIM5CH4_1     DMAMAP_MAP(DMA2, DMAMUX_TIM5_CH4)
#define DMAMAP_DMA12_TIM5UP_0      DMAMAP_MAP(DMA1, DMAMUX_TIM5_UP)
#define DMAMAP_DMA12_TIM5UP_1      DMAMAP_MAP(DMA2, DMAMUX_TIM5_UP)
#define DMAMAP_DMA12_TIM5TRIG_0    DMAMAP_MAP(DMA1, DMAMUX_TIM5_TRIG)
#define DMAMAP_DMA12_TIM5TRIG_1    DMAMAP_MAP(DMA2, DMAMUX_TIM5_TRIG)


#define DMAMAP_DMA12_TIM20CH1_0    DMAMAP_MAP(DMA1, DMAMUX_TIM20_CH1)
#define DMAMAP_DMA12_TIM20CH1_1    DMAMAP_MAP(DMA2, DMAMUX_TIM20_CH1)
#define DMAMAP_DMA12_TIM20CH2_0    DMAMAP_MAP(DMA1, DMAMUX_TIM20_CH2)
#define DMAMAP_DMA12_TIM20CH2_1    DMAMAP_MAP(DMA2, DMAMUX_TIM20_CH2)
#define DMAMAP_DMA12_TIM20CH3_0    DMAMAP_MAP(DMA1, DMAMUX_TIM20_CH3)
#define DMAMAP_DMA12_TIM20CH3_1    DMAMAP_MAP(DMA2, DMAMUX_TIM20_CH3)
#define DMAMAP_DMA12_TIM20CH4_0    DMAMAP_MAP(DMA1, DMAMUX_TIM20_CH4)
#define DMAMAP_DMA12_TIM20CH4_1    DMAMAP_MAP(DMA2, DMAMUX_TIM20_CH4)
#define DMAMAP_DMA12_TIM20UP_0     DMAMAP_MAP(DMA1, DMAMUX_TIM20_UP)
#define DMAMAP_DMA12_TIM20UP_1     DMAMAP_MAP(DMA2, DMAMUX_TIM20_UP)

#define DMAMAP_DMA12_TIM20TRIG_0   DMAMAP_MAP(DMA1, DMAMUX_TIM20_TRIG)
#define DMAMAP_DMA12_TIM20TRIG_1   DMAMAP_MAP(DMA2, DMAMUX_TIM20_TRIG)
#define DMAMAP_DMA12_TIM20COM_0    DMAMAP_MAP(DMA1, DMAMUX_TIM20_COM)
#define DMAMAP_DMA12_TIM20COM_1    DMAMAP_MAP(DMA2, DMAMUX_TIM20_COM)

#define DMAMAP_DMA12_SDIO2_0       DMAMAP_MAP(DMA1, DMAMUX_SDIO2)
#define DMAMAP_DMA12_SDIO2_1       DMAMAP_MAP(DMA2, DMAMUX_SDIO2)
#define DMAMAP_DMA12_QSPI2_0       DMAMAP_MAP(DMA1, DMAMUX_QSPI2)
#define DMAMAP_DMA12_QSPI2_1       DMAMAP_MAP(DMA2, DMAMUX_QSPI2)
#define DMAMAP_DMA12_DVP_0         DMAMAP_MAP(DMA1, DMAMUX_DVP)
#define DMAMAP_DMA12_DVP_1         DMAMAP_MAP(DMA2, DMAMUX_DVP)
#define DMAMAP_DMA12_SPI4RX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI4_RX)
#define DMAMAP_DMA12_SPI4RX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI4_RX)
#define DMAMAP_DMA12_SPI4TX_0      DMAMAP_MAP(DMA1, DMAMUX_SPI4_TX)
#define DMAMAP_DMA12_SPI4TX_1      DMAMAP_MAP(DMA2, DMAMUX_SPI4_TX)
#define DMAMAP_DMA12_SAI1A_0       DMAMAP_MAP(DMA1, DMAMUX_SAI1_A)
#define DMAMAP_DMA12_SAI1A_1       DMAMAP_MAP(DMA2, DMAMUX_SAI1_A)
#define DMAMAP_DMA12_SAI1B_0       DMAMAP_MAP(DMA1, DMAMUX_SAI1_B)
#define DMAMAP_DMA12_SAI1B_1       DMAMAP_MAP(DMA2, DMAMUX_SAI1_B)
#define DMAMAP_DMA12_I2S2_EXT_RX_0   DMAMAP_MAP(DMA1, DMAMUX_I2S2_EXT_RX)
#define DMAMAP_DMA12_I2S2_EXT_RX_1   DMAMAP_MAP(DMA2, DMAMUX_I2S2_EXT_RX)
#define DMAMAP_DMA12_I2S2_EXT_TX_0   DMAMAP_MAP(DMA1, DMAMUX_I2S2_EXT_TX)
#define DMAMAP_DMA12_I2S2_EXT_TX_1   DMAMAP_MAP(DMA2, DMAMUX_I2S2_EXT_TX)
#define DMAMAP_DMA12_I2S3_EXT_RX_0   DMAMAP_MAP(DMA1, DMAMUX_I2S3_EXT_RX)
#define DMAMAP_DMA12_I2S3_EXT_RX_1   DMAMAP_MAP(DMA2, DMAMUX_I2S3_EXT_RX)
#define DMAMAP_DMA12_I2S3_EXT_TX_0   DMAMAP_MAP(DMA1, DMAMUX_I2S3_EXT_TX)
#define DMAMAP_DMA12_I2S3_EXT_TX_1   DMAMAP_MAP(DMA2, DMAMUX_I2S3_EXT_TX)

#define DMAMAP_DMA12_UART6RX_0     DMAMAP_MAP(DMA1, DMAMUX_UART6_RX)
#define DMAMAP_DMA12_UART6RX_1     DMAMAP_MAP(DMA2, DMAMUX_UART6_RX)
#define DMAMAP_DMA12_UART6TX_0     DMAMAP_MAP(DMA1, DMAMUX_UART6_TX)
#define DMAMAP_DMA12_UART6TX_1     DMAMAP_MAP(DMA2, DMAMUX_UART6_TX)
#define DMAMAP_DMA12_UART7RX_0     DMAMAP_MAP(DMA1, DMAMUX_UART7_RX)
#define DMAMAP_DMA12_UART7RX_1     DMAMAP_MAP(DMA2, DMAMUX_UART7_RX)
#define DMAMAP_DMA12_UART7TX_0     DMAMAP_MAP(DMA1, DMAMUX_UART7_TX)
#define DMAMAP_DMA12_UART7TX_1     DMAMAP_MAP(DMA2, DMAMUX_UART7_TX)
#define DMAMAP_DMA12_UART8RX_0     DMAMAP_MAP(DMA1, DMAMUX_UART8_RX)
#define DMAMAP_DMA12_UART8RX_1     DMAMAP_MAP(DMA2, DMAMUX_UART8_RX)
#define DMAMAP_DMA12_UART8TX_0     DMAMAP_MAP(DMA1, DMAMUX_UART8_TX)
#define DMAMAP_DMA12_UART8TX_1     DMAMAP_MAP(DMA2, DMAMUX_UART8_TX)
#define DMAMAP_DMA12_TIM2TRIG_0    DMAMAP_MAP(DMA1, DMAMUX_TIM2_TRIG)
#define DMAMAP_DMA12_TIM2TRIG_1    DMAMAP_MAP(DMA2, DMAMUX_TIM2_TRIG)

#endif

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32F43XXX_DMAMUX_H */
