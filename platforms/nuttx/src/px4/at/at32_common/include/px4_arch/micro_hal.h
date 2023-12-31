/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include <px4_platform/micro_hal.h>

__BEGIN_DECLS

#include <at32_tim.h>
#include <at32_spi.h>
#include <at32_i2c.h>


// Fixme: using ??
#define PX4_BBSRAM_SIZE             2048
#define PX4_HF_GETDESC_IOCTL        PROGMEM_DUMP_GETDESC_IOCTL
//#define PX4_NUMBER_I2C_BUSES        2


/* AT32/32F7 defines the 96 bit UUID as
 *  init32_t[3] that can be read as bytes/half-words/words
 *  init32_t[0] PX4_CPU_UUID_ADDRESS[0] bits 31:0  (offset 0)
 *  init32_t[1] PX4_CPU_UUID_ADDRESS[1] bits 63:32 (offset 4)
 *  init32_t[2] PX4_CPU_UUID_ADDRESS[3] bits 96:64 (offset 8)
 *
 * The original PX4 at32 (legacy) based implementation **displayed** the
 * UUID as: ABCD EFGH IJKL
 * Where:
 *       A was bit 31 and D was bit 0
 *       E was bit 63 and H was bit 32
 *       I was bit 95 and L was bit 64
 *
 * Since the string was used by some manufactures to identify the units
 * it must be preserved.
 *
 * For new targets moving forward we will use
 *      IJKL EFGH ABCD
 */
#define PX4_CPU_UUID_BYTE_LENGTH                12
#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

/* The mfguid will be an array of bytes with
 * MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
 *
 * It will be converted to a string with the MSD on left and LSD on the right most position.
 */
#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

/* By not defining PX4_CPU_UUID_CORRECT_CORRELATION the following maintains the legacy incorrect order
 * used for selection of significant digits of the UUID in the PX4 code base.
 * This is done to avoid the ripple effects changing the IDs used on AT32 base platforms
 */
#if defined(PX4_CPU_UUID_CORRECT_CORRELATION)
# define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            2 /* Most significant digits change the least */
#else
/* Legacy incorrect ordering */
# define PX4_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
# define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
#endif

/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

//#define px4_savepanic(fileno, context, length)  progmem_dump_savepanic(fileno, context, length)//at32_bbsram_savepanic(fileno, context, length)

#define PX4_BUS_OFFSET       0                  /* STM buses are 1 based no adjustment needed */
#define px4_spibus_initialize(bus_num_1based)   at32_spibus_initialize(bus_num_1based)

#define px4_i2cbus_initialize(bus_num_1based)   at32_i2cbus_initialize(bus_num_1based)
#define px4_i2cbus_uninitialize(pdev)           at32_i2cbus_uninitialize(pdev)

#define px4_arch_configgpio(pinset)             at32_configgpio(pinset)
#define px4_arch_unconfiggpio(pinset)           at32_unconfiggpio(pinset)
#define px4_arch_gpioread(pinset)               at32_gpioread(pinset)
#define px4_arch_gpiowrite(pinset, value)       at32_gpiowrite(pinset, value)
#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  at32_gpiosetevent(pinset,r,f,e,fp,a)

#define PX4_MAKE_GPIO_INPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
#define PX4_MAKE_GPIO_EXTI(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_EXTI|GPIO_INPUT|GPIO_PULLUP))
#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_DRV_MODETATE|GPIO_OUTPUT_CLEAR))
#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_DRV_MODETATE|GPIO_OUTPUT_SET))

#define PX4_GPIO_PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_FLOAT|GPIO_DRV_MODETATE))

/* CAN bootloader usage */

#define TIMER_HRT_CYCLES_PER_US (AT32_HCLK_FREQUENCY/1000000)
#define TIMER_HRT_CYCLES_PER_MS (AT32_HCLK_FREQUENCY/1000)

/*  CAN_FiRx where (i=0..27|13, x=1, 2)
 *                      AT32_CAN1_FIR(i,x)
 * Using i = 2 does not requier there block
 * to be enabled nor FINIT in CAN_FMR to be set.
 * todo:Validate this claim on F2, F3
 */

#define crc_HiLOC       AT32_CAN1_FIR(2,1)
#define crc_LoLOC       AT32_CAN1_FIR(2,2)
#define signature_LOC   AT32_CAN1_FIR(3,1)
#define bus_speed_LOC   AT32_CAN1_FIR(3,2)
#define node_id_LOC     AT32_CAN1_FIR(4,1)

#if defined(CONFIG_ARMV7M_DCACHE)
#  define PX4_ARCH_DCACHE_ALIGNMENT ARMV7M_DCACHE_LINESIZE
#  define px4_cache_aligned_data() aligned_data(ARMV7M_DCACHE_LINESIZE)
#  define px4_cache_aligned_alloc(s) memalign(ARMV7M_DCACHE_LINESIZE,(s))
#else
#  define px4_cache_aligned_data()
#  define px4_cache_aligned_alloc malloc
#endif


__END_DECLS
