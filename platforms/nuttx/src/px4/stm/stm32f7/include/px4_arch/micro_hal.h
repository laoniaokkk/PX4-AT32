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


#include "../../../stm32_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_STM32F7
#include <chip.h>
#include <stm32_gpio.h>
#include <stm32_can.h>
#include <hardware/stm32_flash.h>
#include <arm_internal.h> //include up_systemreset() which is included on stm32.h
#if defined(CONFIG_STM32F7_BKPSRAM)
# include <stm32_bbsram.h>
# define PX4_BBSRAM_SIZE STM32F7_BBSRAM_SIZE
# define PX4_HF_GETDESC_IOCTL STM32F7_BBSRAM_GETDESC_IOCTL
# define HAS_BBSRAM CONFIG_STM32F7_BBSRAM
# define BBSRAM_FILE_COUNT CONFIG_STM32F7_BBSRAM_FILES
# define SAVE_CRASHDUMP CONFIG_STM32F7_SAVE_CRASHDUMP
#endif // CONFIG_STM32F7_BKPSRAM
#define PX4_FLASH_BASE  0x08000000
#define PX4_NUMBER_I2C_BUSES STM32F7_NI2C
#define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 18


int stm32_flash_lock(void);
int stm32_flash_unlock(void);
int stm32_flash_writeprotect(size_t page, bool enabled);

__END_DECLS

