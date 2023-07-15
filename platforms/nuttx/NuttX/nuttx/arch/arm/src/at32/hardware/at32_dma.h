/****************************************************************************
 * arch/arm/src/at32/hardware/at32_dma.h
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

#ifndef __ARCH_ARM_SRC_AT32_HARDWARE_AT32_DMA_H
#define __ARCH_ARM_SRC_AT32_HARDWARE_AT32_DMA_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"

/* Include the correct DMA register definitions for
 * selected AT32 DMA IP core:
 *   - AT32 DMA IP version 1 - F43X
 */

#if defined(CONFIG_AT32_HAVE_IP_DMA_V1) && defined(CONFIG_AT32_HAVE_IP_DMA_V2)
#  error Only one AT32 DMA IP version must be selected
#endif

#if defined(CONFIG_AT32_HAVE_IP_DMA_V1)
#  include "at32_dma_v1.h"
#elif defined(CONFIG_AT32_HAVE_IP_DMA_V2)
#  include "at32_dma_v2.h"
#else
#  error "AT32 DMA IP version not specified"
#endif

#endif /* __ARCH_ARM_SRC_AT32_HARDWARE_AT32_DMA_H */
