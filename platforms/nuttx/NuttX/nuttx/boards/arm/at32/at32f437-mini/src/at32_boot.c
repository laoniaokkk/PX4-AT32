/****************************************************************************
 * boards/arm/at32/at32f4discovery/src/at32_boot.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "nvic.h"
#include "itm.h"

#include "at32.h"
#include "at32f437-mini.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_boardinitialize
 *
 * Description:
 *   All at32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void at32_boardinitialize(void)
{
#if defined(CONFIG_AT32_SPI1) || defined(CONFIG_AT32_SPI2) || \
defined(CONFIG_AT32_SPI3) || defined(CONFIG_AT32_SPI4)
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function at32_spidev_initialize() has been brought into the link.
   */

  if (at32_spidev_initialize)
    {
      at32_spidev_initialize();
    }
#endif

#ifdef CONFIG_AT32_OTGFS
  /* Initialize USB if the 1) OTG FS controller is in the configuration and
   * 2) disabled, and 3) the weak function at32_usbinitialize() has been
   * brought into the build. Presumably either CONFIG_USBDEV or
   * CONFIG_USBHOST is also selected.
   */

  if (at32_usbinitialize)
    {
      at32_usbinitialize();
    }
#endif

#ifdef HAVE_NETMONITOR
  /* Configure board resources to support networking. */

  if (at32_netinitialize)
    {
      at32_netinitialize();
    }
#endif

#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  /* Perform board-specific initialization */

  at32_bringup();
}
#endif
