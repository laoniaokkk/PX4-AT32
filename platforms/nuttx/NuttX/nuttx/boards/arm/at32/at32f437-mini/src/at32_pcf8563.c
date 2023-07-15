/****************************************************************************
 * boards/arm/at32/at32f437-mini/src/at32_pcf85263.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/nxffs.h>

#include "at32_i2c.h"
#include "at32f437-mini.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at32_pcf85263_init
 *
 * Description:
 *   Initialize and configure the pcf85263 serial rtc
 *
 ****************************************************************************/

int at32_pcf85263_init(void)
{
    int ret = 0;
    struct i2c_master_s *i2c;

    i2c = at32_i2cbus_initialize(PCF8563_BUS);
    if (i2c == NULL)
    {
        ferr("ERROR: Failed to initialize I2C%d\n", AT24_I2C_BUS);
        return -ENODEV;
    }
    else
    {
        /* Use the I2C interface to initialize the PCF2863 timer */

        ret = pcf85263_rtc_initialize(i2c);
        if (ret < 0)
        {
            ferr("ERROR: pcf85263_rtc_initialize() failed: %d\n",ret);
        }
        else
        {
            /* Synchronize the system time to the RTC time */

            clock_synchronize(NULL);
        }
    }

}