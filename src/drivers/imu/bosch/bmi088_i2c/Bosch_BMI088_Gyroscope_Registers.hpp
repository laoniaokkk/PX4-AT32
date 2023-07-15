/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

namespace Bosch::BMI088::Gyroscope
{

// TODO: move to a central header
static constexpr uint8_t Bit0 = (1 << 0);
static constexpr uint8_t Bit1 = (1 << 1);
static constexpr uint8_t Bit2 = (1 << 2);
static constexpr uint8_t Bit3 = (1 << 3);
static constexpr uint8_t Bit4 = (1 << 4);
static constexpr uint8_t Bit5 = (1 << 5);
static constexpr uint8_t Bit6 = (1 << 6);
static constexpr uint8_t Bit7 = (1 << 7);

static constexpr uint32_t SPI_SPEED = 10 * 1000 * 1000; // 10MHz SPI serial interface

static constexpr uint32_t I2C_400_SPEED = 400 * 1000; // 400kHz I2C interface
static constexpr uint32_t I2C_200_SPEED = 200 * 1000; // 200kHz I2C interface
static constexpr uint32_t I2C_100_SPEED = 100 * 1000; // 100kHz I2C interface

static constexpr uint8_t DIR_READ = 0x80;

static constexpr uint8_t ID = 0x0F;

static constexpr uint8_t GYRO_I2C_ADDR_PRIMARY = 0x68;
static constexpr uint8_t GYRO_I2C_ADDR_SECONDARY = 0x69;

enum class Register : uint8_t {
	GYRO_CHIP_ID      = 0x00,

	FIFO_STATUS       = 0x0E,
	GYRO_RANGE        = 0x0F,
	GYRO_BANDWIDTH    = 0x10,

	GYRO_SOFTRESET    = 0x14,
	GYRO_INT_CTRL     = 0x15,
	INT3_INT4_IO_CONF = 0x16,
	INT3_INT4_IO_MAP  = 0x18,

	FIFO_WM_ENABLE    = 0x1E,

	FIFO_CONFIG_0     = 0x3D,
	FIFO_CONFIG_1     = 0x3E,
	FIFO_DATA         = 0x3F,
	SELF_TEST         = 0x3C,
	READ_GYRO         = 0x02,
};

// FIFO_STATUS
enum FIFO_STATUS_BIT : uint8_t {
	Fifo_overrun       = Bit7,
	Fifo_frame_counter = Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0,
};

// GYRO_RANGE
enum GYRO_RANGE_BIT : uint8_t {
	gyro_range_2000_dps = 0x00, // ±2000
	gyro_range_1000_dps = 0x01, // ±1000
	gyro_range_500_dps  = 0x02, // ±500
	gyro_range_250_dps  = 0x04, // ±250
	gyro_range_125_dps  = 0x05, // ±125
};

// GYRO_BANDWIDTH
enum GYRO_BANDWIDTH_BIT : uint8_t {
	gyro_bw_100_Hz = Bit2 | Bit1 | Bit0,
	gyro_bw_200_Hz = Bit4,
	gyro_bw_2000_Hz = 0x00,
};

// GYRO_INT_CTRL
enum GYRO_INT_CTRL_BIT : uint8_t {
	data_en = Bit7,
	fifo_en = Bit6,
};

// INT3_INT4_IO_CONF
enum INT3_INT4_IO_CONF_BIT : uint8_t {
	Int3_od  = Bit1, // ‘0’ Push-pull
	Int3_lvl = Bit0, // ‘0’ Active low
};

// INT3_INT4_IO_MAP
enum INT3_INT4_IO_MAP_BIT : uint8_t {
	Int4_data = Bit7,
	Int4_fifo = Bit5,
	Int3_fifo = Bit2,
	Int3_data = Bit0,
};

// FIFO_WM_ENABLE
enum FIFO_WM_ENABLE_BIT : uint8_t {
	fifo_wm_enable  = Bit7 | Bit3,
	fifo_wm_disable = Bit3,
};

// FIFO_CONFIG_1
enum FIFO_CONFIG_1_BIT : uint8_t {
	FIFO_MODE = Bit6,
	FIFO_MODE_STREAM = Bit7,
};


namespace FIFO
{
struct DATA {
	uint8_t RATE_X_LSB;
	uint8_t RATE_X_MSB;
	uint8_t RATE_Y_LSB;
	uint8_t RATE_Y_MSB;
	uint8_t RATE_Z_LSB;
	uint8_t RATE_Z_MSB;
};
static_assert(sizeof(DATA) == 6);

// 100 frames of data in FIFO mode
static constexpr size_t SIZE = sizeof(DATA) * 100;

} // namespace FIFO
} // namespace Bosch::BMI088::Gyroscope
