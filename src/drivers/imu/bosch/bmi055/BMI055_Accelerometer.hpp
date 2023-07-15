/****************************************************************************
 *
 *   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

#include "BMI055.hpp"

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>

#include "Bosch_BMI055_Accelerometer_Registers.hpp"

namespace Bosch::BMI055::Accelerometer
{

class BMI055_Accelerometer : public BMI055
{
public:
	BMI055_Accelerometer(const I2CSPIDriverConfig &config);
	~BMI055_Accelerometer() override;

	void RunImpl() override;
	void print_status() override;

private:
	void exit_and_cleanup() override;

	// Sensor Configuration
	static constexpr uint32_t RATE{2000}; // 2000 Hz
	static constexpr float FIFO_SAMPLE_DT{1e6f / RATE};

	static constexpr int32_t FIFO_MAX_SAMPLES{math::min(FIFO::SIZE / sizeof(FIFO::DATA), sizeof(sensor_accel_fifo_s::x) / sizeof(sensor_accel_fifo_s::x[0]))};

	// Transfer data
	struct FIFOTransferBuffer {
		uint8_t cmd{static_cast<uint8_t>(Register::FIFO_DATA) | DIR_READ};
		FIFO::DATA f[FIFO_MAX_SAMPLES] {};
	};
	// ensure no struct padding
	static_assert(sizeof(FIFOTransferBuffer) == (1 + FIFO_MAX_SAMPLES *sizeof(FIFO::DATA)));

	struct register_config_t {
		Register reg;
		uint8_t set_bits{0};
		uint8_t clear_bits{0};
	};

	int probe() override;

	bool Configure();
	void ConfigureAccel();
	void ConfigureSampleRate(int sample_rate = 0);
	void ConfigureFIFOWatermark(uint8_t samples);

	static int DataReadyInterruptCallback(int irq, void *context, void *arg);
	void DataReady();
	bool DataReadyInterruptConfigure();
	bool DataReadyInterruptDisable();

	bool RegisterCheck(const register_config_t &reg_cfg);

	uint8_t RegisterRead(Register reg);
	void RegisterWrite(Register reg, uint8_t value);
	void RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits);

	bool FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples);
	void FIFOReset();

	void UpdateTemperature();

	PX4Accelerometer _px4_accel;

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME"_accel: bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME"_accel: bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME"_accel: FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME"_accel: FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME"_accel: FIFO reset")};
	perf_counter_t _drdy_missed_perf{nullptr};

	uint8_t _fifo_samples{static_cast<uint8_t>(_fifo_empty_interval_us / (1000000 / RATE))};

	uint8_t _checked_register{0};
	static constexpr uint8_t size_register_cfg{7};
	register_config_t _register_cfg[size_register_cfg] {
		// Register                    | Set bits, Clear bits
		{ Register::PMU_RANGE,         PMU_RANGE_BIT::range_16g_set, PMU_RANGE_BIT::range_16g_clear},
		{ Register::ACCD_HBW,          ACCD_HBW_BIT::data_high_bw, 0},
		{ Register::INT_EN_1,          INT_EN_1_BIT::int_fwm_en, 0},
		{ Register::INT_MAP_1,         INT_MAP_1_BIT::int1_fwm, 0},
		{ Register::INT_OUT_CTRL,      0, INT_OUT_CTRL_BIT::int1_od | INT_OUT_CTRL_BIT::int1_lvl},
		{ Register::FIFO_CONFIG_0,     0, 0 }, // fifo_water_mark_level_trigger_retain<5:0>
		{ Register::FIFO_CONFIG_1,     FIFO_CONFIG_1_BIT::fifo_mode, 0},
	};
};

} // namespace Bosch::BMI055::Accelerometer
