/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include "ICM20608G.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20608G::ICM20608G(const I2CSPIDriverConfig &config) :
	SPI(config),
	I2CSPIDriver(config),
	_drdy_gpio(config.drdy_gpio),
	_px4_accel(get_device_id(), config.rotation),
	_px4_gyro(get_device_id(), config.rotation)
{
	if (_drdy_gpio != 0) {
		_drdy_missed_perf = perf_alloc(PC_COUNT, MODULE_NAME": DRDY missed");
	}

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM20608G::~ICM20608G()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);
}

int ICM20608G::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM20608G::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20608G::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM20608G::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);
}

bool ICM20608G::StoreCheckedRegisterValue(Register reg)
{
	// 3 retries
	for (int i = 0; i < 3; i++) {
		uint8_t read1 = RegisterRead(reg);
		uint8_t read2 = RegisterRead(reg);

		if (read1 == read2) {
			for (auto &r : _register_cfg) {
				if (r.reg == reg) {
					r.set_bits = read1;
					r.clear_bits = ~read1;
					return true;
				}
			}

		} else {
			PX4_ERR("0x%02hhX read 1 != read 2 (0x%02hhX != 0x%02hhX)", static_cast<uint8_t>(reg), read1, read2);
		}
	}

	return false;
}

int ICM20608G::probe()
{
	const uint8_t whoami = RegisterRead(Register::WHO_AM_I);

	if (whoami != WHOAMI) {
		DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	return PX4_OK;
}

void ICM20608G::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		//  Document Number: RM-000030 Page 5 of 23
		if ((RegisterRead(Register::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::PWR_MGMT_1) == 0x40)) {

			// offset registers (factory calibration) should not change during normal operation
			StoreCheckedRegisterValue(Register::XA_OFFSET_H);
			StoreCheckedRegisterValue(Register::XA_OFFSET_L);
			StoreCheckedRegisterValue(Register::YA_OFFSET_H);
			StoreCheckedRegisterValue(Register::YA_OFFSET_L);
			StoreCheckedRegisterValue(Register::ZA_OFFSET_H);
			StoreCheckedRegisterValue(Register::ZA_OFFSET_L);

			// Wakeup and reset digital signal path
			RegisterWrite(Register::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
			RegisterWrite(Register::SIGNAL_PATH_RESET, SIGNAL_PATH_RESET_BIT::ACCEL_RST | SIGNAL_PATH_RESET_BIT::TEMP_RST);
			RegisterWrite(Register::USER_CTRL, USER_CTRL_BIT::SIG_COND_RST | USER_CTRL_BIT::I2C_IF_DIS);

			// if reset succeeded then configure
			_state = STATE::CONFIGURE;
			ScheduleDelayed(100_ms);

		} else {
			// RESET not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Reset failed, retrying");
				_state = STATE::RESET;
				ScheduleDelayed(100_ms);

			} else {
				PX4_DEBUG("Reset not complete, check again in 10 ms");
				ScheduleDelayed(10_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading from FIFO
			_state = STATE::FIFO_READ;

			if (DataReadyInterruptConfigure()) {
				_data_ready_interrupt_enabled = true;

				// backup schedule as a watchdog timeout
				ScheduleDelayed(100_ms);

			} else {
				_data_ready_interrupt_enabled = false;
				ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);
			}

			FIFOReset();

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 1000_ms) {
				PX4_DEBUG("Configure failed, resetting");
				_state = STATE::RESET;

			} else {
				PX4_DEBUG("Configure failed, retrying");
			}

			ScheduleDelayed(100_ms);
		}

		break;

	case STATE::FIFO_READ: {
			hrt_abstime timestamp_sample = now;

			if (_data_ready_interrupt_enabled) {
				// scheduled from interrupt if _drdy_timestamp_sample was set as expected
				const hrt_abstime drdy_timestamp_sample = _drdy_timestamp_sample.fetch_and(0);

				if ((now - drdy_timestamp_sample) < _fifo_empty_interval_us) {
					timestamp_sample = drdy_timestamp_sample;

				} else {
					perf_count(_drdy_missed_perf);
				}

				// push backup schedule back
				ScheduleDelayed(_fifo_empty_interval_us * 2);
			}

			// always check current FIFO count
			bool success = false;
			const uint16_t fifo_count = FIFOReadCount();

			if (fifo_count >= FIFO::SIZE) {
				FIFOReset();
				perf_count(_fifo_overflow_perf);

			} else if (fifo_count == 0) {
				perf_count(_fifo_empty_perf);

			} else {
				// FIFO count (size in bytes) should be a multiple of the FIFO::DATA structure
				uint8_t samples = fifo_count / sizeof(FIFO::DATA);

				// tolerate minor jitter, leave sample to next iteration if behind by only 1
				if (samples == _fifo_gyro_samples + 1) {
					timestamp_sample -= static_cast<int>(FIFO_SAMPLE_DT);
					samples--;
				}

				if (samples > FIFO_MAX_SAMPLES) {
					// not technically an overflow, but more samples than we expected or can publish
					FIFOReset();
					perf_count(_fifo_overflow_perf);

				} else if (samples >= SAMPLES_PER_TRANSFER) {
					if (FIFORead(timestamp_sample, samples)) {
						success = true;

						if (_failure_count > 0) {
							_failure_count--;
						}
					}
				}
			}

			if (!success) {
				_failure_count++;

				// full reset if things are failing consistently
				if (_failure_count > 10) {
					Reset();
					return;
				}
			}

			if (!success || hrt_elapsed_time(&_last_config_check_timestamp) > 100_ms) {
				// check configuration registers periodically or immediately following any failure
				if (RegisterCheck(_register_cfg[_checked_register])) {
					_last_config_check_timestamp = now;
					_checked_register = (_checked_register + 1) % size_register_cfg;

				} else {
					// register check failed, force reset
					perf_count(_bad_register_perf);
					Reset();
				}

			} else {
				// periodically update temperature (~1 Hz)
				if (hrt_elapsed_time(&_temperature_update_timestamp) >= 1_s) {
					UpdateTemperature();
					_temperature_update_timestamp = now;
				}
			}
		}

		break;
	}
}

void ICM20608G::ConfigureAccel()
{
	const uint8_t ACCEL_FS_SEL = RegisterRead(Register::ACCEL_CONFIG) & (Bit4 | Bit3); // [4:3] ACCEL_FS_SEL[1:0]

	switch (ACCEL_FS_SEL) {
	case ACCEL_FS_SEL_2G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 16384.f);
		_px4_accel.set_range(2.f * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_4G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);
		_px4_accel.set_range(4.f * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_8G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 4096.f);
		_px4_accel.set_range(8.f * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_16G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
		_px4_accel.set_range(16.f * CONSTANTS_ONE_G);
		break;
	}
}

void ICM20608G::ConfigureGyro()
{
	const uint8_t FS_SEL = RegisterRead(Register::GYRO_CONFIG) & (Bit4 | Bit3); // [4:3] FS_SEL[1:0]

	float range_dps = 0.f;

	switch (FS_SEL) {
	case FS_SEL_250_DPS:
		range_dps = 250.f;
		break;

	case FS_SEL_500_DPS:
		range_dps = 500.f;
		break;

	case FS_SEL_1000_DPS:
		range_dps = 1000.f;
		break;

	case FS_SEL_2000_DPS:
		range_dps = 2000.f;
		break;
	}

	_px4_gyro.set_scale(math::radians(range_dps / 32768.f));
	_px4_gyro.set_range(math::radians(range_dps));
}

void ICM20608G::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

bool ICM20608G::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	ConfigureAccel();
	ConfigureGyro();

	return success;
}

int ICM20608G::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM20608G *>(arg)->DataReady();
	return 0;
}

void ICM20608G::DataReady()
{
	// at least the required number of samples in the FIFO
	if (++_drdy_count >= _fifo_gyro_samples) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count -= _fifo_gyro_samples;
		ScheduleNow();
	}
}

bool ICM20608G::DataReadyInterruptConfigure()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
}

bool ICM20608G::DataReadyInterruptDisable()
{
	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
}

bool ICM20608G::RegisterCheck(const register_config_t &reg_cfg)
{
	bool success = true;

	const uint8_t reg_value = RegisterRead(reg_cfg.reg);

	if (reg_cfg.set_bits && ((reg_value & reg_cfg.set_bits) != reg_cfg.set_bits)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not set)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.set_bits);
		success = false;
	}

	if (reg_cfg.clear_bits && ((reg_value & reg_cfg.clear_bits) != 0)) {
		PX4_DEBUG("0x%02hhX: 0x%02hhX (0x%02hhX not cleared)", (uint8_t)reg_cfg.reg, reg_value, reg_cfg.clear_bits);
		success = false;
	}

	return success;
}

uint8_t ICM20608G::RegisterRead(Register reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

void ICM20608G::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	transfer(cmd, cmd, sizeof(cmd));
}

void ICM20608G::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM20608G::FIFOReadCount()
{
	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::FIFO_COUNTH) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool ICM20608G::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 1, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}


	ProcessGyro(timestamp_sample, buffer.f, samples);
	return ProcessAccel(timestamp_sample, buffer.f, samples);
}

void ICM20608G::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_EN: disable FIFO
	RegisterWrite(Register::FIFO_EN, 0);

	// USER_CTRL: reset FIFO
	RegisterSetAndClearBits(Register::USER_CTRL, USER_CTRL_BIT::FIFO_RST, USER_CTRL_BIT::FIFO_EN);

	// reset while FIFO is disabled
	_drdy_count = 0;
	_drdy_timestamp_sample.store(0);

	// FIFO_EN: enable both gyro and accel
	// USER_CTRL: re-enable FIFO
	for (const auto &r : _register_cfg) {
		if ((r.reg == Register::FIFO_EN) || (r.reg == Register::USER_CTRL)) {
			RegisterSetAndClearBits(r.reg, r.set_bits, r.clear_bits);
		}
	}
}

static bool fifo_accel_equal(const FIFO::DATA &f0, const FIFO::DATA &f1)
{
	return (memcmp(&f0.ACCEL_XOUT_H, &f1.ACCEL_XOUT_H, 6) == 0);
}

bool ICM20608G::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;

	bool bad_data = false;

	// accel data is doubled in FIFO, but might be shifted
	int accel_first_sample = 1;

	if (samples >= 4) {
		if (fifo_accel_equal(fifo[0], fifo[1]) && fifo_accel_equal(fifo[2], fifo[3])) {
			// [A0, A1, A2, A3]
			//  A0==A1, A2==A3
			accel_first_sample = 1;

		} else if (fifo_accel_equal(fifo[1], fifo[2])) {
			// [A0, A1, A2, A3]
			//  A0, A1==A2, A3
			accel_first_sample = 0;

		} else {
			// no matching accel samples is an error
			bad_data = true;
			perf_count(_bad_transfer_perf);
		}
	}

	for (int i = accel_first_sample; i < samples; i = i + SAMPLES_PER_TRANSFER) {
		int16_t accel_x = combine(fifo[i].ACCEL_XOUT_H, fifo[i].ACCEL_XOUT_L);
		int16_t accel_y = combine(fifo[i].ACCEL_YOUT_H, fifo[i].ACCEL_YOUT_L);
		int16_t accel_z = combine(fifo[i].ACCEL_ZOUT_H, fifo[i].ACCEL_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[accel.samples] = accel_x;
		accel.y[accel.samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[accel.samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
		accel.samples++;
	}

	_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	if (accel.samples > 0) {
		_px4_accel.updateFIFO(accel);
	}

	return !bad_data;
}

void ICM20608G::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = FIFO_SAMPLE_DT;

	for (int i = 0; i < samples; i++) {
		const int16_t gyro_x = combine(fifo[i].GYRO_XOUT_H, fifo[i].GYRO_XOUT_L);
		const int16_t gyro_y = combine(fifo[i].GYRO_YOUT_H, fifo[i].GYRO_YOUT_L);
		const int16_t gyro_z = combine(fifo[i].GYRO_ZOUT_H, fifo[i].GYRO_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
				  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));

	_px4_gyro.updateFIFO(gyro);
}

void ICM20608G::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::TEMP_OUT_H) | DIR_READ;

	if (transfer(temperature_buf, temperature_buf, sizeof(temperature_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return;
	}

	const int16_t TEMP_OUT = combine(temperature_buf[1], temperature_buf[2]);
	const float TEMP_degC = (TEMP_OUT / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)) {
		_px4_accel.set_temperature(TEMP_degC);
		_px4_gyro.set_temperature(TEMP_degC);
	}
}
