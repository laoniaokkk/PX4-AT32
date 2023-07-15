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

#include "ICM20948.hpp"

#include "AKM_AK09916_registers.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20948::ICM20948(const I2CSPIDriverConfig &config) :
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

	bool enable_magnetometer = config.custom1 == 1;

	if (enable_magnetometer) {
		_slave_ak09916_magnetometer = new AKM_AK09916::ICM20948_AK09916(*this, config.rotation);

		if (_slave_ak09916_magnetometer) {
			for (auto &r : _register_bank3_cfg) {
				if (r.reg == Register::BANK_3::I2C_SLV4_CTRL) {
					r.set_bits = I2C_SLV4_CTRL_BIT::I2C_MST_DLY;

				} else if (r.reg == Register::BANK_3::I2C_MST_CTRL) {
					r.set_bits = I2C_MST_CTRL_BIT::I2C_MST_P_NSR | I2C_MST_CTRL_BIT::I2C_MST_CLK_400_kHz;

				} else if (r.reg == Register::BANK_3::I2C_MST_DELAY_CTRL) {
					r.set_bits = I2C_MST_DELAY_CTRL_BIT::I2C_SLVX_DLY_EN;
				}
			}
		}
	}
}

ICM20948::~ICM20948()
{
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_missed_perf);

	delete _slave_ak09916_magnetometer;
}

int ICM20948::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("SPI::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool ICM20948::Reset()
{
	_state = STATE::RESET;
	DataReadyInterruptDisable();
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20948::exit_and_cleanup()
{
	DataReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM20948::print_status()
{
	I2CSPIDriverBase::print_status();

	PX4_INFO("FIFO empty interval: %d us (%.1f Hz)", _fifo_empty_interval_us, 1e6 / _fifo_empty_interval_us);

	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_missed_perf);


	if (_slave_ak09916_magnetometer) {
		_slave_ak09916_magnetometer->PrintInfo();
	}
}

int ICM20948::probe()
{
	for (int i = 0; i < 3; i++) {
		uint8_t whoami = RegisterRead(Register::BANK_0::WHO_AM_I);

		if (whoami == WHOAMI) {
			return PX4_OK;

		} else {
			DEVICE_DEBUG("unexpected WHO_AM_I 0x%02x", whoami);

			uint8_t reg_bank_sel = RegisterRead(Register::BANK_0::REG_BANK_SEL);
			int bank = reg_bank_sel >> 4;

			if (bank >= 1 && bank <= 3) {
				DEVICE_DEBUG("incorrect register bank for WHO_AM_I REG_BANK_SEL:0x%02x, bank:%d", reg_bank_sel, bank);
				// force bank selection and retry
				SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0, true);
			}
		}
	}

	return PX4_ERROR;
}

void ICM20948::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		// PWR_MGMT_1: Device Reset
		RegisterWrite(Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::DEVICE_RESET);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET:

		// The reset value is 0x00 for all registers other than the registers below
		if ((RegisterRead(Register::BANK_0::WHO_AM_I) == WHOAMI)
		    && (RegisterRead(Register::BANK_0::PWR_MGMT_1) == 0x41)) {

			// Wakeup and reset
			RegisterWrite(Register::BANK_0::PWR_MGMT_1, PWR_MGMT_1_BIT::CLKSEL_0);
			RegisterWrite(Register::BANK_0::USER_CTRL,
				      USER_CTRL_BIT::I2C_MST_EN | USER_CTRL_BIT::I2C_IF_DIS | USER_CTRL_BIT::SRAM_RST | USER_CTRL_BIT::I2C_MST_RST);

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
				PX4_DEBUG("Reset not complete, check again in 100 ms");
				ScheduleDelayed(100_ms);
			}
		}

		break;

	case STATE::CONFIGURE:
		if (Configure()) {

			// start AK09916 magnetometer (I2C aux)
			if (_slave_ak09916_magnetometer) {
				_slave_ak09916_magnetometer->Reset();
			}

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

				if (samples > _fifo_gyro_samples) {
					// grab desired number of samples, but reschedule next cycle sooner
					int extra_samples = samples - _fifo_gyro_samples;
					samples = _fifo_gyro_samples;

					if (_fifo_gyro_samples > extra_samples) {
						// reschedule to run when a total of _fifo_gyro_samples should be available in the FIFO
						const uint32_t reschedule_delay_us = (_fifo_gyro_samples - extra_samples) * static_cast<int>(FIFO_SAMPLE_DT);
						ScheduleOnInterval(_fifo_empty_interval_us, reschedule_delay_us);

					} else {
						// otherwise reschedule to run immediately
						ScheduleOnInterval(_fifo_empty_interval_us);
					}

				} else if (samples < _fifo_gyro_samples) {
					// reschedule next cycle to catch the desired number of samples
					ScheduleOnInterval(_fifo_empty_interval_us, (_fifo_gyro_samples - samples) * static_cast<int>(FIFO_SAMPLE_DT));
				}

				if (samples == _fifo_gyro_samples) {
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
				if (RegisterCheck(_register_bank0_cfg[_checked_register_bank0])
				    && RegisterCheck(_register_bank2_cfg[_checked_register_bank2])
				    && RegisterCheck(_register_bank3_cfg[_checked_register_bank3])
				   ) {
					_last_config_check_timestamp = now;
					_checked_register_bank0 = (_checked_register_bank0 + 1) % size_register_bank0_cfg;
					_checked_register_bank2 = (_checked_register_bank2 + 1) % size_register_bank2_cfg;
					_checked_register_bank3 = (_checked_register_bank3 + 1) % size_register_bank3_cfg;

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

void ICM20948::ConfigureAccel()
{
	const uint8_t ACCEL_FS_SEL = RegisterRead(Register::BANK_2::ACCEL_CONFIG) & (Bit2 | Bit1); // 2:1 ACCEL_FS_SEL[1:0]

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

void ICM20948::ConfigureGyro()
{
	const uint8_t GYRO_FS_SEL = RegisterRead(Register::BANK_2::GYRO_CONFIG_1) & (Bit2 | Bit1); // 2:1 GYRO_FS_SEL[1:0]

	float range_dps = 0.f;

	switch (GYRO_FS_SEL) {
	case GYRO_FS_SEL_250_DPS:
		range_dps = 250.f;
		break;

	case GYRO_FS_SEL_500_DPS:
		range_dps = 500.f;
		break;

	case GYRO_FS_SEL_1000_DPS:
		range_dps = 1000.f;
		break;

	case GYRO_FS_SEL_2000_DPS:
		range_dps = 2000.f;
		break;
	}

	_px4_gyro.set_scale(math::radians(range_dps / 32768.f));
	_px4_gyro.set_range(math::radians(range_dps));
}

void ICM20948::ConfigureSampleRate(int sample_rate)
{
	// round down to nearest FIFO sample dt * SAMPLES_PER_TRANSFER
	const float min_interval = FIFO_SAMPLE_DT * SAMPLES_PER_TRANSFER;
	_fifo_empty_interval_us = math::max(roundf((1e6f / (float)sample_rate) / min_interval) * min_interval, min_interval);

	_fifo_gyro_samples = roundf(math::min((float)_fifo_empty_interval_us / (1e6f / GYRO_RATE), (float)FIFO_MAX_SAMPLES));

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1e6f / GYRO_RATE);
}

void ICM20948::SelectRegisterBank(enum REG_BANK_SEL_BIT bank, bool force)
{
	if (bank != _last_register_bank || force) {
		// select BANK_0
		uint8_t cmd_bank_sel[2] {};
		cmd_bank_sel[0] = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
		cmd_bank_sel[1] = bank;
		transfer(cmd_bank_sel, cmd_bank_sel, sizeof(cmd_bank_sel));

		_last_register_bank = bank;
	}
}

bool ICM20948::Configure()
{
	// first set and clear all configured register bits
	for (const auto &reg_cfg : _register_bank0_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	for (const auto &reg_cfg : _register_bank3_cfg) {
		RegisterSetAndClearBits(reg_cfg.reg, reg_cfg.set_bits, reg_cfg.clear_bits);
	}

	// now check that all are configured
	bool success = true;

	for (const auto &reg_cfg : _register_bank0_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank2_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	for (const auto &reg_cfg : _register_bank3_cfg) {
		if (!RegisterCheck(reg_cfg)) {
			success = false;
		}
	}

	ConfigureAccel();
	ConfigureGyro();

	return success;
}

int ICM20948::DataReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM20948 *>(arg)->DataReady();
	return 0;
}

void ICM20948::DataReady()
{
	// at least the required number of samples in the FIFO
	if (++_drdy_count >= _fifo_gyro_samples) {
		_drdy_timestamp_sample.store(hrt_absolute_time());
		_drdy_count -= _fifo_gyro_samples;
		ScheduleNow();
	}
}

bool ICM20948::DataReadyInterruptConfigure()
{
	// TODO: enable data ready interrupt
	return false;
#if 0

	if (_drdy_gpio == 0) {
		return false;
	}

	// Setup data ready on falling edge
	return px4_arch_gpiosetevent(_drdy_gpio, false, true, true, &DataReadyInterruptCallback, this) == 0;
#endif
}

bool ICM20948::DataReadyInterruptDisable()
{
	// TODO: enable data ready interrupt
	return false;
#if 0

	if (_drdy_gpio == 0) {
		return false;
	}

	return px4_arch_gpiosetevent(_drdy_gpio, false, false, false, nullptr, nullptr) == 0;
#endif
}

template <typename T>
bool ICM20948::RegisterCheck(const T &reg_cfg)
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

template <typename T>
uint8_t ICM20948::RegisterRead(T reg)
{
	uint8_t cmd[2] {};
	cmd[0] = static_cast<uint8_t>(reg) | DIR_READ;
	SelectRegisterBank(reg);
	transfer(cmd, cmd, sizeof(cmd));
	return cmd[1];
}

template <typename T>
void ICM20948::RegisterWrite(T reg, uint8_t value)
{
	uint8_t cmd[2] { (uint8_t)reg, value };
	SelectRegisterBank(reg);
	transfer(cmd, cmd, sizeof(cmd));
}

template <typename T>
void ICM20948::RegisterSetAndClearBits(T reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);

	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}

uint16_t ICM20948::FIFOReadCount()
{
	SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	// read FIFO count
	uint8_t fifo_count_buf[3] {};
	fifo_count_buf[0] = static_cast<uint8_t>(Register::BANK_0::FIFO_COUNTH) | DIR_READ;

	if (transfer(fifo_count_buf, fifo_count_buf, sizeof(fifo_count_buf)) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return 0;
	}

	return combine(fifo_count_buf[1], fifo_count_buf[2]);
}

bool ICM20948::FIFORead(const hrt_abstime &timestamp_sample, uint8_t samples)
{
	SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

	FIFOTransferBuffer buffer{};
	const size_t transfer_size = math::min(samples * sizeof(FIFO::DATA) + 3, FIFO::SIZE);

	if (transfer((uint8_t *)&buffer, (uint8_t *)&buffer, transfer_size) != PX4_OK) {
		perf_count(_bad_transfer_perf);
		return false;
	}


	const uint16_t fifo_count_bytes = combine(buffer.FIFO_COUNTH, buffer.FIFO_COUNTL);

	if (fifo_count_bytes >= FIFO::SIZE) {
		perf_count(_fifo_overflow_perf);
		FIFOReset();
		return false;
	}

	const uint8_t fifo_count_samples = fifo_count_bytes / sizeof(FIFO::DATA);

	if (fifo_count_samples == 0) {
		perf_count(_fifo_empty_perf);
		return false;
	}

	const uint16_t valid_samples = math::min(samples, fifo_count_samples);

	if (valid_samples > 0) {
		ProcessGyro(timestamp_sample, buffer.f, valid_samples);

		if (ProcessAccel(timestamp_sample, buffer.f, valid_samples)) {
			return true;
		}
	}

	return false;
}

void ICM20948::FIFOReset()
{
	perf_count(_fifo_reset_perf);

	// FIFO_RST: reset FIFO
	RegisterSetBits(Register::BANK_0::FIFO_RST, FIFO_RST_BIT::FIFO_RESET);
	RegisterClearBits(Register::BANK_0::FIFO_RST, FIFO_RST_BIT::FIFO_RESET);

	// reset while FIFO is disabled
	_drdy_count = 0;
	_drdy_timestamp_sample.store(0);
}

static bool fifo_accel_equal(const FIFO::DATA &f0, const FIFO::DATA &f1)
{
	return (memcmp(&f0.ACCEL_XOUT_H, &f1.ACCEL_XOUT_H, 6) == 0);
}

bool ICM20948::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
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

void ICM20948::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
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

void ICM20948::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	temperature_buf[0] = static_cast<uint8_t>(Register::BANK_0::TEMP_OUT_H) | DIR_READ;
	SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

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

void ICM20948::I2CSlaveRegisterWrite(uint8_t slave_i2c_addr, uint8_t reg, uint8_t val)
{
	RegisterWrite(Register::BANK_3::I2C_SLV0_ADDR, slave_i2c_addr);
	RegisterWrite(Register::BANK_3::I2C_SLV0_REG, reg);
	RegisterWrite(Register::BANK_3::I2C_SLV0_DO, val);
}

void ICM20948::I2CSlaveExternalSensorDataEnable(uint8_t slave_i2c_addr, uint8_t reg, uint8_t size)
{
	RegisterWrite(Register::BANK_3::I2C_SLV0_ADDR, slave_i2c_addr | I2C_SLV0_ADDR_BIT::I2C_SLV0_RNW);
	RegisterWrite(Register::BANK_3::I2C_SLV0_REG, reg);
	RegisterWrite(Register::BANK_3::I2C_SLV0_CTRL, size | I2C_SLV0_CTRL_BIT::I2C_SLV0_EN);
}

bool ICM20948::I2CSlaveExternalSensorDataRead(uint8_t *buffer, uint8_t length)
{
	bool ret = false;

	if (buffer != nullptr && length <= 24) {
		// max EXT_SENS_DATA 24 bytes
		uint8_t transfer_buffer[24 + 1] {};
		transfer_buffer[0] = static_cast<uint8_t>(Register::BANK_0::EXT_SLV_SENS_DATA_00) | DIR_READ;
		SelectRegisterBank(REG_BANK_SEL_BIT::USER_BANK_0);

		if (transfer(transfer_buffer, transfer_buffer, length + 1) == PX4_OK) {
			ret = true;
		}

		// copy data after cmd back to return buffer
		memcpy(buffer, &transfer_buffer[1], length);
	}

	return ret;
}
