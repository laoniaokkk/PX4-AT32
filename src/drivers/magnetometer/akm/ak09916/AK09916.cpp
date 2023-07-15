/****************************************************************************
 *
 *   Copyright (c) 2019-2022 PX4 Development Team. All rights reserved.
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

#include "AK09916.hpp"

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

AK09916::AK09916(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_mag(get_device_id(), config.rotation)
{
}

AK09916::~AK09916()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_magnetic_sensor_overflow_perf);
}

int AK09916::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool AK09916::Reset()
{
	_state = STATE::RESET;
	ScheduleClear();
	ScheduleNow();
	return true;
}

void AK09916::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("Variant: %s", _is_ak09918 ? "AK09918" : "AK09916");

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_magnetic_sensor_overflow_perf);
}

int AK09916::probe()
{
	// 3 retries
	for (int i = 0; i < 3; i++) {

		const uint8_t WIA1 = RegisterRead(Register::WIA1);
		const uint8_t WIA2 = RegisterRead(Register::WIA2);

		if ((WIA1 == Company_ID) && (WIA2 == Device_ID)) {
			_is_ak09918 = false;
			return PX4_OK;
		}

		if ((WIA1 == Company_ID) && (WIA2 == Device_ID_AK09918)) {
			_is_ak09918 = true;
			return PX4_OK;
		}

		if (WIA1 != Company_ID) {
			PX4_DEBUG("unexpected WIA1 0x%02x", WIA1);
		}

		if (WIA2 != Device_ID && WIA2 != Device_ID_AK09918) {
			PX4_DEBUG("unexpected WIA2 0x%02x", WIA2);
		}
	}

	return PX4_ERROR;
}

void AK09916::RunImpl()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_state) {
	case STATE::RESET:
		perf_count(_reset_perf);
		// CNTL3 SRST: Soft reset
		RegisterWrite(Register::CNTL3, CNTL3_BIT::SRST);
		_reset_timestamp = now;
		_failure_count = 0;
		_state = STATE::WAIT_FOR_RESET;
		ScheduleDelayed(100_ms);
		break;

	case STATE::WAIT_FOR_RESET: {
			uint8_t device_id = _is_ak09918 ? Device_ID_AK09918 : Device_ID;

			if ((RegisterRead(Register::WIA1) == Company_ID) && (RegisterRead(Register::WIA2) == device_id)) {
				// if reset succeeded then configure
				_state = STATE::CONFIGURE;
				ScheduleDelayed(100_ms);

			} else {
				// RESET not complete
				if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
					PX4_ERR("Reset failed, retrying");
					Reset();

				} else {
					PX4_DEBUG("Reset not complete, check again in 100 ms");
					ScheduleDelayed(100_ms);
				}
			}

			break;
		}

	case STATE::CONFIGURE:
		if (Configure()) {
			// if configure succeeded then start reading
			_state = STATE::READ;
			ScheduleOnInterval(20_ms, 20_ms); // 50 Hz

		} else {
			// CONFIGURE not complete
			if (hrt_elapsed_time(&_reset_timestamp) > 30_s) {
				PX4_ERR("Configure failed, resetting");
				Reset();

			} else {
				PX4_DEBUG("Configure failed, retrying");
				ScheduleDelayed(100_ms);
			}
		}

		break;

	case STATE::READ: {

			struct TransferBuffer {
				uint8_t ST1;
				uint8_t HXL;
				uint8_t HXH;
				uint8_t HYL;
				uint8_t HYH;
				uint8_t HZL;
				uint8_t HZH;
				uint8_t TMPS;
				uint8_t ST2;
			} buffer{};

			uint8_t cmd = static_cast<uint8_t>(Register::ST1);
			int ret = transfer(&cmd, 1, (uint8_t *)&buffer, sizeof(TransferBuffer));

			bool success = false;

			if (ret == PX4_OK) {
				if (buffer.ST2 & ST2_BIT::HOFL) {
					perf_count(_magnetic_sensor_overflow_perf);

				} else if (buffer.ST1 & ST1_BIT::DRDY) {
					const int16_t x = combine(buffer.HXH, buffer.HXL);
					const int16_t y = combine(buffer.HYH, buffer.HYL);
					const int16_t z = combine(buffer.HZH, buffer.HZL);

					// sensor's frame is +X forward (X), +Y right (Y), +Z down (Z)
					_px4_mag.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf)
								 + perf_event_count(_magnetic_sensor_overflow_perf));

					_px4_mag.update(now, x, y, z);

					success = true;

					if (_failure_count > 0) {
						_failure_count--;
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
			}
		}

		break;
	}
}

bool AK09916::Configure()
{
	_retries = 2;

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

	// mag resolution is 1.5 milli Gauss per bit (0.15 μT/LSB)
	_px4_mag.set_scale(1.5e-3f);

	return success;
}

bool AK09916::RegisterCheck(const register_config_t &reg_cfg)
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

uint8_t AK09916::RegisterRead(Register reg)
{
	const uint8_t cmd = static_cast<uint8_t>(reg);
	uint8_t buffer{};
	transfer(&cmd, 1, &buffer, 1);
	return buffer;
}

void AK09916::RegisterWrite(Register reg, uint8_t value)
{
	uint8_t buffer[2] { (uint8_t)reg, value };
	transfer(buffer, sizeof(buffer), nullptr, 0);
}

void AK09916::RegisterSetAndClearBits(Register reg, uint8_t setbits, uint8_t clearbits)
{
	const uint8_t orig_val = RegisterRead(reg);
	uint8_t val = (orig_val & ~clearbits) | setbits;

	if (orig_val != val) {
		RegisterWrite(reg, val);
	}
}
