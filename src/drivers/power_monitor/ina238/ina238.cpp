/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

/**
 * Driver for the I2C attached INA238
 */

#include "ina238.h"


INA238::INA238(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina238_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina238_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "ina238_collection_err")),
	_battery(battery_index, this, INA238_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
	float fvalue = DEFAULT_MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA238_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	_range = _max_current > (DEFAULT_MAX_CURRENT - 1.0f) ? INA238_ADCRANGE_HIGH : INA238_ADCRANGE_LOW;

	fvalue = DEFAULT_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA238_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	_current_lsb = _max_current / INA238_DN_MAX;

	// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
	_battery.setConnected(false);
	_battery.updateVoltage(0.f);
	_battery.updateCurrent(0.f);
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

INA238::~INA238()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
}

int INA238::read(uint8_t address, uint16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int INA238::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int INA238::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	if (write(INA238_REG_CONFIG, (uint16_t)(INA238_RST_RESET | _range)) != PX4_OK) {
		return ret;
	}

	uint16_t shunt_calibration = static_cast<uint16_t>(INA238_CONST * _current_lsb * _rshunt);

	if (_range == INA238_ADCRANGE_LOW) {
		shunt_calibration *= 4;
	}

	if (write(INA238_REG_SHUNTCAL, shunt_calibration) < 0) {
		return -3;
	}

	// Set the CONFIG for max I
	if (write(INA238_REG_CONFIG, (uint16_t) _range) != PX4_OK) {
		return ret;
	}

	// Start ADC continous mode here
	ret = write(INA238_REG_ADCCONFIG, (uint16_t)INA238_ADCCONFIG);

	start();
	_sensor_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int INA238::force_init()
{
	int ret = init();

	start();

	return ret;
}

int INA238::probe()
{
	uint16_t value{0};

	if (read(INA238_MANUFACTURER_ID, value) != PX4_OK || value != INA238_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	if (read(INA238_DEVICE_ID, value) != PX4_OK || (
		    INA238_DEVICEID(value) != INA238_MFG_DIE
	    )) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}

	return PX4_OK;
}


int INA238::collect()
{
	perf_begin(_sample_perf);

	if (_parameter_update_sub.updated()) {
		// Read from topic to clear updated flag
		parameter_update_s parameter_update;
		_parameter_update_sub.copy(&parameter_update);

		updateParams();
	}

	// read from the sensor
	// Note: If the power module is connected backwards, then the values of _current will be negative but otherwise valid.
	bool success{true};
	int16_t bus_voltage{0};
	int16_t current{0};

	success = success && (read(INA238_REG_VSBUS, bus_voltage) == PX4_OK);
	success = success && (read(INA238_REG_CURRENT, current) == PX4_OK);

	if (!success) {
		PX4_DEBUG("error reading from sensor");
		bus_voltage = current = 0;
	}

	_battery.setConnected(success);
	_battery.updateVoltage(static_cast<float>(bus_voltage * INA238_VSCALE));
	_battery.updateCurrent(static_cast<float>(current * _current_lsb));
	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void INA238::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA238_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void INA238::RunImpl()
{
	if (_initialized) {
		if (_collect_phase) {
			/* perform collection */
			if (collect() != PX4_OK) {
				perf_count(_collection_errors);
				/* if error restart the measurement state machine */
				start();
				return;
			}

			/* next phase is measurement */
			_collect_phase = true;

			if (_measure_interval > INA238_CONVERSION_INTERVAL) {
				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(_measure_interval - INA238_CONVERSION_INTERVAL);
				return;
			}
		}

		/* next phase is collection */
		_collect_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(INA238_CONVERSION_INTERVAL);

	} else {
		_battery.setConnected(false);
		_battery.updateVoltage(0.f);
		_battery.updateCurrent(0.f);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

		if (init() != PX4_OK) {
			ScheduleDelayed(INA238_INIT_RETRY_INTERVAL_US);
		}
	}
}

void INA238::print_status()
{
	I2CSPIDriverBase::print_status();

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 INA238_INIT_RETRY_INTERVAL_US / 1000);
	}
}
