package vl53l0x

//--------------------------------------------------------------------------------------------------

// This code is an adaptation and translation of well-formed C++ library to GOLANG for the distance
// measure sensor's family VL53L0X taken from https://github.com/pololu/vl53l0x-arduino:
//      https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.cpp
//      https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.h
//
// Some portion of code taken from Adafruit https://github.com/adafruit/Adafruit_VL53L0X:
//      https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/core/src/vl53l0x_api.cpp
//      https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/vl53l0x_def.h
//      https://github.com/adafruit/Adafruit_VL53L0X/blob/master/src/vl53l0x_device.h
//
//
// Copyright (c) 2018 Denis Dyakov
// Copyright (c) 2017 Pololu Corporation
// Copyright (c) 2016 Adafruit Industries
// Copyright (c) 2016 STMicroelectronics International N.V.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
// associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial
// portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
// BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//--------------------------------------------------------------------------------------------------

import (
	"errors"
	"time"

	i2c "github.com/d2r2/go-i2c"
	"github.com/davecgh/go-spew/spew"
)

// Registers from sensor hardware.
const (
	SYSRANGE_START = 0x00

	SYSTEM_THRESH_HIGH = 0x0C
	SYSTEM_THRESH_LOW  = 0x0E

	SYSTEM_SEQUENCE_CONFIG         = 0x01
	SYSTEM_RANGE_CONFIG            = 0x09
	SYSTEM_INTERMEASUREMENT_PERIOD = 0x04

	SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A

	GPIO_HV_MUX_ACTIVE_HIGH = 0x84

	SYSTEM_INTERRUPT_CLEAR = 0x0B

	RESULT_INTERRUPT_STATUS = 0x13
	RESULT_RANGE_STATUS     = 0x14

	RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC
	RESULT_CORE_RANGING_TOTAL_EVENTS_RTN  = 0xC0
	RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0
	RESULT_CORE_RANGING_TOTAL_EVENTS_REF  = 0xD4
	RESULT_PEAK_SIGNAL_RATE_REF           = 0xB6

	ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28

	I2C_SLAVE_DEVICE_ADDRESS = 0x8A

	MSRC_CONFIG_CONTROL = 0x60

	PRE_RANGE_CONFIG_MIN_SNR           = 0x27
	PRE_RANGE_CONFIG_VALID_PHASE_LOW   = 0x56
	PRE_RANGE_CONFIG_VALID_PHASE_HIGH  = 0x57
	PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64

	FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67
	FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47
	FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48
	FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44

	PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61
	PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62

	PRE_RANGE_CONFIG_VCSEL_PERIOD      = 0x50
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
	PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52

	SYSTEM_HISTOGRAM_BIN                  = 0x81
	HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33
	HISTOGRAM_CONFIG_READOUT_CTRL         = 0x55

	FINAL_RANGE_CONFIG_VCSEL_PERIOD       = 0x70
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI  = 0x71
	FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO  = 0x72
	CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20

	MSRC_CONFIG_TIMEOUT_MACROP = 0x46

	SOFT_RESET_GO2_SOFT_RESET_N = 0xBF
	IDENTIFICATION_MODEL_ID     = 0xC0
	IDENTIFICATION_REVISION_ID  = 0xC2

	OSC_CALIBRATE_VAL = 0xF8

	GLOBAL_CONFIG_VCSEL_WIDTH        = 0x32
	GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
	GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1
	GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2
	GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3
	GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4
	GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5

	GLOBAL_CONFIG_REF_EN_START_SELECT   = 0xB6
	DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
	DYNAMIC_SPAD_REF_EN_START_OFFSET    = 0x4F
	POWER_MANAGEMENT_GO1_POWER_FORCE    = 0x80

	VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89

	ALGO_PHASECAL_LIM            = 0x30
	ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30
)

// VcselPeriodType is a type of VCSEL (vertical cavity surface emitting laser) pulse period.
type VcselPeriodType int

const (
	// pre-range pulse period
	VcselPeriodPreRange VcselPeriodType = iota + 1
	// final range pulse period
	VcselPeriodFinalRange
)

// RangeSpec used to configure sensor for expected distance to measure.
type RangeSpec int

const (
	// Signal rate limit = 0.25 MCPS, laser pulse periods = (14, 10).
	RegularRange RangeSpec = iota + 1
	// Signal rate limit = 0.10 MCPS, laser pulse periods = (18, 14).
	// Use "long range" mode only when "regular" can't detect distance
	// (returned distance value is 8190 mm or more). It's ordinary
	// happens, when distance exceed something about a meter.
	LongRange
)

// String implement Stringer interface.
func (v RangeSpec) String() string {
	switch v {
	case RegularRange:
		return "RegularRange"
	case LongRange:
		return "LongRange"
	default:
		return "<unknown>"
	}
}

// SpeedAccuracySpec used to configure sensor for accuracy/measure time.
// It's clear that to improve accuracy, you should increase
// measure time.
type SpeedAccuracySpec int

const (
	// HighSpeed distance measurement takes 20 ms.
	HighSpeed SpeedAccuracySpec = iota + 1
	// RegularAccuracy distance measurement takes 33 ms.
	RegularAccuracy
	// GoodAccuracy distance measurement takes 66 ms.
	GoodAccuracy
	// HighAccuracy distance measurement takes 100 ms.
	HighAccuracy
	// HighestAccuracy distance measurement takes 200 ms.
	HighestAccuracy
)

// String implement Stringer interface.
func (v SpeedAccuracySpec) String() string {
	switch v {
	case HighSpeed:
		return "HighSpeed"
	case RegularAccuracy:
		return "RegularAccuracy"
	case GoodAccuracy:
		return "GoodAccuracy"
	case HighAccuracy:
		return "HighAccuracy"
	case HighestAccuracy:
		return "HighestAccuracy"
	default:
		return "<unknown>"
	}
}

// Vl53l0x contains sensor data and corresponding methods.
type Vl53l0x struct {
	// read by init and used when starting measurement;
	// is StopVariable field of VL53L0X_DevData_t structure in API
	stopVariable uint8
	// total measurement timing budget in microseconds
	measurementTimingBudgetUsec uint32
	// default timeout value
	ioTimeout time.Duration
}

// NewVl53l0x creates sensor instance.
func NewVl53l0x() *Vl53l0x {
	v := &Vl53l0x{}
	return v
}

// Config configure sensor expected distance range and time to make a measurement.
func (v *Vl53l0x) Config(i2c *i2c.I2C, rng RangeSpec, speed SpeedAccuracySpec) error {

	lg.Debug("Start config")

	switch rng {
	case RegularRange:
		// default is 0.25 MCPS
		err := v.SetSignalRateLimit(i2c, 0.25)
		if err != nil {
			return err
		}
		// defaults are 14 and 10 PCLKs)
		err = v.SetVcselPulsePeriod(i2c, VcselPeriodPreRange, 14)
		if err != nil {
			return err
		}
		err = v.SetVcselPulsePeriod(i2c, VcselPeriodFinalRange, 10)
		if err != nil {
			return err
		}
	case LongRange:
		// lower the return signal rate limit (default is 0.25 MCPS)
		err := v.SetSignalRateLimit(i2c, 0.1)
		if err != nil {
			return err
		}
		// increase laser pulse periods (defaults are 14 and 10 PCLKs)
		err = v.SetVcselPulsePeriod(i2c, VcselPeriodPreRange, 18)
		if err != nil {
			return err
		}
		err = v.SetVcselPulsePeriod(i2c, VcselPeriodFinalRange, 14)
		if err != nil {
			return err
		}
	}

	switch speed {
	case HighSpeed:
		// reduce timing budget to 20 ms (default is about 33 ms)
		err := v.SetMeasurementTimingBudget(i2c, 20000)
		if err != nil {
			return err
		}
	case RegularAccuracy:
		// default is about 33 ms
		err := v.SetMeasurementTimingBudget(i2c, 33000)
		if err != nil {
			return err
		}
	case GoodAccuracy:
		// increase timing budget to 66 ms
		err := v.SetMeasurementTimingBudget(i2c, 66000)
		if err != nil {
			return err
		}
	case HighAccuracy:
		// increase timing budget to 100 ms
		err := v.SetMeasurementTimingBudget(i2c, 100000)
		if err != nil {
			return err
		}
	case HighestAccuracy:
		// increase timing budget to 200 ms
		err := v.SetMeasurementTimingBudget(i2c, 200000)
		if err != nil {
			return err
		}
	}

	lg.Debug("End config")

	return nil
}

// Reset soft-reset the sensor.
// Based on VL53L0X_ResetDevice().
func (v *Vl53l0x) Reset(i2c *i2c.I2C) error {
	// Set reset bit
	lg.Debug("Set reset bit")
	err := v.writeRegU8(i2c, SOFT_RESET_GO2_SOFT_RESET_N, 0x00)
	if err != nil {
		return err
	}
	// Wait for some time
	err = v.waitUntilOrTimeout(i2c, IDENTIFICATION_MODEL_ID,
		func(checkReg byte, err error) (bool, error) {
			return checkReg == 0, err
		})
	if err != nil {
		return err
	}
	// Release reset
	lg.Debug("Release reset bit")
	err = v.writeRegU8(i2c, SOFT_RESET_GO2_SOFT_RESET_N, 0x01)
	if err != nil {
		return err
	}
	// Wait for some time
	err = v.waitUntilOrTimeout(i2c, IDENTIFICATION_MODEL_ID,
		func(checkReg byte, err error) (bool, error) {
			// Skip error like "read /dev/i2c-x: no such device or address"
			// for a while, because sensor in reboot has temporary
			// no connection to I2C-bus. So, that is why we are
			// returning nil instead of err, suppressing this.
			return checkReg != 0, nil
		})
	if err != nil {
		return err
	}
	return nil
}

// GetProductMinorRevision takes revision from sensor hardware.
// Based on VL53L0X_GetProductRevision.
func (v *Vl53l0x) GetProductMinorRevision(i2c *i2c.I2C) (byte, error) {
	u8, err := v.readRegU8(i2c, IDENTIFICATION_REVISION_ID)
	if err != nil {
		return 0, err
	}
	return (u8 & 0xF0) >> 4, nil
}

// SetAddress change default address of sensor and reopen I2C-connection.
func (v *Vl53l0x) SetAddress(i2cRef **i2c.I2C, newAddr byte) error {
	err := v.writeRegU8(*i2cRef, I2C_SLAVE_DEVICE_ADDRESS, newAddr&0x7F)
	if err != nil {
		return err
	}
	*i2cRef, err = i2c.NewI2C(newAddr, (*i2cRef).GetBus())
	return err
}

// Init initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
func (v *Vl53l0x) Init(i2c *i2c.I2C) error {

	v.setTimeout(time.Millisecond * 1000)

	// VL53L0X_DataInit() begin

	// "Set I2C standard mode"
	err := v.writeRegU8(i2c, 0x88, 0x00)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x80, Value: 0x01},
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x00},
	}...)
	if err != nil {
		return err
	}
	v.stopVariable, err = v.readRegU8(i2c, 0x91)
	if err != nil {
		return err
	}
	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x00, Value: 0x01},
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x80, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	// disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
	u8, err := v.readRegU8(i2c, MSRC_CONFIG_CONTROL)
	if err != nil {
		return err
	}
	err = v.writeRegU8(i2c, MSRC_CONFIG_CONTROL, u8|0x12)
	if err != nil {
		return err
	}

	// set final range signal rate limit to 0.25 MCPS (million counts per second)
	err = v.SetSignalRateLimit(i2c, 0.25)
	if err != nil {
		return err
	}

	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, 0xFF)
	if err != nil {
		return err
	}

	// VL53L0X_DataInit() end

	// VL53L0X_StaticInit() begin

	spadInfo, err := v.getSpadInfo(i2c)
	if err != nil {
		return err
	}

	// The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
	// the API, but the same data seems to be more easily readable from
	// GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
	spadMap := make([]byte, 6)
	err = v.readRegBytes(i2c, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spadMap)
	if err != nil {
		return err
	}

	// -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: DYNAMIC_SPAD_REF_EN_START_OFFSET, Value: 0x00},
		{Reg: DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, Value: 0x2C},
		{Reg: 0xFF, Value: 0x00},
		{Reg: GLOBAL_CONFIG_REF_EN_START_SELECT, Value: 0xB4},
	}...)
	if err != nil {
		return err
	}

	var firstSpadToEnable byte
	if spadInfo.TypeIsAperture {
		// 12 is the first aperture spad
		firstSpadToEnable = 12
	}
	var spadsEnabled byte

	var i byte
	for i = 0; i < 48; i++ {
		if i < firstSpadToEnable || spadsEnabled == spadInfo.Count {
			// This bit is lower than the first one that should be enabled, or
			// (reference_spad_count) bits have already been enabled, so zero this bit
			spadMap[i/8] &= ^(1 << (i % 8))
		} else if (spadMap[i/8]>>(i%8))&0x1 != 0 {
			spadsEnabled++
		}
	}

	err = v.writeBytes(i2c, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spadMap)
	if err != nil {
		return err
	}

	// -- VL53L0X_set_reference_spads() end

	// -- VL53L0X_load_tuning_settings() begin
	// DefaultTuningSettings from vl53l0x_tuning.h

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x09, Value: 0x00},
		{Reg: 0x10, Value: 0x00},
		{Reg: 0x11, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x24, Value: 0x01},
		{Reg: 0x25, Value: 0xFF},
		{Reg: 0x75, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x4E, Value: 0x2C},
		{Reg: 0x48, Value: 0x00},
		{Reg: 0x30, Value: 0x20},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x30, Value: 0x09},
		{Reg: 0x54, Value: 0x00},
		{Reg: 0x31, Value: 0x04},
		{Reg: 0x32, Value: 0x03},
		{Reg: 0x40, Value: 0x83},
		{Reg: 0x46, Value: 0x25},
		{Reg: 0x60, Value: 0x00},
		{Reg: 0x27, Value: 0x00},
		{Reg: 0x50, Value: 0x06},
		{Reg: 0x51, Value: 0x00},
		{Reg: 0x52, Value: 0x96},
		{Reg: 0x56, Value: 0x08},
		{Reg: 0x57, Value: 0x30},
		{Reg: 0x61, Value: 0x00},
		{Reg: 0x62, Value: 0x00},
		{Reg: 0x64, Value: 0x00},
		{Reg: 0x65, Value: 0x00},
		{Reg: 0x66, Value: 0xA0},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x22, Value: 0x32},
		{Reg: 0x47, Value: 0x14},
		{Reg: 0x49, Value: 0xFF},
		{Reg: 0x4A, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x7A, Value: 0x0A},
		{Reg: 0x7B, Value: 0x00},
		{Reg: 0x78, Value: 0x21},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x23, Value: 0x34},
		{Reg: 0x42, Value: 0x00},
		{Reg: 0x44, Value: 0xFF},
		{Reg: 0x45, Value: 0x26},
		{Reg: 0x46, Value: 0x05},
		{Reg: 0x40, Value: 0x40},
		{Reg: 0x0E, Value: 0x06},
		{Reg: 0x20, Value: 0x1A},
		{Reg: 0x43, Value: 0x40},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x34, Value: 0x03},
		{Reg: 0x35, Value: 0x44},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x31, Value: 0x04},
		{Reg: 0x4B, Value: 0x09},
		{Reg: 0x4C, Value: 0x05},
		{Reg: 0x4D, Value: 0x04},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x44, Value: 0x00},
		{Reg: 0x45, Value: 0x20},
		{Reg: 0x47, Value: 0x08},
		{Reg: 0x48, Value: 0x28},
		{Reg: 0x67, Value: 0x00},
		{Reg: 0x70, Value: 0x04},
		{Reg: 0x71, Value: 0x01},
		{Reg: 0x72, Value: 0xFE},
		{Reg: 0x76, Value: 0x00},
		{Reg: 0x77, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x0D, Value: 0x01},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x80, Value: 0x01},
		{Reg: 0x01, Value: 0xF8},
	}...)
	if err != nil {
		return err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x8E, Value: 0x01},
		{Reg: 0x00, Value: 0x01},
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x80, Value: 0x00},
	}...)
	if err != nil {
		return err
	}

	// -- VL53L0X_load_tuning_settings() end

	// "Set interrupt config to new sample ready"
	// -- VL53L0X_SetGpioConfig() begin

	err = v.writeRegU8(i2c, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
	if err != nil {
		return err
	}
	u8, err = v.readRegU8(i2c, GPIO_HV_MUX_ACTIVE_HIGH)
	if err != nil {
		return err
	}
	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: GPIO_HV_MUX_ACTIVE_HIGH, Value: u8 & ^byte(0x10)}, // active low
		{Reg: SYSTEM_INTERRUPT_CLEAR, Value: 0x01},
	}...)
	if err != nil {
		return err
	}

	// -- VL53L0X_SetGpioConfig() end

	u32, err := v.getMeasurementTimingBudget(i2c)
	if err != nil {
		return err
	}
	v.measurementTimingBudgetUsec = u32

	// "Disable MSRC and TCC by default"
	// MSRC = Minimum Signal Rate Check
	// TCC = Target CentreCheck
	// -- VL53L0X_SetSequenceStepEnable() begin

	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, 0xE8)
	if err != nil {
		return err
	}

	// -- VL53L0X_SetSequenceStepEnable() end

	// "Recalculate timing budget"
	err = v.SetMeasurementTimingBudget(i2c, v.measurementTimingBudgetUsec)
	if err != nil {
		return err
	}

	// VL53L0X_StaticInit() end

	// VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

	// -- VL53L0X_perform_vhv_calibration() begin

	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, 0x01)
	if err != nil {
		return err
	}
	err = v.performSingleRefCalibration(i2c, 0x40)
	if err != nil {
		return err
	}

	// -- VL53L0X_perform_vhv_calibration() end

	// -- VL53L0X_perform_phase_calibration() begin

	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, 0x02)
	if err != nil {
		return err
	}
	err = v.performSingleRefCalibration(i2c, 0x00)
	if err != nil {
		return err
	}

	// -- VL53L0X_perform_phase_calibration() end

	// "restore the previous Sequence Config"
	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, 0xE8)
	if err != nil {
		return err
	}

	// VL53L0X_PerformRefCalibration() end

	return nil
}

// SetSignalRateLimit set the return signal rate limit check value in units of MCPS
// (mega counts per second). "This represents the amplitude of the signal reflected
// from the target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
func (v *Vl53l0x) SetSignalRateLimit(i2c *i2c.I2C, limitMcps float32) error {
	if limitMcps < 0 || limitMcps > 511.99 {
		return errors.New("out of MCPS range")
	}
	// Q9.7 fixed point format (9 integer bits, 7 fractional bits)
	err := v.writeRegU16(i2c, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
		uint16(limitMcps*(1<<7)))
	return err
}

// GetSignalRateLimit gets the return signal rate limit check value in MCPS.
func (v *Vl53l0x) GetSignalRateLimit(i2c *i2c.I2C) (float32, error) {
	u16, err := v.readRegU16(i2c, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT)
	if err != nil {
		return 0, err
	}
	limit := float32(u16) / (1 << 7)
	return limit, nil
}

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
type SequenceStepEnables struct {
	TCC        bool
	MSRC       bool
	DSS        bool
	PreRange   bool
	FinalRange bool
}

type SequenceStepTimeouts struct {
	PreRangeVcselPeriodPclks   uint16
	FinalRangeVcselPeriodPclks uint16

	MsrcDssTccMclks uint16
	PreRangeMclks   uint16
	FinalRangeMclks uint16

	MsrcDssTccUsec uint32
	PreRangeUsec   uint32
	FinalRangeUsec uint32
}

// Get sequence step enables.
// Based on VL53L0X_GetSequenceStepEnables().
func (v *Vl53l0x) getSequenceStepEnables(i2c *i2c.I2C) (*SequenceStepEnables, error) {

	lg.Debug("Start getting sequence step enables")

	sequenceConfig, err := v.readRegU8(i2c, SYSTEM_SEQUENCE_CONFIG)
	if err != nil {
		return nil, err
	}

	se := &SequenceStepEnables{
		TCC:        (sequenceConfig>>4)&0x1 != 0,
		DSS:        (sequenceConfig>>3)&0x1 != 0,
		MSRC:       (sequenceConfig>>2)&0x1 != 0,
		PreRange:   (sequenceConfig>>6)&0x1 != 0,
		FinalRange: (sequenceConfig>>7)&0x1 != 0,
	}
	return se, nil
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value. Based on VL53L0X_decode_vcsel_period().
func (v *Vl53l0x) decodeVcselPeriod(value byte) byte {
	return (value + 1) << 1
}

// Encode VCSEL pulse period register value from period in PCLKs.
// Based on VL53L0X_encode_vcsel_period().
func (v *Vl53l0x) encodeVcselPeriod(periodPclks byte) byte {
	return periodPclks>>1 - 1
}

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs.
// Based on VL53L0X_calc_macro_period_ps().
// PLL_period_ps = 1655; macro_period_vclks = 2304.
func (v *Vl53l0x) calcMacroPeriod(vcselPeriodPclks uint16) uint32 {
	return (uint32(vcselPeriodPclks)*2304*1655 + 500) / 1000
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs.
// Based on VL53L0X_calc_timeout_us().
func (v *Vl53l0x) timeoutMclksToMicroseconds(timeoutPeriodMclks uint16, vcselPeriodPclks uint16) uint32 {
	macroPeriodNsec := v.calcMacroPeriod(vcselPeriodPclks)
	return (uint32(timeoutPeriodMclks)*macroPeriodNsec + macroPeriodNsec/2) / 1000
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs.
// Based on VL53L0X_calc_timeout_mclks().
func (v *Vl53l0x) timeoutMicrosecondsToMclks(timeoutPeriodUsec uint32, vcselPeriodPclks uint16) uint32 {
	macroPeriodNsec := v.calcMacroPeriod(vcselPeriodPclks)
	return (timeoutPeriodUsec*1000 + macroPeriodNsec/2) / macroPeriodNsec
}

// SetVcselPulsePeriod set the VCSEL (vertical cavity surface emitting laser) pulse period
// for the given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14),
//  final: 8 to 14 (initialized default: 10).
// Based on VL53L0X_set_vcsel_pulse_period().
func (v *Vl53l0x) SetVcselPulsePeriod(i2c *i2c.I2C, tpe VcselPeriodType, periodPclks uint8) error {
	vcselPeriodReg := v.encodeVcselPeriod(periodPclks)

	enables, err := v.getSequenceStepEnables(i2c)
	if err != nil {
		return err
	}
	timeouts, err := v.getSequenceStepTimeouts(i2c, *enables)
	if err != nil {
		return err
	}

	// "Apply specific settings for the requested clock period"
	// "Re-calculate and apply timeouts, in macro periods"

	// "When the VCSEL period for the pre or final range is changed,
	// the corresponding timeout must be read from the device using
	// the current VCSEL period, then the new VCSEL period can be
	// applied. The timeout then must be written back to the device
	// using the new VCSEL period.
	//
	// For the MSRC timeout, the same applies - this timeout being
	// dependant on the pre-range vcsel period."

	if tpe == VcselPeriodPreRange {
		// "Set phase check limits"
		switch periodPclks {
		case 12:
			err := v.writeRegU8(i2c, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18)
			if err != nil {
				return err
			}
		case 14:
			err := v.writeRegU8(i2c, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30)
			if err != nil {
				return err
			}
		case 16:
			err := v.writeRegU8(i2c, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40)
			if err != nil {
				return err
			}
		case 18:
			err := v.writeRegU8(i2c, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50)
			if err != nil {
				return err
			}
		default:
			// invalid period
			return errors.New("invalid period")
		}
		err = v.writeRegU8(i2c, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08)
		if err != nil {
			return err
		}

		// apply new VCSEL period
		err = v.writeRegU8(i2c, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodReg)
		if err != nil {
			return err
		}

		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

		newPreRangeTimeoutMclks := v.timeoutMicrosecondsToMclks(timeouts.PreRangeUsec,
			uint16(periodPclks))

		err = v.writeRegU16(i2c, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
			v.encodeTimeout(uint16(newPreRangeTimeoutMclks)))
		if err != nil {
			return err
		}

		// set_sequence_step_timeout() end

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

		newMsrcTimeoutMclks := v.timeoutMicrosecondsToMclks(timeouts.MsrcDssTccUsec,
			uint16(periodPclks))

		if newMsrcTimeoutMclks > 256 {
			newMsrcTimeoutMclks = 255
		} else {
			newMsrcTimeoutMclks--
		}
		err = v.writeRegU8(i2c, MSRC_CONFIG_TIMEOUT_MACROP, uint8(newMsrcTimeoutMclks))
		if err != nil {
			return err
		}

		// set_sequence_step_timeout() end
	} else if tpe == VcselPeriodFinalRange {
		switch periodPclks {
		case 8:
			err := v.writeRegValues(i2c, []RegBytePair{
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, Value: 0x10},
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_LOW, Value: 0x08},
				{Reg: GLOBAL_CONFIG_VCSEL_WIDTH, Value: 0x02},
				{Reg: ALGO_PHASECAL_CONFIG_TIMEOUT, Value: 0x0C},
				{Reg: 0xFF, Value: 0x01},
				{Reg: ALGO_PHASECAL_LIM, Value: 0x30},
				{Reg: 0xFF, Value: 0x00},
			}...)
			if err != nil {
				return err
			}
		case 10:
			err := v.writeRegValues(i2c, []RegBytePair{
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, Value: 0x28},
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_LOW, Value: 0x08},
				{Reg: GLOBAL_CONFIG_VCSEL_WIDTH, Value: 0x03},
				{Reg: ALGO_PHASECAL_CONFIG_TIMEOUT, Value: 0x09},
				{Reg: 0xFF, Value: 0x01},
				{Reg: ALGO_PHASECAL_LIM, Value: 0x20},
				{Reg: 0xFF, Value: 0x00},
			}...)
			if err != nil {
				return err
			}
		case 12:
			err := v.writeRegValues(i2c, []RegBytePair{
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, Value: 0x38},
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_LOW, Value: 0x08},
				{Reg: GLOBAL_CONFIG_VCSEL_WIDTH, Value: 0x03},
				{Reg: ALGO_PHASECAL_CONFIG_TIMEOUT, Value: 0x08},
				{Reg: 0xFF, Value: 0x01},
				{Reg: ALGO_PHASECAL_LIM, Value: 0x20},
				{Reg: 0xFF, Value: 0x00},
			}...)
			if err != nil {
				return err
			}
		case 14:
			err := v.writeRegValues(i2c, []RegBytePair{
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, Value: 0x48},
				{Reg: FINAL_RANGE_CONFIG_VALID_PHASE_LOW, Value: 0x08},
				{Reg: GLOBAL_CONFIG_VCSEL_WIDTH, Value: 0x03},
				{Reg: ALGO_PHASECAL_CONFIG_TIMEOUT, Value: 0x07},
				{Reg: 0xFF, Value: 0x01},
				{Reg: ALGO_PHASECAL_LIM, Value: 0x20},
				{Reg: 0xFF, Value: 0x00},
			}...)
			if err != nil {
				return err
			}
		default:
			// invalid period
			return errors.New("invalid period")
		}

		// apply new VCSEL period
		err = v.writeRegU8(i2c, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcselPeriodReg)
		if err != nil {
			return err
		}

		// update timeouts

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		newFinalRangeTimeoutMclks := v.timeoutMicrosecondsToMclks(timeouts.FinalRangeUsec,
			uint16(periodPclks))

		if enables.PreRange {
			newFinalRangeTimeoutMclks += uint32(timeouts.PreRangeMclks)
		}

		err = v.writeRegU16(i2c, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
			v.encodeTimeout(uint16(newFinalRangeTimeoutMclks)))
		if err != nil {
			return err
		}

		// set_sequence_step_timeout end
	} else {
		// invalid type
		return errors.New("invalid type")
	}

	// "Finally, the timing budget must be re-applied"

	err = v.SetMeasurementTimingBudget(i2c, v.measurementTimingBudgetUsec)
	if err != nil {
		return err
	}

	// "Perform the phase calibration. This is needed after changing on vcsel period."
	// VL53L0X_perform_phase_calibration() begin

	sequenceConfig, err := v.readRegU8(i2c, SYSTEM_SEQUENCE_CONFIG)
	if err != nil {
		return err
	}
	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, 0x02)
	if err != nil {
		return err
	}
	err = v.performSingleRefCalibration(i2c, 0x0)
	if err != nil {
		return err
	}
	err = v.writeRegU8(i2c, SYSTEM_SEQUENCE_CONFIG, sequenceConfig)
	if err != nil {
		return err
	}

	// VL53L0X_perform_phase_calibration() end

	return nil
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// Based on VL53L0X_get_vcsel_pulse_period().
func (v *Vl53l0x) getVcselPulsePeriod(i2c *i2c.I2C, tpe VcselPeriodType) (byte, error) {

	lg.Debug("Start getting VCSEL pulse period")

	switch tpe {
	case VcselPeriodPreRange:
		u8, err := v.readRegU8(i2c, PRE_RANGE_CONFIG_VCSEL_PERIOD)
		if err != nil {
			return 0, err
		}
		return v.decodeVcselPeriod(u8), nil
	case VcselPeriodFinalRange:
		u8, err := v.readRegU8(i2c, FINAL_RANGE_CONFIG_VCSEL_PERIOD)
		if err != nil {
			return 0, err
		}
		return v.decodeVcselPeriod(u8), nil
	default:
		return 0, errors.New("invalid VCSEL period type specified")
	}
}

// StartContinuous start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement. Based on VL53L0X_StartMeasurement().
func (v *Vl53l0x) StartContinuous(i2c *i2c.I2C, periodMs uint32) error {

	lg.Debug("Start continuous")

	err := v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x80, Value: 0x01},
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x00},
		{Reg: 0x91, Value: v.stopVariable},
		{Reg: 0x00, Value: 0x01},
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x80, Value: 0x00},
	}...)
	if err != nil {
		return err
	}
	if periodMs != 0 {
		// continuous timed mode

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

		oscCalibrateVal, err := v.readRegU16(i2c, OSC_CALIBRATE_VAL)
		if err != nil {
			return err
		}

		if oscCalibrateVal != 0 {
			periodMs *= uint32(oscCalibrateVal)
		}

		err = v.writeRegU32(i2c, SYSTEM_INTERMEASUREMENT_PERIOD, periodMs)
		if err != nil {
			return err
		}

		// VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

		err = v.writeRegU8(i2c, SYSRANGE_START, 0x04) // VL53L0X_REG_SYSRANGE_MODE_TIMED
		if err != nil {
			return err
		}
	} else {
		// continuous back-to-back mode
		err = v.writeRegU8(i2c, SYSRANGE_START, 0x02) // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
		if err != nil {
			return err
		}
	}
	return nil
}

// StopContinuous stop continuous measurements.
// Based on VL53L0X_StopMeasurement().
func (v *Vl53l0x) StopContinuous(i2c *i2c.I2C) error {

	lg.Debug("Stop continuous")

	err := v.writeRegValues(i2c, []RegBytePair{
		{Reg: SYSRANGE_START, Value: 0x01}, // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x00},
		{Reg: 0x91, Value: 0x00},
		{Reg: 0x00, Value: 0x01},
		{Reg: 0xFF, Value: 0x00},
	}...)
	return err
}

// Read measured distance from the sensor.
func (v *Vl53l0x) readRangeMillimeters(i2c *i2c.I2C) (uint16, error) {

	err := v.waitUntilOrTimeout(i2c, RESULT_INTERRUPT_STATUS,
		func(checkReg byte, err error) (bool, error) {
			return checkReg&0x07 != 0, err
		})
	if err != nil {
		return 0, err
	}

	// assumptions: Linearity Corrective Gain is 1000 (default);
	// fractional ranging is not enabled
	rng, err := v.readRegU16(i2c, RESULT_RANGE_STATUS+10)
	if err != nil {
		return 0, err
	}
	err = v.writeRegU8(i2c, SYSTEM_INTERRUPT_CLEAR, 0x01)
	if err != nil {
		return 0, err
	}

	return rng, nil
}

// ReadRangeContinuousMillimeters returns a range reading in millimeters
// when continuous mode is active (readRangeSingleMillimeters() also calls
// this function after starting a single-shot range measurement).
func (v *Vl53l0x) ReadRangeContinuousMillimeters(i2c *i2c.I2C) (uint16, error) {

	lg.Debug("Read range continuous")

	return v.readRangeMillimeters(i2c)
}

// ReadRangeSingleMillimeters performs a single-shot range measurement and returns the reading in
// millimeters based on VL53L0X_PerformSingleRangingMeasurement().
func (v *Vl53l0x) ReadRangeSingleMillimeters(i2c *i2c.I2C) (uint16, error) {

	lg.Debug("Read range single")

	err := v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x80, Value: 0x01},
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x00},
		{Reg: 0x91, Value: v.stopVariable},
		{Reg: 0x00, Value: 0x01},
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x80, Value: 0x00},
		{Reg: SYSRANGE_START, Value: 0x01},
	}...)
	if err != nil {
		return 0, err
	}

	// "Wait until start bit has been cleared"
	err = v.waitUntilOrTimeout(i2c, SYSRANGE_START,
		func(checkReg byte, err error) (bool, error) {
			return checkReg&0x01 == 0, err
		})
	if err != nil {
		return 0, err
	}
	return v.readRangeMillimeters(i2c)
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
func (v *Vl53l0x) decodeTimeout(regVal uint16) uint16 {
	// format: "(LSByte * 2^MSByte) + 1"
	return (regVal&0x00FF)<<((regVal&0xFF00)>>8) + 1
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
func (v *Vl53l0x) encodeTimeout(timeoutMclks uint16) uint16 {
	// format: "(LSByte * 2^MSByte) + 1"
	var lsByte uint32
	var msByte uint16

	if timeoutMclks > 0 {
		lsByte = uint32(timeoutMclks) - 1
		for lsByte&0xFFFFFF00 > 0 {
			lsByte >>= 1
			msByte++
		}
		return msByte<<8 | uint16(lsByte&0xFF)
	}
	return 0
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values.
func (v *Vl53l0x) getSequenceStepTimeouts(i2c *i2c.I2C, enables SequenceStepEnables) (*SequenceStepTimeouts, error) {

	lg.Debug("Start getting sequence step timeouts")

	timeouts := &SequenceStepTimeouts{}

	u8, err := v.getVcselPulsePeriod(i2c, VcselPeriodPreRange)
	if err != nil {
		return nil, err
	}
	timeouts.PreRangeVcselPeriodPclks = uint16(u8)

	u8, err = v.readRegU8(i2c, MSRC_CONFIG_TIMEOUT_MACROP)
	if err != nil {
		return nil, err
	}
	timeouts.MsrcDssTccMclks = uint16(u8) + 1

	timeouts.MsrcDssTccUsec = v.timeoutMclksToMicroseconds(timeouts.MsrcDssTccMclks,
		timeouts.PreRangeVcselPeriodPclks)

	u16, err := v.readRegU16(i2c, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI)
	if err != nil {
		return nil, err
	}
	timeouts.PreRangeMclks = v.decodeTimeout(u16)

	timeouts.PreRangeUsec = v.timeoutMclksToMicroseconds(timeouts.PreRangeMclks,
		timeouts.PreRangeVcselPeriodPclks)

	u8, err = v.getVcselPulsePeriod(i2c, VcselPeriodFinalRange)
	if err != nil {
		return nil, err
	}
	timeouts.FinalRangeVcselPeriodPclks = uint16(u8)

	u16, err = v.readRegU16(i2c, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI)
	if err != nil {
		return nil, err
	}
	timeouts.FinalRangeMclks = v.decodeTimeout(u16)

	if enables.PreRange {
		timeouts.FinalRangeMclks -= timeouts.PreRangeMclks
	}

	timeouts.FinalRangeUsec = v.timeoutMclksToMicroseconds(timeouts.FinalRangeMclks,
		timeouts.FinalRangeVcselPeriodPclks)

	return timeouts, nil
}

// SetMeasurementTimingBudget set the measurement timing budget in microseconds,
// which is the time allowed for one measurement; the ST API and this library take care
// of splitting the timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// Based on VL53L0X_set_measurement_timing_budget_micro_seconds().
func (v *Vl53l0x) SetMeasurementTimingBudget(i2c *i2c.I2C, budgetUsec uint32) error {
	const StartOverhead = 1320 // note that this is different than the value in get_
	const EndOverhead = 960
	const MsrcOverhead = 660
	const TccOverhead = 590
	const DssOverhead = 690
	const PreRangeOverhead = 660
	const FinalRangeOverhead = 550

	const MinTimingBudget = 20000

	lg.Debug("Start setting measurement timing budget")

	if budgetUsec < MinTimingBudget {
		return errors.New("budget is lower than minimum allowed")
	}
	var usedBudgetUsec uint32 = StartOverhead + EndOverhead

	enables, err := v.getSequenceStepEnables(i2c)
	if err != nil {
		return err
	}
	lg.Debugf("Sequence step enables = %#v", enables)
	timeouts, err := v.getSequenceStepTimeouts(i2c, *enables)
	if err != nil {
		return err
	}
	lg.Debugf("Sequence step timeouts = %#v", timeouts)

	if enables.TCC {
		usedBudgetUsec += timeouts.MsrcDssTccUsec + TccOverhead
	}

	if enables.DSS {
		usedBudgetUsec += 2 * (timeouts.MsrcDssTccUsec + DssOverhead)
	} else if enables.MSRC {
		usedBudgetUsec += timeouts.MsrcDssTccUsec + MsrcOverhead
	}

	if enables.PreRange {
		usedBudgetUsec += timeouts.PreRangeUsec + PreRangeOverhead
	}

	if enables.FinalRange {
		usedBudgetUsec += FinalRangeOverhead

		// "Note that the final range timeout is determined by the timing
		// budget and the sum of all other timeouts within the sequence.
		// If there is no room for the final range timeout, then an error
		// will be set. Otherwise the remaining time will be applied to
		// the final range."

		if usedBudgetUsec > budgetUsec {
			// "Requested timeout too big."
			return errors.New("requested timeout too big")
		}

		finalRangeTimeoutUsec := budgetUsec - usedBudgetUsec

		// set_sequence_step_timeout() begin
		// (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

		// "For the final range timeout, the pre-range timeout
		//  must be added. To do this both final and pre-range
		//  timeouts must be expressed in macro periods MClks
		//  because they have different vcsel periods."

		lg.Debug("set_sequence_step_timeout() begin")

		finalRangeTimeoutMclks := v.timeoutMicrosecondsToMclks(finalRangeTimeoutUsec,
			timeouts.FinalRangeVcselPeriodPclks)

		if enables.PreRange {
			finalRangeTimeoutMclks += uint32(timeouts.PreRangeMclks)
		}

		err = v.writeRegU16(i2c, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
			v.encodeTimeout(uint16(finalRangeTimeoutMclks)))
		if err != nil {
			return err
		}

		lg.Debug("set_sequence_step_timeout() end")

		// set_sequence_step_timeout() end

		v.measurementTimingBudgetUsec = budgetUsec // store for internal reuse
	}

	lg.Debug("End setting measurement timing budget")

	return nil
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us (microseconds).
func (v *Vl53l0x) getMeasurementTimingBudget(i2c *i2c.I2C) (uint32, error) {
	const StartOverhead = 1910 // note that this is different than the value in set_
	const EndOverhead = 960
	const MsrcOverhead = 660
	const TccOverhead = 590
	const DssOverhead = 690
	const PreRangeOverhead = 660
	const FinalRangeOverhead = 550

	var budgetUsec uint32 = StartOverhead + EndOverhead

	enables, err := v.getSequenceStepEnables(i2c)
	if err != nil {
		return 0, err
	}
	timeouts, err := v.getSequenceStepTimeouts(i2c, *enables)
	if err != nil {
		return 0, err
	}

	if enables.TCC {
		budgetUsec += timeouts.MsrcDssTccUsec + TccOverhead
	}

	if enables.DSS {
		budgetUsec += 2 * (timeouts.MsrcDssTccUsec + DssOverhead)
	} else if enables.MSRC {
		budgetUsec += timeouts.MsrcDssTccUsec + MsrcOverhead
	}

	if enables.PreRange {
		budgetUsec += timeouts.PreRangeUsec + PreRangeOverhead
	}

	if enables.FinalRange {
		budgetUsec += timeouts.FinalRangeUsec + FinalRangeOverhead
	}

	v.measurementTimingBudgetUsec = budgetUsec // store for internal reuse

	return budgetUsec, nil
}

// SpadInfo keeps information about sensor
// SPAD (single photon avalanche diode) photodetector structure.
type SpadInfo struct {
	Count          byte
	TypeIsAperture bool
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type.
func (v *Vl53l0x) getSpadInfo(i2c *i2c.I2C) (*SpadInfo, error) {
	var tmp uint8

	err := v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x80, Value: 0x01},
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x00},
	}...)
	if err != nil {
		return nil, err
	}

	err = v.writeRegU8(i2c, 0xFF, 0x06)
	if err != nil {
		return nil, err
	}
	u8, err := v.readRegU8(i2c, 0x83)
	if err != nil {
		return nil, err
	}
	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x83, Value: u8 | 0x04},
		{Reg: 0xFF, Value: 0x07},
		{Reg: 0x81, Value: 0x01},
	}...)
	if err != nil {
		return nil, err
	}

	err = v.writeRegU8(i2c, 0x80, 0x01)
	if err != nil {
		return nil, err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x94, Value: 0x6b},
		{Reg: 0x83, Value: 0x00},
	}...)
	if err != nil {
		return nil, err
	}
	err = v.waitUntilOrTimeout(i2c, 0x83,
		func(checkReg byte, err error) (bool, error) {
			return checkReg != 0, err
		})
	if err != nil {
		return nil, err
	}
	err = v.writeRegU8(i2c, 0x83, 0x01)
	if err != nil {
		return nil, err
	}
	tmp, err = v.readRegU8(i2c, 0x92)
	if err != nil {
		return nil, err
	}

	si := &SpadInfo{Count: tmp & 0x7F, TypeIsAperture: (tmp>>7)&0x01 != 0}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x81, Value: 0x00},
		{Reg: 0xFF, Value: 0x06},
	}...)
	if err != nil {
		return nil, err
	}
	u8, err = v.readRegU8(i2c, 0x83)
	if err != nil {
		return nil, err
	}
	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0x83, Value: u8 & ^byte(0x04)},
		{Reg: 0xFF, Value: 0x01},
		{Reg: 0x00, Value: 0x01},
	}...)
	if err != nil {
		return nil, err
	}

	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: 0xFF, Value: 0x00},
		{Reg: 0x80, Value: 0x00},
	}...)
	if err != nil {
		return nil, err
	}

	return si, nil
}

// Based on VL53L0X_perform_single_ref_calibration().
func (v *Vl53l0x) performSingleRefCalibration(i2c *i2c.I2C, vhvInitByte uint8) error {
	err := v.writeRegU8(i2c, SYSRANGE_START, 0x01|vhvInitByte) // VL53L0X_REG_SYSRANGE_MODE_START_STOP
	if err != nil {
		return err
	}
	err = v.waitUntilOrTimeout(i2c, RESULT_INTERRUPT_STATUS,
		func(checkReg byte, err error) (bool, error) {
			return checkReg&0x07 != 0, err
		})
	if err != nil {
		return err
	}
	err = v.writeRegValues(i2c, []RegBytePair{
		{Reg: SYSTEM_INTERRUPT_CLEAR, Value: 0x01},
		{Reg: SYSRANGE_START, Value: 0x00},
	}...)
	if err != nil {
		return err
	}
	return nil
}

// Set timeout duration for operations which could be
// terminated on timeout events.
func (v *Vl53l0x) setTimeout(timeout time.Duration) {
	v.ioTimeout = timeout
}

// Returns current time.
func (v *Vl53l0x) startTimeout() time.Time {
	return time.Now()
}

// Raise timeout event if execution time exceed value in Vl53l0x.ioTimeout.
func (v *Vl53l0x) checkTimeoutExpired(startTime time.Time) bool {
	left := time.Now().Sub(startTime)
	return v.ioTimeout > 0 && left > v.ioTimeout
}

// Read specific register in the loop until condition is true,
// or wait for timeout event.
func (v *Vl53l0x) waitUntilOrTimeout(i2c *i2c.I2C, reg byte,
	breakWhen func(chechReg byte, err error) (bool, error)) error {

	st := v.startTimeout()
	for {
		u8, err := v.readRegU8(i2c, reg)
		f, err2 := breakWhen(u8, err)
		if err2 != nil {
			return err2
		} else if f {
			break
		}
		if v.checkTimeoutExpired(st) {
			return errors.New(spew.Sprintf("timeout occurs; last read register 0x%x equal to 0x%x", reg, u8))
		}
	}
	return nil
}

// Write an 8-bit register.
func (v *Vl53l0x) writeRegU8(i2c *i2c.I2C, reg byte, value uint8) error {
	return i2c.WriteRegU8(reg, value)
}

// Write a 16-bit register.
func (v *Vl53l0x) writeRegU16(i2c *i2c.I2C, reg byte, value uint16) error {
	buf := []byte{reg, byte(value >> 8 & 0xFF), byte(value & 0xFF)}
	_, err := i2c.WriteBytes(buf)
	return err
}

// Write a 32-bit register.
func (v *Vl53l0x) writeRegU32(i2c *i2c.I2C, reg byte, value uint32) error {
	buf := []byte{reg, byte(value >> 24 & 0xFF), byte(value >> 16 & 0xFF),
		byte(value >> 8 & 0xFF), byte(value & 0xFF)}
	_, err := i2c.WriteBytes(buf)
	return err
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register.
func (v *Vl53l0x) writeBytes(i2c *i2c.I2C, reg byte, buf []byte) error {
	b := append([]byte{reg}, buf...)
	_, err := i2c.WriteBytes(b)
	return err
}

// Keeps pair of register and value to write to.
// Used as a bunch of registers which should be
// initialized with corresponding values.
type RegBytePair struct {
	Reg   byte
	Value uint8
}

// Write bunch of registers with with corresponding values.
func (v *Vl53l0x) writeRegValues(i2c *i2c.I2C, pairs ...RegBytePair) error {
	for _, pair := range pairs {
		err := v.writeRegU8(i2c, pair.Reg, pair.Value)
		if err != nil {
			return err
		}
	}
	return nil
}

// Read an 8-bit register.
func (v *Vl53l0x) readRegU8(i2c *i2c.I2C, reg byte) (uint8, error) {
	u8, err := i2c.ReadRegU8(reg)
	return u8, err
}

// Read a 16-bit register.
func (v *Vl53l0x) readRegU16(i2c *i2c.I2C, reg byte) (uint16, error) {
	_, err := i2c.WriteBytes([]byte{reg})
	if err != nil {
		return 0, err
	}
	var buf [2]byte
	_, err = i2c.ReadBytes(buf[0:])
	if err != nil {
		return 0, err
	}
	u16 := uint16(buf[0])<<8 | uint16(buf[1])
	return u16, nil
}

// Read a 32-bit register.
func (v *Vl53l0x) readRegU32(i2c *i2c.I2C, reg byte) (uint32, error) {
	_, err := i2c.WriteBytes([]byte{reg})
	if err != nil {
		return 0, err
	}
	var buf [4]byte
	_, err = i2c.ReadBytes(buf[0:])
	if err != nil {
		return 0, err
	}
	u32 := uint32(buf[0])<<24 | uint32(buf[1])<<16 |
		uint32(buf[2])<<8 | uint32(buf[3])
	return u32, nil
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array.
func (v *Vl53l0x) readRegBytes(i2c *i2c.I2C, reg byte, dest []byte) error {
	_, err := i2c.WriteBytes([]byte{reg})
	if err != nil {
		return err
	}
	_, err = i2c.ReadBytes(dest)
	return err
}
