use crate::{reg::Register, InitialisationError, Vl53l1x};

#[allow(dead_code)]
pub(crate) enum DistanceMode {
    Short,
    Medium,
    Long,
}

#[derive(Default)]
struct ResultBuffer {
    range_status: u8,
    // uint8_t report_status: not used
    stream_count: u8,
    dss_actual_effective_spads_sd0: u16,
    // uint16_t peak_signal_count_rate_mcps_sd0: not used
    ambient_count_rate_mcps_sd0: u16,
    // uint16_t sigma_sd0: not used
    // uint16_t phase_sd0: not used
    final_crosstalk_corrected_range_mm_sd0: u16,
    peak_signal_count_rate_crosstalk_corrected_mcps_sd0: u16,
}

#[derive(Default)]
#[repr(u8)]
enum RangeStatus {
    #[default]
    RangeValid = 0,

    // "sigma estimator check is above the internal defined threshold"
    // (sigma = standard deviation of measurement)
    SigmaFail = 1,

    // "signal value is below the internal defined threshold"
    SignalFail = 2,

    // "Target is below minimum detection threshold."
    RangeValidMinRangeClipped = 3,

    // "phase is out of bounds"
    // (nothing detected in range; try a longer distance mode if applicable)
    OutOfBoundsFail = 4,

    // "HW or VCSEL failure"
    HardwareFail = 5,

    // "The Range is valid but the wraparound check has not been done."
    RangeValidNoWrapCheckFail = 6,

    // "Wrapped target, not matching phases"
    // "no matching phase in other VCSEL period timing."
    WrapTargetFail = 7,

    // "Internal algo underflow or overflow in lite ranging."
    // ProcessingFail            =   8: not used in API

    // "Specific to lite ranging."
    // should never occur with this lib (which uses low power auto ranging,
    // as the API does)
    XtalkSignalFail = 9,

    // "1st interrupt when starting ranging in back to back mode. Ignore
    // data."
    // should never occur with this lib
    SynchronizationInt = 10, // (the API spells this "syncronisation")

    // "All Range ok but object is result of multiple pulses merging together.
    // Used by RQL for merged pulse detection"
    // RangeValid MergedPulse    =  11: not used in API

    // "Used by RQL as different to phase fail."
    // TargetPresentLackOfSignal =  12:

    // "Target is below minimum detection threshold."
    MinRangeFail = 13,

    // "The reported range is invalid"
    // RangeInvalid              =  14: can't actually be returned by API (range can never become
    // negative, even after correction)

    // "No Update."
    None = 255,
}

#[derive(Default)]
struct RangingData {
    range_mm: u16,
    range_status: RangeStatus,
    peak_signal_count_rate_mcps: f32,
    ambient_count_rate_mcps: f32,
}

const TARGET_RATE: u16 = 0x0A00;
const TIMING_GUARD: u32 = 4528;

impl<I2C, X, EI2C, EX> Vl53l1x<I2C, X>
where
    I2C: embedded_hal::i2c::I2c<Error = EI2C>,
    X: embedded_hal::digital::OutputPin<Error = EX>,
{
    pub(crate) fn init(
        &mut self,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        // check model ID and module type registers (values specified in datasheet)
        let model_id = self
            .read_u16(Register::IdentificationModelId as u16)
            .map_err(InitialisationError::I2C)?;
        if model_id != 0xEACC {
            return Err(InitialisationError::InvalidModelId(model_id));
        }

        // VL53L1_software_reset() begin

        self.write(Register::SoftReset as u16, 0x00)
            .map_err(InitialisationError::I2C)?;
        delay.delay_ms(1);
        self.write(Register::SoftReset as u16, 0x01)
            .map_err(InitialisationError::I2C)?;

        // give it some time to boot; otherwise the sensor NACKs during the self.read()
        // call below and the Arduino 101 doesn't seem to handle that well
        delay.delay_ms(1);

        // VL53L1_poll_for_boot_completion() begin

        // check last_status in case we still get a NACK to try to deal with it
        // correctly
        poll_timeout(delay, 500, || {
            Ok((self
                .read(Register::FirmwareSystemStatus as u16)
                .map_err(InitialisationError::I2C)?
                & 0x01)
                != 0)
        })?;

        // VL53L1_poll_for_boot_completion() end

        // VL53L1_software_reset() end

        // VL53L1_DataInit() begin

        // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode as it's the
        // default.

        self.update(Register::PadI2cHvExtsupConfig as u16, |data| {
            *data |= 0x01;
        })
        .map_err(InitialisationError::I2C)?;

        // store oscillator info for later use
        self.fast_osc_frequency = self
            .read_u16(Register::OscMeasuredFastOscFrequency as u16)
            .map_err(InitialisationError::I2C)?;
        self.osc_calibrate_val = self
            .read_u16(Register::ResultOscCalibrateVal as u16)
            .map_err(InitialisationError::I2C)?;

        // VL53L1_DataInit() end

        // VL53L1_StaticInit() begin

        // Note that the API does not actually apply the configuration settings below
        // when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
        // register contents in memory and doesn't actually write them until a
        // measurement is started. Writing the configuration here means we don't have
        // to keep it all in memory and avoids a lot of redundant writes later.

        // the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
        // VL53L1_set_preset_mode() begin

        // VL53L1_preset_mode_standard_ranging() begin

        // values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
        // (API uses these in VL53L1_init_tuning_parm_storage_struct())

        self.write_u16(Register::DssConfigTargetTotalRateMcps as u16, TARGET_RATE)
            .map_err(InitialisationError::I2C)?; // should already be this value after reset
        self.write(Register::GpioTioHvStatus as u16, 0x02)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::SigmaEstimatorEffectivePulseWidthNs as u16, 8)
            .map_err(InitialisationError::I2C)?; // tuning parm default
        self.write(Register::SigmaEstimatorEffectiveAmbientWidthNs as u16, 16)
            .map_err(InitialisationError::I2C)?; // tuning parm default
        self.write(
            Register::AlgoCrosstalkCompensationValidHeightMm as u16,
            0x01,
        )
        .map_err(InitialisationError::I2C)?;
        self.write(Register::AlgoRangeIgnoreValidHeightMm as u16, 0xFF)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::AlgoRangeMinClip as u16, 0)
            .map_err(InitialisationError::I2C)?; // tuning parm default
        self.write(Register::AlgoConsistencyCheckTolerance as u16, 2)
            .map_err(InitialisationError::I2C)?; // tuning parm default

        // general config
        self.write_u16(Register::SystemThreshRateHighHi as u16, 0x0000)
            .map_err(InitialisationError::I2C)?;
        self.write_u16(Register::SystemThreshRateHighLo as u16, 0x0000)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::DssConfigApertureAttenuation as u16, 0x38)
            .map_err(InitialisationError::I2C)?;

        // timing config
        // most of these settings will be determined later by distance and timing
        // budget configuration
        self.write_u16(Register::RangeConfigSigmaThresh as u16, 360)
            .map_err(InitialisationError::I2C)?; // tuning parm default
        self.write_u16(Register::RangeConfigMinCountRateRtnLimitMcps as u16, 192)
            .map_err(InitialisationError::I2C)?; // tuning parm default

        // dynamic config

        self.write(Register::SystemGroupedParameterHold0 as u16, 0x01)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::SystemGroupedParameterHold1 as u16, 0x01)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::SdConfigQuantifier as u16, 2)
            .map_err(InitialisationError::I2C)?; // tuning parm default

        // VL53L1_preset_mode_standard_ranging() end

        // from VL53L1_preset_mode_timed_ranging_*
        // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
        // and things don't seem to work if we don't set GPH back to 0 (which the API
        // does here).
        self.write(Register::SystemGroupedParameterHold as u16, 0x00)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::SystemSeedConfig as u16, 1)
            .map_err(InitialisationError::I2C)?; // tuning parm default

        // from VL53L1_config_low_power_auto_mode
        self.write(Register::SystemSequenceConfig as u16, 0x8B)
            .map_err(InitialisationError::I2C)?; // VHV, PHASECAL, DSS1, RANGE
        self.write_u16(
            Register::DssConfigManualEffectiveSpadsSelect as u16,
            200 << 8,
        )
        .map_err(InitialisationError::I2C)?;
        self.write(Register::DssConfigRoiModeControl as u16, 2)
            .map_err(InitialisationError::I2C)?; // REQUESTED_EFFFECTIVE_SPADS

        // VL53L1_set_preset_mode() end

        // default to long range, 50 ms timing budget
        // note that this is different than what the API defaults to
        self.set_distance_mode(DistanceMode::Long)?;
        self.set_measurement_timing_budget(50000)?;

        // VL53L1_StaticInit() end

        // the API triggers this change in VL53L1_init_and_start_range() once a
        // measurement is started; assumes MM1 and MM2 are disabled
        let range_offset_mm = self
            .read_u16(Register::MmConfigOuterOffsetMm as u16)
            .map_err(InitialisationError::I2C)?
            * 4;
        self.write_u16(
            Register::AlgoPartToPartRangeOffsetMm as u16,
            range_offset_mm,
        )
        .map_err(InitialisationError::I2C)?;

        Ok(())
    }

    pub(crate) fn set_address(&mut self, new_addr: u8) -> Result<(), EI2C> {
        self.write(Register::I2cSlaveDeviceAddress as u16, new_addr & 0x7F)?;
        self.address = new_addr;
        Ok(())
    }

    pub(crate) fn start_continuous(&mut self, period_ms: u32) -> Result<(), EI2C> {
        // from VL53L1_set_inter_measurement_period_ms()
        self.write_u32(
            Register::SystemIntermeasurementPeriod as u16,
            period_ms * u32::from(self.osc_calibrate_val),
        )?;

        self.write(Register::SystemInterruptClear as u16, 0x01)?; // sys_interrupt_clear_range
        self.write(Register::SystemModeStart as u16, 0x40)?; // mode_range__timed

        Ok(())
    }

    pub(crate) fn try_read_inner(&mut self) -> Result<Option<u16>, EI2C> {
        if !self.data_ready()? {
            return Ok(None);
        }

        let results = self.read_results()?;

        if !self.calibrated {
            self.setup_manual_calibration()?;
            self.calibrated = true;
        }

        self.update_dss(&results)?;

        let ranging_data = get_ranging_data(&results);

        self.write(Register::SystemInterruptClear as u16, 0x01)?; // sys_interrupt_clear_range

        Ok(Some(ranging_data.range_mm))
    }

    fn setup_manual_calibration(&mut self) -> Result<(), EI2C> {
        // "save original vhv configs"
        self.saved_vhv_init = self.read(Register::VhvConfigInit as u16)?;
        self.saved_vhv_timeout = self.read(Register::VhvConfigTimeoutMacropLoopBound as u16)?;

        // "disable VHV init"
        self.write(Register::VhvConfigInit as u16, self.saved_vhv_init & 0x7F)?;

        // "set loop bound to tuning param"
        self.write(
            Register::VhvConfigTimeoutMacropLoopBound as u16,
            (self.saved_vhv_timeout & 0x03) + (3 << 2),
        )?; // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

        // "override phasecal"
        self.write(Register::PhasecalConfigOverride as u16, 0x01)?;
        let phasecal_result = self.read(Register::PhasecalResultVcselStart as u16)?;
        self.write(Register::CalConfigVcselStart as u16, phasecal_result)?;

        Ok(())
    }

    fn update_dss(&mut self, results: &ResultBuffer) -> Result<(), EI2C> {
        let spad_count: u16 = results.dss_actual_effective_spads_sd0;

        if spad_count != 0 {
            // "Calc total rate per spad"

            let mut total_rate_per_spad: u32 =
                u32::from(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0)
                    + u32::from(results.ambient_count_rate_mcps_sd0);

            // "clip to 16 bits"
            if total_rate_per_spad > 0xFFFF {
                total_rate_per_spad = 0xFFFF;
            }

            // "shift up to take advantage of 32 bits"
            total_rate_per_spad <<= 16;

            total_rate_per_spad /= u32::from(spad_count);

            if total_rate_per_spad != 0 {
                // "get the target rate and shift up by 16"
                let mut required_spads: u32 = (u32::from(TARGET_RATE) << 16) / total_rate_per_spad;

                // "clip to 16 bit"
                if required_spads > 0xFFFF {
                    required_spads = 0xFFFF;
                }

                // "override DSS config"
                #[allow(clippy::cast_possible_truncation)]
                self.write_u16(
                    Register::DssConfigManualEffectiveSpadsSelect as u16,
                    required_spads as u16,
                )?;
                // DSS_CONFIG__ROI_MODE_CONTROL should already be set to
                // REQUESTED_EFFFECTIVE_SPADS

                return Ok(());
            }
        }

        // If we reached this point, it means something above would have resulted in a
        // divide by zero.
        // "We want to gracefully set a spad target, not just exit with an error"

        // "set target to mid point"
        self.write_u16(Register::DssConfigManualEffectiveSpadsSelect as u16, 0x8000)?;

        Ok(())
    }

    fn data_ready(&mut self) -> Result<bool, EI2C> {
        Ok((self.read(Register::GpioTioHvStatus as u16)? & 0x01) == 0)
    }

    fn read_results(&mut self) -> Result<ResultBuffer, EI2C> {
        let mut results = ResultBuffer::default();

        let mut data: [u8; 17] = [0; 17];
        self.i2c.transaction(
            self.address,
            &mut [
                embedded_hal::i2c::Operation::Write(&[
                    (Register::ResultRangeStatus as u16 >> 8) as u8,
                    Register::ResultRangeStatus as u8,
                ]),
                embedded_hal::i2c::Operation::Read(&mut data),
            ],
        )?;

        results.range_status = data[0];
        // results.range_status = bus.read();

        _ = data[1];
        // bus->read(); // report_status: not used

        results.stream_count = data[2];
        // results.stream_count = bus.read();

        results.dss_actual_effective_spads_sd0 = u16::from(data[3]) << 8 | u16::from(data[4]);
        // results.dss_actual_effective_spads_sd0  = (uint16_t)bus.read() << 8; // high
        // byte results.dss_actual_effective_spads_sd0 |=           bus.read();
        // // low byte

        _ = data[5];
        // bus.read(); // peak_signal_count_rate_mcps_sd0: not used
        _ = data[6];
        // bus.read();

        results.ambient_count_rate_mcps_sd0 = u16::from(data[7]) << 8 | u16::from(data[8]);
        // results.ambient_count_rate_mcps_sd0  = (uint16_t)bus.read() << 8; // high
        // byte results.ambient_count_rate_mcps_sd0 |=           bus.read();
        // // low byte

        _ = data[9];
        // bus.read(); // sigma_sd0: not used
        _ = data[10];
        // bus.read();

        _ = data[11];
        // bus.read(); // phase_sd0: not used
        _ = data[12];
        // bus.read();

        results.final_crosstalk_corrected_range_mm_sd0 =
            u16::from(data[13]) << 8 | u16::from(data[14]);
        // results.final_crosstalk_corrected_range_mm_sd0  = (uint16_t)bus.read() << 8;
        // // high byte results.final_crosstalk_corrected_range_mm_sd0 |=
        // bus.read();      // low byte

        results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
            u16::from(data[15]) << 8 | u16::from(data[16]);
        // results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0  =
        // (uint16_t)bus.read() << 8; // high byte
        // results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 |=
        // bus.read();      // low byte

        Ok(results)
    }

    fn set_distance_mode(
        &mut self,
        mode: DistanceMode,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        // save existing timing budget
        let budget_us: u32 = self
            .get_measurement_timing_budget()
            .map_err(InitialisationError::I2C)?;

        match mode {
            DistanceMode::Short => {
                // from VL53L1_preset_mode_standard_ranging_short_range()

                // timing config
                self.write(Register::RangeConfigVcselPeriodA as u16, 0x07)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::RangeConfigVcselPeriodB as u16, 0x05)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::RangeConfigValidPhaseHigh as u16, 0x38)
                    .map_err(InitialisationError::I2C)?;

                // dynamic config
                self.write(Register::SdConfigWoiSd0 as u16, 0x07)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::SdConfigWoiSd1 as u16, 0x05)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::SdConfigInitialPhaseSd0 as u16, 6)
                    .map_err(InitialisationError::I2C)?; // tuning parm default
                self.write(Register::SdConfigInitialPhaseSd1 as u16, 6)
                    .map_err(InitialisationError::I2C)?; // tuning parm
                                                         // default
            }

            DistanceMode::Medium => {
                // from VL53L1_preset_mode_standard_ranging()

                // timing config
                self.write(Register::RangeConfigVcselPeriodA as u16, 0x0B)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::RangeConfigVcselPeriodB as u16, 0x09)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::RangeConfigValidPhaseHigh as u16, 0x78)
                    .map_err(InitialisationError::I2C)?;

                // dynamic config
                self.write(Register::SdConfigWoiSd0 as u16, 0x0B)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::SdConfigWoiSd1 as u16, 0x09)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::SdConfigInitialPhaseSd0 as u16, 10)
                    .map_err(InitialisationError::I2C)?; // tuning parm default
                self.write(Register::SdConfigInitialPhaseSd1 as u16, 10)
                    .map_err(InitialisationError::I2C)?; // tuning parm
                                                         // default
            }

            DistanceMode::Long => {
                // long
                // from VL53L1_preset_mode_standard_ranging_long_range()

                // timing config
                self.write(Register::RangeConfigVcselPeriodA as u16, 0x0F)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::RangeConfigVcselPeriodB as u16, 0x0D)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::RangeConfigValidPhaseHigh as u16, 0xB8)
                    .map_err(InitialisationError::I2C)?;

                // dynamic config
                self.write(Register::SdConfigWoiSd0 as u16, 0x0F)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::SdConfigWoiSd1 as u16, 0x0D)
                    .map_err(InitialisationError::I2C)?;
                self.write(Register::SdConfigInitialPhaseSd0 as u16, 14)
                    .map_err(InitialisationError::I2C)?; // tuning parm default
                self.write(Register::SdConfigInitialPhaseSd1 as u16, 14)
                    .map_err(InitialisationError::I2C)?; // tuning parm
                                                         // default
            }
        }

        // reapply timing budget
        self.set_measurement_timing_budget(budget_us)?;

        // save mode so it can be returned by getDistanceMode()
        self.distance_mode = mode;

        Ok(())
    }

    fn get_measurement_timing_budget(&mut self) -> Result<u32, EI2C> {
        // assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
        // enabled: VHV, PHASECAL, DSS1, RANGE

        // VL53L1_get_timeouts_us() begin

        // "Update Macro Period for Range A VCSEL Period"
        let period = self.read(Register::RangeConfigVcselPeriodA as u16)?;
        let macro_period_us: u32 = self.calc_macro_period(period);

        // "Get Range Timing A timeout"

        let range_config_timeout_us: u32 = timeout_mclks_to_microseconds(
            decode_timeout(self.read_u16(Register::RangeConfigTimeoutMacropA as u16)?),
            macro_period_us,
        );

        // VL53L1_get_timeouts_us() end

        Ok(2 * range_config_timeout_us + TIMING_GUARD)
    }

    fn set_measurement_timing_budget(
        &mut self,
        mut budget_us: u32,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        // assumes PresetMode is LOWPOWER_AUTONOMOUS

        if budget_us <= TIMING_GUARD {
            return Err(InitialisationError::InvalidTimingBudget);
        }

        budget_us -= TIMING_GUARD;
        let mut range_config_timeout_us: u32 = budget_us;
        if range_config_timeout_us > 1_100_000 {
            return Err(InitialisationError::InvalidTimingBudget);
        } // FDA_MAX_TIMING_BUDGET_US * 2

        range_config_timeout_us /= 2;

        // VL53L1_calc_timeout_register_values() begin

        let mut macro_period_us: u32;

        // "Update Macro Period for Range A VCSEL Period"
        let period = self
            .read(Register::RangeConfigVcselPeriodA as u16)
            .map_err(InitialisationError::I2C)?;
        macro_period_us = self.calc_macro_period(period);

        // "Update Phase timeout - uses Timing A"
        // Timeout of 1000 is tuning parm default
        // (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
        // via VL53L1_get_preset_mode_timing_cfg().
        let mut phasecal_timeout_mclks: u32 = timeout_microseconds_to_mclks(1000, macro_period_us);
        if phasecal_timeout_mclks > 0xFF {
            phasecal_timeout_mclks = 0xFF;
        }
        #[allow(clippy::cast_possible_truncation)]
        self.write(
            Register::PhasecalConfigTimeoutMacrop as u16,
            phasecal_timeout_mclks as u8,
        )
        .map_err(InitialisationError::I2C)?;

        // "Update MM Timing A timeout"
        // Timeout of 1 is tuning parm default
        // (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
        // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
        // actually ends up with a slightly different value because it gets assigned,
        // retrieved, recalculated with a different macro period, and reassigned,
        // but it probably doesn't matter because it seems like the MM ("mode
        // mitigation"?) sequence steps are disabled in low power auto mode anyway.
        self.write_u16(
            Register::MmConfigTimeoutMacropA as u16,
            encode_timeout(timeout_microseconds_to_mclks(1, macro_period_us)),
        )
        .map_err(InitialisationError::I2C)?;

        // "Update Range Timing A timeout"
        self.write_u16(
            Register::RangeConfigTimeoutMacropA as u16,
            encode_timeout(timeout_microseconds_to_mclks(
                range_config_timeout_us,
                macro_period_us,
            )),
        )
        .map_err(InitialisationError::I2C)?;

        // "Update Macro Period for Range B VCSEL Period"
        let period = self
            .read(Register::RangeConfigVcselPeriodB as u16)
            .map_err(InitialisationError::I2C)?;
        macro_period_us = self.calc_macro_period(period);

        // "Update MM Timing B timeout"
        // (See earlier comment about MM Timing A timeout.)
        self.write_u16(
            Register::MmConfigTimeoutMacropB as u16,
            encode_timeout(timeout_microseconds_to_mclks(1, macro_period_us)),
        )
        .map_err(InitialisationError::I2C)?;

        // "Update Range Timing B timeout"
        self.write_u16(
            Register::RangeConfigTimeoutMacropB as u16,
            encode_timeout(timeout_microseconds_to_mclks(
                range_config_timeout_us,
                macro_period_us,
            )),
        )
        .map_err(InitialisationError::I2C)?;

        // VL53L1_calc_timeout_register_values() end

        Ok(())
    }

    // Calculate macro period in microseconds (12.12 format) with given VCSEL period
    // assumes fast_osc_frequency has been read and stored
    // based on VL53L1_calc_macro_period_us()
    fn calc_macro_period(&self, vcsel_period: u8) -> u32 {
        // from VL53L1_calc_pll_period_us()
        // fast osc frequency in 4.12 format; PLL period in 0.24 format
        let pll_period_us: u32 = (0x01 << 30) / u32::from(self.fast_osc_frequency);

        // from VL53L1_decode_vcsel_period()
        let vcsel_period_pclks: u8 = (vcsel_period + 1) << 1;

        // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
        let mut macro_period_us: u32 = 2304_u32 * pll_period_us;
        macro_period_us >>= 6;
        macro_period_us *= u32::from(vcsel_period_pclks);
        macro_period_us >>= 6;

        macro_period_us
    }
}

impl<I2C, X, EI2C, EX> Vl53l1x<I2C, X>
where
    I2C: embedded_hal::i2c::I2c<Error = EI2C>,
    X: embedded_hal::digital::OutputPin<Error = EX>,
{
    fn read(&mut self, register: u16) -> Result<u8, EI2C> {
        let mut data = [0];
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write_read(
            self.address,
            &[(register >> 8) as u8, register as u8],
            &mut data,
        )?;
        Ok(data[0])
    }

    fn read_u16(&mut self, register: u16) -> Result<u16, EI2C> {
        let mut data = [0; 2];
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write_read(
            self.address,
            &[(register >> 8) as u8, register as u8],
            &mut data,
        )?;
        Ok(u16::from(data[0]) << 8 | u16::from(data[1]))
    }

    fn write(&mut self, register: u16, data: u8) -> Result<(), EI2C> {
        #[allow(clippy::cast_possible_truncation)]
        self.i2c
            .write(self.address, &[(register >> 8) as u8, register as u8, data])
    }

    fn write_u16(&mut self, register: u16, data: u16) -> Result<(), EI2C> {
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write(
            self.address,
            &[
                (register >> 8) as u8,
                register as u8,
                (data >> 8) as u8,
                data as u8,
            ],
        )
    }

    fn write_u32(&mut self, register: u16, data: u32) -> Result<(), EI2C> {
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write(
            self.address,
            &[
                (register >> 8) as u8,
                register as u8,
                (data >> 24) as u8,
                (data >> 16) as u8,
                (data >> 8) as u8,
                data as u8,
            ],
        )
    }

    fn update(&mut self, register: u16, f: impl FnOnce(&mut u8)) -> Result<(), EI2C> {
        let mut data = [0];
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write_read(
            self.address,
            &[(register >> 8) as u8, register as u8],
            &mut data,
        )?;
        f(&mut data[0]);
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write(
            self.address,
            &[(register >> 8) as u8, register as u8, data[0]],
        )
    }
}

#[allow(clippy::cast_possible_truncation)]
fn timeout_mclks_to_microseconds(timeout_mclks: u32, macro_period_us: u32) -> u32 {
    ((u64::from(timeout_mclks) * u64::from(macro_period_us) + 0x800) >> 12) as u32
}

fn timeout_microseconds_to_mclks(timeout_us: u32, macro_period_us: u32) -> u32 {
    ((timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us
}

fn decode_timeout(reg_val: u16) -> u32 {
    ((u32::from(reg_val & 0xFF)) << (reg_val >> 8)) + 1
}

fn encode_timeout(timeout_mclks: u32) -> u16 {
    // encoded format: "(LSByte * 2^MSByte) + 1"

    let mut ls_byte: u32;
    let mut ms_byte: u16 = 0;

    if timeout_mclks > 0 {
        ls_byte = timeout_mclks - 1;

        while (ls_byte & 0xFFFF_FF00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }

        (ms_byte << 8) | ((ls_byte & 0xFF) as u16)
    } else {
        0
    }
}

fn count_rate_fixed_to_float(count_rate_fixed: u16) -> f32 {
    f32::from(count_rate_fixed) / f32::from(1_u16 << 7_u16)
}

#[allow(clippy::cast_possible_truncation)]
fn get_ranging_data(results: &ResultBuffer) -> RangingData {
    let mut ranging_data = RangingData::default();
    // VL53L1_copy_sys_and_core_results_to_range_results() begin

    let range: u16 = results.final_crosstalk_corrected_range_mm_sd0;

    // "apply correction gain"
    // gain factor of 2011 is tuning parm default
    // (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT) Basically, this
    // appears to scale the result by 2011/2048, or about 98% (with the 1024
    // added for proper rounding).
    ranging_data.range_mm = ((u32::from(range) * 2011 + 0x0400) / 0x0800) as u16;

    // VL53L1_copy_sys_and_core_results_to_range_results() end

    // set range_status in ranging_data based on value of RESULT__RANGE_STATUS
    // register mostly based on ConvertStatusLite()
    match results.range_status {
        17 | 2 | 1 | 3 => {
            // MULTCLIPFAIL
            // VCSELWATCHDOGTESTFAILURE
            // VCSELCONTINUITYTESTFAILURE
            // NOVHVVALUEFOUND
            // from SetSimpleData()
            ranging_data.range_status = RangeStatus::HardwareFail;
        }

        13 => {
            // USERROICLIP
            // from SetSimpleData()
            ranging_data.range_status = RangeStatus::MinRangeFail;
        }

        18 => {
            // GPHSTREAMCOUNT0READY
            ranging_data.range_status = RangeStatus::SynchronizationInt;
        }

        5 => {
            // RANGEPHASECHECK
            ranging_data.range_status = RangeStatus::OutOfBoundsFail;
        }

        4 => {
            // MSRCNOTARGET
            ranging_data.range_status = RangeStatus::SignalFail;
        }

        6 => {
            // SIGMATHRESHOLDCHECK
            ranging_data.range_status = RangeStatus::SigmaFail;
        }

        7 => {
            // PHASECONSISTENCY
            ranging_data.range_status = RangeStatus::WrapTargetFail;
        }

        12 => {
            // RANGEIGNORETHRESHOLD
            ranging_data.range_status = RangeStatus::XtalkSignalFail;
        }

        8 => {
            // MINCLIP
            ranging_data.range_status = RangeStatus::RangeValidMinRangeClipped;
        }

        9 => {
            // RANGECOMPLETE
            // from VL53L1_copy_sys_and_core_results_to_range_results()
            if results.stream_count == 0 {
                ranging_data.range_status = RangeStatus::RangeValidNoWrapCheckFail;
            } else {
                ranging_data.range_status = RangeStatus::RangeValid;
            }
        }

        _ => {
            ranging_data.range_status = RangeStatus::None;
        }
    }

    // from SetSimpleData()
    ranging_data.peak_signal_count_rate_mcps =
        count_rate_fixed_to_float(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
    ranging_data.ambient_count_rate_mcps =
        count_rate_fixed_to_float(results.ambient_count_rate_mcps_sd0);

    ranging_data
}

/// Poll a function until it returns true or a timeout is reached.
fn poll_timeout<EI2C, EX>(
    delay: &mut impl embedded_hal::delay::DelayNs,
    timeout_ms: u32,
    mut f: impl FnMut() -> Result<bool, InitialisationError<EI2C, EX>>,
) -> Result<(), InitialisationError<EI2C, EX>> {
    let mut counter = 0;
    loop {
        if f()? {
            return Ok(());
        }

        if counter >= timeout_ms {
            return Err(InitialisationError::Timeout);
        }

        delay.delay_ms(1);
        counter += 1;
    }
}
