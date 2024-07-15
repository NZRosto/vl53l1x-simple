use crate::{reg::Register, ClockSource, InitialisationError, Vl53l0x};

#[allow(clippy::struct_excessive_bools)]
#[derive(Default, Clone, Copy)]
struct SequenceStepEnables {
    tcc: bool,
    msrc: bool,
    dss: bool,
    pre_range: bool,
    final_range: bool,
}

#[derive(Default, Clone, Copy)]
struct SequenceStepTimeouts {
    pre_range_vcsel_period_pclks: u16,
    final_range_vcsel_period_pclks: u16,
    msrc_dss_tcc_mclks: u16,
    pre_range_mclks: u16,
    final_range_mclks: u16,
    msrc_dss_tcc_us: u32,
    pre_range_us: u32,
    final_range_us: u32,
}

#[derive(Clone, Copy)]
enum VcselPeriodType {
    VcselPeriodPreRange,
    VcselPeriodFinalRange,
}

impl<I2C, X, EI2C, EX> Vl53l0x<I2C, X>
where
    I2C: embedded_hal::i2c::I2c<Error = EI2C>,
    X: embedded_hal::digital::OutputPin<Error = EX>,
{
    #[allow(clippy::too_many_lines)]
    pub(crate) fn init(
        &mut self,
        timer: &impl ClockSource,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        // check model ID register (value specified in datasheet)
        let model_id = self
            .read(Register::IdentificationModelId as u8)
            .map_err(InitialisationError::I2C)?;
        if model_id != 0xEE {
            return Err(InitialisationError::InvalidModelId(model_id));
        }

        // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode as it's default
        // in the Arduino libraries
        self.update(Register::VhvConfigPadSclSdaExtsupHv as u8, |data| {
            *data |= 0x01;
        })
        .map_err(InitialisationError::I2C)?; // set bit 0

        // "Set I2C standard mode"
        self.write(0x88, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0x80, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x00, 0x00).map_err(InitialisationError::I2C)?;
        self.stop_variable = self.read(0x91).map_err(InitialisationError::I2C)?;
        self.write(0x00, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x80, 0x00).map_err(InitialisationError::I2C)?;

        // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit
        // checks
        self.update(Register::MsrcConfigControl as u8, |data| *data |= 0x12)
            .map_err(InitialisationError::I2C)?;

        // set final range signal rate limit to 0.25 MCPS (million counts per second)
        self.set_signal_rate_limit(0.25)?;

        self.write(Register::SystemSequenceConfig as u8, 0xFF)
            .map_err(InitialisationError::I2C)?;

        // VL53L0X_DataInit() end

        // VL53L0X_StaticInit() begin

        let (spad_count, spad_type_is_aperture) = self.get_spad_info(timer)?;

        // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
        // the API, but the same data seems to be more easily readable from
        // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
        let mut ref_spad_map = [0; 6];
        self.read_many(
            Register::GlobalConfigSpadEnablesRef0 as u8,
            &mut ref_spad_map,
        )
        .map_err(InitialisationError::I2C)?;

        // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(Register::DynamicSpadRefEnStartOffset as u8, 0x00)
            .map_err(InitialisationError::I2C)?;
        self.write(Register::DynamicSpadNumRequestedRefSpad as u8, 0x2C)
            .map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(Register::GlobalConfigRefEnStartSelect as u8, 0xB4)
            .map_err(InitialisationError::I2C)?;

        let first_spad_to_enable: u8 = if spad_type_is_aperture { 12 } else { 0 }; // 12 is the first aperture spad
        let mut spads_enabled: u8 = 0;

        for i in 0..48 {
            if i < first_spad_to_enable || spads_enabled == spad_count {
                // This bit is lower than the first one that should be enabled, or
                // (reference_spad_count) bits have already been enabled, so zero this bit
                ref_spad_map[(i / 8) as usize] &= !(1 << (i % 8));
            } else if (ref_spad_map[(i / 8) as usize] >> (i % 8)) & 0x1 != 0 {
                spads_enabled += 1;
            }
        }

        self.write_many(Register::GlobalConfigSpadEnablesRef0 as u8, &ref_spad_map)
            .map_err(InitialisationError::I2C)?;

        // -- VL53L0X_set_reference_spads() end

        // -- VL53L0X_load_tuning_settings() begin
        // DefaultTuningSettings from vl53l0x_tuning.h

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x00, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x09, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x10, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x11, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0x24, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x25, 0xFF).map_err(InitialisationError::I2C)?;
        self.write(0x75, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x4E, 0x2C).map_err(InitialisationError::I2C)?;
        self.write(0x48, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x30, 0x20).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x30, 0x09).map_err(InitialisationError::I2C)?;
        self.write(0x54, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x31, 0x04).map_err(InitialisationError::I2C)?;
        self.write(0x32, 0x03).map_err(InitialisationError::I2C)?;
        self.write(0x40, 0x83).map_err(InitialisationError::I2C)?;
        self.write(0x46, 0x25).map_err(InitialisationError::I2C)?;
        self.write(0x60, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x27, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x50, 0x06).map_err(InitialisationError::I2C)?;
        self.write(0x51, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x52, 0x96).map_err(InitialisationError::I2C)?;
        self.write(0x56, 0x08).map_err(InitialisationError::I2C)?;
        self.write(0x57, 0x30).map_err(InitialisationError::I2C)?;
        self.write(0x61, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x62, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x64, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x65, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x66, 0xA0).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x22, 0x32).map_err(InitialisationError::I2C)?;
        self.write(0x47, 0x14).map_err(InitialisationError::I2C)?;
        self.write(0x49, 0xFF).map_err(InitialisationError::I2C)?;
        self.write(0x4A, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x7A, 0x0A).map_err(InitialisationError::I2C)?;
        self.write(0x7B, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x78, 0x21).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x23, 0x34).map_err(InitialisationError::I2C)?;
        self.write(0x42, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x44, 0xFF).map_err(InitialisationError::I2C)?;
        self.write(0x45, 0x26).map_err(InitialisationError::I2C)?;
        self.write(0x46, 0x05).map_err(InitialisationError::I2C)?;
        self.write(0x40, 0x40).map_err(InitialisationError::I2C)?;
        self.write(0x0E, 0x06).map_err(InitialisationError::I2C)?;
        self.write(0x20, 0x1A).map_err(InitialisationError::I2C)?;
        self.write(0x43, 0x40).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x34, 0x03).map_err(InitialisationError::I2C)?;
        self.write(0x35, 0x44).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x31, 0x04).map_err(InitialisationError::I2C)?;
        self.write(0x4B, 0x09).map_err(InitialisationError::I2C)?;
        self.write(0x4C, 0x05).map_err(InitialisationError::I2C)?;
        self.write(0x4D, 0x04).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x44, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x45, 0x20).map_err(InitialisationError::I2C)?;
        self.write(0x47, 0x08).map_err(InitialisationError::I2C)?;
        self.write(0x48, 0x28).map_err(InitialisationError::I2C)?;
        self.write(0x67, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x70, 0x04).map_err(InitialisationError::I2C)?;
        self.write(0x71, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x72, 0xFE).map_err(InitialisationError::I2C)?;
        self.write(0x76, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x77, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x0D, 0x01).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x80, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x01, 0xF8).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x8E, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x00, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x80, 0x00).map_err(InitialisationError::I2C)?;

        // -- VL53L0X_load_tuning_settings() end

        // "Set interrupt config to new sample ready"
        // -- VL53L0X_SetGpioConfig() begin

        self.write(Register::SystemInterruptConfigGpio as u8, 0x04)
            .map_err(InitialisationError::I2C)?;
        self.update(Register::GpioHvMuxActiveHigh as u8, |data| {
            *data &= !0x10;
        })
        .map_err(InitialisationError::I2C)?; // active low
        self.write(Register::SystemInterruptClear as u8, 0x01)
            .map_err(InitialisationError::I2C)?;

        // -- VL53L0X_SetGpioConfig() end

        self.measurement_timing_budget_us = self
            .get_measurement_timing_budget()
            .map_err(InitialisationError::I2C)?;

        // "Disable MSRC and TCC by default"
        // MSRC = Minimum Signal Rate Check
        // TCC = Target CentreCheck
        // -- VL53L0X_SetSequenceStepEnable() begin

        self.write(Register::SystemSequenceConfig as u8, 0xE8)
            .map_err(InitialisationError::I2C)?;

        // -- VL53L0X_SetSequenceStepEnable() end

        // "Recalculate timing budget"
        self.set_measurement_timing_budget(self.measurement_timing_budget_us)?;

        // VL53L0X_StaticInit() end

        // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

        // -- VL53L0X_perform_vhv_calibration() begin

        self.write(Register::SystemSequenceConfig as u8, 0x01)
            .map_err(InitialisationError::I2C)?;
        self.perform_single_ref_calibration(0x40, timer)?;

        // -- VL53L0X_perform_vhv_calibration() end

        // -- VL53L0X_perform_phase_calibration() begin

        self.write(Register::SystemSequenceConfig as u8, 0x02)
            .map_err(InitialisationError::I2C)?;
        self.perform_single_ref_calibration(0x00, timer)?;

        // -- VL53L0X_perform_phase_calibration() end

        // "restore the previous Sequence Config"
        self.write(Register::SystemSequenceConfig as u8, 0xE8)
            .map_err(InitialisationError::I2C)?;

        // VL53L0X_PerformRefCalibration() end

        Ok(())
    }

    pub(crate) fn set_address(&mut self, new_addr: u8) -> Result<(), EI2C> {
        self.write(Register::I2cSlaveDeviceAddress as u8, new_addr & 0x7F)?;
        self.address = new_addr;
        Ok(())
    }

    // Start continuous ranging measurements. If period_ms (optional) is 0 or not
    // given, continuous back-to-back mode is used (the sensor takes measurements as
    // often as possible); otherwise, continuous timed mode is used, with the given
    // inter-measurement period in milliseconds determining how often the sensor
    // takes a measurement.
    // based on VL53L0X_StartMeasurement()
    pub(crate) fn start_continuous(&mut self, mut period_ms: u32) -> Result<(), EI2C> {
        self.write(0x80, 0x01)?;
        self.write(0xFF, 0x01)?;
        self.write(0x00, 0x00)?;
        self.write(0x91, self.stop_variable)?;
        self.write(0x00, 0x01)?;
        self.write(0xFF, 0x00)?;
        self.write(0x80, 0x00)?;

        if period_ms != 0 {
            // continuous timed mode

            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

            let osc_calibrate_val: u16 = self.read_u16(Register::OscCalibrateVal as u8)?;

            if osc_calibrate_val != 0 {
                period_ms *= u32::from(osc_calibrate_val);
            }

            self.write_u32(Register::SystemIntermeasurementPeriod as u8, period_ms)?;

            // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

            self.write(Register::SysrangeStart as u8, 0x04)?; // VL53L0X_REG_SYSRANGE_MODE_TIMED
        } else {
            // continuous back-to-back mode
            self.write(Register::SysrangeStart as u8, 0x02)?; // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
        }

        Ok(())
    }

    pub(crate) fn try_read_range_continuous_millimeters(&mut self) -> Result<Option<u16>, EI2C> {
        #[allow(clippy::verbose_bit_mask)]
        if (self.read(Register::ResultInterruptStatus as u8)? & 0x07) == 0 {
            return Ok(None);
        }

        // assumptions: Linearity Corrective Gain is 1000 (default);
        // fractional ranging is not enabled
        let range: u16 = self.read_u16(Register::ResultRangeStatus as u8 + 10)?;

        self.write(Register::SystemInterruptClear as u8, 0x01)?;

        Ok(Some(range))
    }

    // Set the return signal rate limit check value in units of MCPS (mega counts
    // per second). "This represents the amplitude of the signal reflected from the
    // target and detected by the device"; setting this limit presumably determines
    // the minimum measurement necessary for the sensor to report a valid reading.
    // Setting a lower limit increases the potential range of the sensor but also
    // seems to increase the likelihood of getting an inaccurate reading because of
    // unwanted reflections from objects other than the intended target.
    // Defaults to 0.25 MCPS as initialized by the ST API and this library.
    fn set_signal_rate_limit(
        &mut self,
        limit_mcps: f32,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        if !(0.0..=511.99).contains(&limit_mcps) {
            return Err(InitialisationError::InvalidSignalRateLimit(limit_mcps));
        }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        //
        // This is absolutely horrible, this couldn't have been written
        // more confusingly by the API devs if they tried. Hopefully this works?.
        // Just look at clippy complain...
        #[allow(clippy::cast_possible_truncation)]
        #[allow(clippy::cast_precision_loss)]
        #[allow(clippy::cast_sign_loss)]
        self.write_u16(
            Register::FinalRangeConfigMinCountRateRtnLimit as u8,
            (limit_mcps * ((1_u32 << 7_u32) as f32)) as u16,
        )
        .map_err(InitialisationError::I2C)
    }

    fn get_spad_info(
        &mut self,
        timer: &impl ClockSource,
    ) -> Result<(u8, bool), InitialisationError<EI2C, EX>> {
        self.write(0x80, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x00, 0x00).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x06).map_err(InitialisationError::I2C)?;
        self.update(0x83, |data| *data |= 0x04)
            .map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x07).map_err(InitialisationError::I2C)?;
        self.write(0x81, 0x01).map_err(InitialisationError::I2C)?;

        self.write(0x80, 0x01).map_err(InitialisationError::I2C)?;

        self.write(0x94, 0x6b).map_err(InitialisationError::I2C)?;
        self.write(0x83, 0x00).map_err(InitialisationError::I2C)?;
        let start_time: u32 = timer.get_ms();
        while self.read(0x83).map_err(InitialisationError::I2C)? == 0x00 {
            if (timer.get_ms() - start_time) > 500 {
                // 500 ms
                return Err(InitialisationError::CalibrationTimeout);
            }
        }
        self.write(0x83, 0x01).map_err(InitialisationError::I2C)?;
        let tmp = self.read(0x92).map_err(InitialisationError::I2C)?;

        let count = tmp & 0x7f;
        let type_is_aperture = ((tmp >> 7) & 0x01) != 0;

        self.write(0x81, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x06).map_err(InitialisationError::I2C)?;
        self.update(0x83, |data| *data &= !0x04)
            .map_err(InitialisationError::I2C)?;
        self.write(0xFF, 0x01).map_err(InitialisationError::I2C)?;
        self.write(0x00, 0x01).map_err(InitialisationError::I2C)?;

        self.write(0xFF, 0x00).map_err(InitialisationError::I2C)?;
        self.write(0x80, 0x00).map_err(InitialisationError::I2C)?;

        Ok((count, type_is_aperture))
    }

    fn get_measurement_timing_budget(&mut self) -> Result<u32, EI2C> {
        const START_OVERHEAD: u16 = 1910;
        const END_OVERHEAD: u16 = 960;
        const MSRC_OVERHEAD: u16 = 660;
        const TCC_OVERHEAD: u16 = 590;
        const DSS_OVERHEAD: u16 = 690;
        const PRE_RANGE_OVERHEAD: u16 = 660;
        const FINAL_RANGE_OVERHEAD: u16 = 550;

        let mut enables: SequenceStepEnables = SequenceStepEnables::default();
        let mut timeouts: SequenceStepTimeouts = SequenceStepTimeouts::default();

        // "Start and end overhead times always present"
        let mut budget_us: u32 = u32::from(START_OVERHEAD) + u32::from(END_OVERHEAD);

        self.get_sequence_step_enables(&mut enables)?;
        self.get_sequence_step_timeouts(&mut timeouts, enables)?;

        if enables.tcc {
            budget_us += timeouts.msrc_dss_tcc_us + u32::from(TCC_OVERHEAD);
        }

        if enables.dss {
            budget_us += 2 * (timeouts.msrc_dss_tcc_us + u32::from(DSS_OVERHEAD));
        } else if enables.msrc {
            budget_us += timeouts.msrc_dss_tcc_us + u32::from(MSRC_OVERHEAD);
        }

        if enables.pre_range {
            budget_us += timeouts.pre_range_us + u32::from(PRE_RANGE_OVERHEAD);
        }

        if enables.final_range {
            budget_us += timeouts.final_range_us + u32::from(FINAL_RANGE_OVERHEAD);
        }

        self.measurement_timing_budget_us = budget_us; // store for internal reuse
        Ok(budget_us)
    }

    fn set_measurement_timing_budget(
        &mut self,
        budget_us: u32,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        const START_OVERHEAD: u16 = 1910;
        const END_OVERHEAD: u16 = 960;
        const MSRC_OVERHEAD: u16 = 660;
        const TCC_OVERHEAD: u16 = 590;
        const DSS_OVERHEAD: u16 = 690;
        const PRE_RANGE_OVERHEAD: u16 = 660;
        const FINAL_RANGE_OVERHEAD: u16 = 550;

        let mut enables: SequenceStepEnables = SequenceStepEnables::default();
        let mut timeouts: SequenceStepTimeouts = SequenceStepTimeouts::default();

        let mut used_budget_us: u32 = u32::from(START_OVERHEAD) + u32::from(END_OVERHEAD);

        self.get_sequence_step_enables(&mut enables)
            .map_err(InitialisationError::I2C)?;
        self.get_sequence_step_timeouts(&mut timeouts, enables)
            .map_err(InitialisationError::I2C)?;

        if enables.tcc {
            used_budget_us += timeouts.msrc_dss_tcc_us + u32::from(TCC_OVERHEAD);
        }

        if enables.dss {
            used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + u32::from(DSS_OVERHEAD));
        } else if enables.msrc {
            used_budget_us += timeouts.msrc_dss_tcc_us + u32::from(MSRC_OVERHEAD);
        }

        if enables.pre_range {
            used_budget_us += timeouts.pre_range_us + u32::from(PRE_RANGE_OVERHEAD);
        }

        if enables.final_range {
            used_budget_us += u32::from(FINAL_RANGE_OVERHEAD);

            // "Note that the final range timeout is determined by the timing
            // budget and the sum of all other timeouts within the sequence.
            // If there is no room for the final range timeout, then an error
            // will be set. Otherwise the remaining time will be applied to
            // the final range."

            if used_budget_us > budget_us {
                // "Requested timeout too big."
                return Err(InitialisationError::TimingBudgetTooShort);
            }

            let final_range_timeout_us: u32 = budget_us - used_budget_us;

            // set_sequence_step_timeout() begin
            // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

            // "For the final range timeout, the pre-range timeout
            //  must be added. To do this both final and pre-range
            //  timeouts must be expressed in macro periods MClks
            //  because they have different vcsel periods."

            let mut final_range_timeout_mclks: u32 = timeout_microseconds_to_mclks(
                final_range_timeout_us,
                timeouts.final_range_vcsel_period_pclks,
            );

            if enables.pre_range {
                final_range_timeout_mclks += u32::from(timeouts.pre_range_mclks);
            }

            self.write_u16(
                Register::FinalRangeConfigTimeoutMacropHi as u8,
                encode_timeout(final_range_timeout_mclks),
            )
            .map_err(InitialisationError::I2C)?;

            // set_sequence_step_timeout() end

            self.measurement_timing_budget_us = budget_us; // store for internal
                                                           // reuse
        }

        Ok(())
    }

    fn get_sequence_step_enables(&mut self, enables: &mut SequenceStepEnables) -> Result<(), EI2C> {
        let sequence_config: u8 = self.read(Register::SystemSequenceConfig as u8)?;

        enables.tcc = ((sequence_config >> 4) & 0x1) != 0;
        enables.dss = ((sequence_config >> 3) & 0x1) != 0;
        enables.msrc = ((sequence_config >> 2) & 0x1) != 0;
        enables.pre_range = ((sequence_config >> 6) & 0x1) != 0;
        enables.final_range = ((sequence_config >> 7) & 0x1) != 0;

        Ok(())
    }

    fn get_sequence_step_timeouts(
        &mut self,
        timeouts: &mut SequenceStepTimeouts,
        enables: SequenceStepEnables,
    ) -> Result<(), EI2C> {
        timeouts.pre_range_vcsel_period_pclks =
            self.get_vcsel_pulse_period(VcselPeriodType::VcselPeriodPreRange)?;

        timeouts.msrc_dss_tcc_mclks =
            u16::from(self.read(Register::MsrcConfigTimeoutMacrop as u8)?) + 1;
        timeouts.msrc_dss_tcc_us = timeout_mclks_to_microseconds(
            timeouts.msrc_dss_tcc_mclks,
            timeouts.pre_range_vcsel_period_pclks,
        );

        timeouts.pre_range_mclks =
            decode_timeout(self.read_u16(Register::PreRangeConfigTimeoutMacropHi as u8)?);
        timeouts.pre_range_us = timeout_mclks_to_microseconds(
            timeouts.pre_range_mclks,
            timeouts.pre_range_vcsel_period_pclks,
        );

        timeouts.final_range_vcsel_period_pclks =
            self.get_vcsel_pulse_period(VcselPeriodType::VcselPeriodFinalRange)?;

        timeouts.final_range_mclks =
            decode_timeout(self.read_u16(Register::FinalRangeConfigTimeoutMacropHi as u8)?);

        if enables.pre_range {
            timeouts.final_range_mclks -= timeouts.pre_range_mclks;
        }

        timeouts.final_range_us = timeout_mclks_to_microseconds(
            timeouts.final_range_mclks,
            timeouts.final_range_vcsel_period_pclks,
        );

        Ok(())
    }

    fn get_vcsel_pulse_period(&mut self, period_type: VcselPeriodType) -> Result<u16, EI2C> {
        Ok(match period_type {
            VcselPeriodType::VcselPeriodPreRange => {
                decode_vcsel_period(self.read(Register::PreRangeConfigVcselPeriod as u8)?).into()
            }
            VcselPeriodType::VcselPeriodFinalRange => {
                decode_vcsel_period(self.read(Register::FinalRangeConfigVcselPeriod as u8)?).into()
            }
        })
    }

    fn perform_single_ref_calibration(
        &mut self,
        vhv_init_byte: u8,
        timer: &impl ClockSource,
    ) -> Result<(), InitialisationError<EI2C, EX>> {
        self.write(Register::SysrangeStart as u8, 0x01 | vhv_init_byte)
            .map_err(InitialisationError::I2C)?; // VL53L0X_REG_SYSRANGE_MODE_START_STOP

        let start_time: u32 = timer.get_ms();
        #[allow(clippy::verbose_bit_mask)]
        while (self
            .read(Register::ResultInterruptStatus as u8)
            .map_err(InitialisationError::I2C)?
            & 0x07)
            == 0
        {
            if (timer.get_ms() - start_time) > 500 {
                // 500 ms
                return Err(InitialisationError::CalibrationTimeout);
            }
        }

        self.write(Register::SystemInterruptClear as u8, 0x01)
            .map_err(InitialisationError::I2C)?;

        self.write(Register::SysrangeStart as u8, 0x00)
            .map_err(InitialisationError::I2C)?;

        Ok(())
    }
}

impl<I2C, X, EI2C, EX> Vl53l0x<I2C, X>
where
    I2C: embedded_hal::i2c::I2c<Error = EI2C>,
    X: embedded_hal::digital::OutputPin<Error = EX>,
{
    fn write(&mut self, register: u8, data: u8) -> Result<(), EI2C> {
        self.i2c.write(self.address, &[register, data])
    }

    fn write_u16(&mut self, register: u8, data: u16) -> Result<(), EI2C> {
        #[allow(clippy::cast_possible_truncation)]
        self.i2c
            .write(self.address, &[register, (data >> 8) as u8, data as u8])
    }

    fn write_u32(&mut self, register: u8, data: u32) -> Result<(), EI2C> {
        #[allow(clippy::cast_possible_truncation)]
        self.i2c.write(
            self.address,
            &[
                register,
                (data >> 24) as u8,
                (data >> 16) as u8,
                (data >> 8) as u8,
                data as u8,
            ],
        )
    }

    fn write_many(&mut self, register: u8, data: &[u8]) -> Result<(), EI2C> {
        self.i2c.transaction(
            self.address,
            &mut [
                embedded_hal::i2c::Operation::Write(&[register]),
                embedded_hal::i2c::Operation::Write(data),
            ],
        )
    }

    fn read(&mut self, register: u8) -> Result<u8, EI2C> {
        let mut data = [0];
        self.i2c.write_read(self.address, &[register], &mut data)?;
        Ok(data[0])
    }

    fn read_u16(&mut self, register: u8) -> Result<u16, EI2C> {
        let mut data = [0; 2];
        self.i2c.write_read(self.address, &[register], &mut data)?;
        Ok(u16::from(data[0]) << 8 | u16::from(data[1]))
    }

    fn read_many(&mut self, register: u8, data: &mut [u8]) -> Result<(), EI2C> {
        self.i2c.write_read(self.address, &[register], data)
    }

    fn update(&mut self, register: u8, f: impl FnOnce(&mut u8)) -> Result<(), EI2C> {
        let mut data = [0];
        self.i2c.write_read(self.address, &[register], &mut data)?;
        f(&mut data[0]);
        self.i2c.write(self.address, &[register, data[0]])
    }
}

fn calc_macro_period(vcsel_period_pclks: u16) -> u32 {
    ((2304 * u32::from(vcsel_period_pclks) * 1655) + 500) / 1000
}

fn timeout_microseconds_to_mclks(timeout_period_us: u32, vcsel_period_pclks: u16) -> u32 {
    let macro_period_ns: u32 = calc_macro_period(vcsel_period_pclks);

    ((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns
}

fn timeout_mclks_to_microseconds(timeout_period_mclks: u16, vcsel_period_pclks: u16) -> u32 {
    let macro_period_ns: u32 = calc_macro_period(vcsel_period_pclks);

    ((u32::from(timeout_period_mclks) * macro_period_ns) + 500) / 1000
}

fn encode_timeout(timeout_mclks: u32) -> u16 {
    let mut ls_byte: u32;
    let mut ms_byte: u16 = 0;

    if timeout_mclks > 0 {
        ls_byte = timeout_mclks - 1;

        while (ls_byte & 0xFFFF_FF00) > 0 {
            ls_byte >>= 1;
            ms_byte += 1;
        }

        (ms_byte << 8) | (ls_byte & 0xFF) as u16
    } else {
        0
    }
}

fn decode_timeout(reg_val: u16) -> u16 {
    ((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1
}

fn decode_vcsel_period(reg_val: u8) -> u8 {
    ((reg_val) + 1) << 1
}
