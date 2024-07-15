#![doc = include_str!("../README.md")]
#![no_std]

mod imp;
mod reg;

/// The default I2C address for the VL53L0X.
pub const DEFAULT_ADDRESS: u8 = 0b010_1001;

/// Possible errors that can occur during initialisation.
#[derive(Debug)]
pub enum InitialisationError<EI2C, EX> {
    /// An error occurred on the I2C bus.
    I2C(EI2C),
    /// An error occurred setting the xshut pin.
    XShut(EX),

    /// The model ID read from the device was invalid.
    InvalidModelId(u8),
    /// The signal rate limit was invalid.
    InvalidSignalRateLimit(f32),
    /// The measurement timing budget was too short.
    TimingBudgetTooShort,
    /// The device timed out while calibrating.
    CalibrationTimeout,
}

/// A time source with millisecond precision.
pub trait ClockSource {
    /// Get the current time in milliseconds.
    fn get_ms(&self) -> u32;
}

/// A VL53L0X driver. Use the `new` function to create a new instance, then
/// call `range` to get the current range, if available.
pub struct Vl53l0x<I2C, X> {
    i2c: I2C,
    _x_shut: X,
    address: u8,

    stop_variable: u8,
    measurement_timing_budget_us: u32,
}

impl<I2C, X, EI2C, EX> Vl53l0x<I2C, X>
where
    I2C: embedded_hal::i2c::I2c<Error = EI2C>,
    X: embedded_hal::digital::OutputPin<Error = EX>,
{
    /// Create a new instance of the VL53L0X driver. This performs a reset of
    /// the device and may fail if the device is not present. This will use the
    /// `x_shut` pin to select the device to be reset, then it will change that
    /// devices' address to the provided address.
    ///
    /// For calibration, a time source is needed to measure a timeout. Provide
    /// an implementation of [`ClockSource`], this could be time since program
    /// start, or a realtime clock. This is also used to provide small
    /// delays, seeing as we already have a time source.
    ///
    /// If these sensors are being used in an array, set all the xshut pins low
    /// prior to calling this function which will allow this to only initialize
    /// a single sensor.
    ///
    /// # Errors
    /// Forwards any errors from the I2C bus and xshut pin, as well as any
    /// initialisation errors from the device itself.
    pub fn new(
        i2c: I2C,
        mut x_shut: X,
        address: u8,
        timer: &impl ClockSource,
    ) -> Result<Self, InitialisationError<EI2C, EX>> {
        // Reset the device by driving XSHUT low for 10ms.
        x_shut.set_low().map_err(InitialisationError::XShut)?;
        {
            let start_time = timer.get_ms();
            while timer.get_ms() - start_time < 10 {} // Delay for 10ms.
        }
        x_shut.set_high().map_err(InitialisationError::XShut)?;
        {
            let start_time = timer.get_ms();
            while timer.get_ms() - start_time < 10 {} // Delay for 10ms.
        }

        let mut this = Self {
            i2c,
            _x_shut: x_shut,
            address: DEFAULT_ADDRESS,

            stop_variable: 0,
            measurement_timing_budget_us: 0,
        };

        this.init(timer)?;

        this.set_address(address)
            .map_err(InitialisationError::I2C)?;

        this.start_continuous(50)
            .map_err(InitialisationError::I2C)?;

        Ok(this)
    }

    /// Poll the device for a new range value. Returns `None` if the device is
    /// still measuring.
    ///
    /// The range is returned in millimeters.
    ///
    /// # Errors
    /// Forwards any errors from the I2C bus.
    pub fn try_read(&mut self) -> Result<Option<u16>, EI2C> {
        self.try_read_range_continuous_millimeters()
    }
}
