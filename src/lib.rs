#![doc = include_str!("../README.md")]
#![no_std]

mod imp;
mod reg;

/// The default I2C address for the VL53L1X.
pub const DEFAULT_ADDRESS: u8 = 0b010_1001;

/// Possible errors that can occur during initialisation.
#[derive(Debug)]
pub enum InitialisationError<EI2C, EX> {
    /// An error occurred on the I2C bus.
    I2C(EI2C),
    /// An error occurred setting the xshut pin.
    XShut(EX),

    /// The model ID read from the device was invalid.
    InvalidModelId(u16),
    /// The device timed out.
    Timeout,
    /// The timing budget was invalid.
    InvalidTimingBudget,
}

/// A VL53L1X driver. Use the `new` function to create a new instance, then
/// call `range` to get the current range, if available.
pub struct Vl53l1x<I2C, X> {
    i2c: I2C,
    _x_shut: X,
    address: u8,

    fast_osc_frequency: u16,
    osc_calibrate_val: u16,
    distance_mode: imp::DistanceMode,
    calibrated: bool,
    saved_vhv_init: u8,
    saved_vhv_timeout: u8,
}

impl<I2C, X, EI2C, EX> Vl53l1x<I2C, X>
where
    I2C: embedded_hal::i2c::I2c<Error = EI2C>,
    X: embedded_hal::digital::OutputPin<Error = EX>,
{
    /// Create a new instance of the VL53L1X driver. This performs a reset of
    /// the device and may fail if the device is not present. This will use the
    /// `x_shut` pin to select the device to be reset, then it will change that
    /// devices' address to the provided address.
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
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<Self, InitialisationError<EI2C, EX>> {
        // Reset the device by driving XSHUT low for 10ms.
        x_shut.set_low().map_err(InitialisationError::XShut)?;
        delay.delay_ms(10);
        x_shut.set_high().map_err(InitialisationError::XShut)?;
        delay.delay_ms(10);

        let mut this = Self {
            i2c,
            _x_shut: x_shut,
            address: DEFAULT_ADDRESS,

            fast_osc_frequency: 0,
            osc_calibrate_val: 0,
            distance_mode: imp::DistanceMode::Long,
            calibrated: false,
            saved_vhv_init: 0,
            saved_vhv_timeout: 0,
        };

        this.init(delay)?;

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
        self.try_read_inner()
    }
}
