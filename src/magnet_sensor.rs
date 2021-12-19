//! # Protocol implementation for an AS5600 Hall Sensor.
//!
//! there are two information bundles you can query from the sensor.
//!
//!  1. setup: Reads the agc, status, and magnitude value from the sensor.
//!  2. data: reads the raw angle from the sensor. To do so:
//!      - you can read all data at once in a blocking fashion
//!      - You can use the chunked method to operate only one i2C operation per call (1. query, 2: Read HByte 3. Read LByte)
//!      - TODO: You can use DMA
//!
//! # Examples
//! ```
//! use seeeduino_xiao_m0_common::{
//!     i2c:I2c,
//!     magnet_sensor::MagnetSensor
//! };
//!
//! let mut peripherals = Peripherals::take().unwrap();
//! let mut clocks = GenericClockController::with_internal_8mhz(
//!     peripherals.GCLK,
//!     &mut peripherals.PM,
//!     &mut peripherals.SYSCTRL,
//!     &mut peripherals.NVMCTRL,
//! );
//!
//! let i2c = I2c::init(
//!     &mut clocks,
//!     peripherals.SERCOM0,
//!     &mut peripherals.PM,
//!     pins.a4,
//!     pins.a5,
//! );
//!
//! let mut magnet_sensor = MagnetSensor::init();
//! assert_eq!(magnet_sensor.poll(&mut i2c), Ok(()));
//!
//! assert_eq!(magnet_sensor.detected, true);
//! assert_eq!(magnet_sensor.status, 0b100);
//! assert_eq!(magnet_sensor.raw_angle, 4095);
//! ```

#![allow(dead_code)]
use xiao_m0::hal::sercom::I2CError;

use super::i2c::I2c;

enum ReadPhase {
    Query,
    Read1,
    Read2,
}

/// Representing the state of the hall sensor and the current connection state
pub struct MagnetSensor {
    status: u8,
    /// value if the sensor detects the magnet
    pub detected: bool,
    low: bool,
    heigh: bool,
    /// raw_angle value from the sensor (0-360°) 0 - 4095_u16
    pub raw_angle: u16,
    /// gain value of the hall sensor
    pub agc: u8,
    /// strength of the magnetic field
    pub magnitude: u16,

    read_phase: ReadPhase,
    read_phase_buf: u16,
}

impl MagnetSensor {
    /// Create a new instance of an MagnetSensor.
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::magnet_sensor::MagnetSensor;
    ///
    /// MagnetSensor::init();
    /// ```
    pub fn init() -> Self {
        Self {
            status: 0,
            detected: false,
            low: false,
            heigh: false,
            raw_angle: 0,
            agc: 0,
            magnitude: 0,
            read_phase: ReadPhase::Query,
            read_phase_buf: 0,
        }
    }

    /// Poll the current angle from the hall magnet sensor.
    ///
    /// An I2C instance is required to read the data from the sensor.
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::{
    ///     i2c:I2c,
    ///     magnet_sensor::MagnetSensor
    /// };
    ///
    /// let mut peripherals = Peripherals::take().unwrap();
    /// let mut clocks = GenericClockController::with_internal_8mhz(
    ///     peripherals.GCLK,
    ///     &mut peripherals.PM,
    ///     &mut peripherals.SYSCTRL,
    ///     &mut peripherals.NVMCTRL,
    /// );
    ///
    /// let i2c = I2c::init(
    ///     &mut clocks,
    ///     peripherals.SERCOM0,
    ///     &mut peripherals.PM,
    ///     pins.a4,
    ///     pins.a5,
    /// );
    ///
    /// let mut magnet_sensor = MagnetSensor::init();
    /// assert_eq!(magnet_sensor.poll(&mut i2c), Ok(()));
    /// assert_eq!(magnet_sensor.detected, true);
    /// assert_eq!(magnet_sensor.status, 0b100);
    /// assert_eq!(magnet_sensor.raw_angle, 4095);
    /// ```
    ///
    /// # Errors
    ///
    /// This function will return an error if `i2c.i2c_read_some` fails.
    pub fn poll(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 5];

        self.detected = false;

        i2c.i2c_read_some(0x36, 0x0Bu8, 3, &mut buf)?;

        self.status = (buf[0] & 0b111000) >> 3;
        self.detected = (self.status & 0b100) != 0;
        self.low = (self.status & 0b10) != 0;
        self.heigh = (self.status & 0b1) != 0;
        self.raw_angle = (((buf[1] as u16) << 8) + (buf[2] as u16)) & 0x0FFF;

        Ok(())
    }

    /// .
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::{
    ///     i2c:I2c,
    ///     magnet_sensor::MagnetSensor
    /// };
    ///
    /// let mut peripherals = Peripherals::take().unwrap();
    /// let mut clocks = GenericClockController::with_internal_8mhz(
    ///     peripherals.GCLK,
    ///     &mut peripherals.PM,
    ///     &mut peripherals.SYSCTRL,
    ///     &mut peripherals.NVMCTRL,
    /// );
    ///
    /// let i2c = I2c::init(
    ///     &mut clocks,
    ///     peripherals.SERCOM0,
    ///     &mut peripherals.PM,
    ///     pins.a4,
    ///     pins.a5,
    /// );
    ///
    /// let mut magnet_sensor = MagnetSensor::init();
    /// assert_eq!(magnet_sensor.stepwise_read(&mut i2c), Ok(false));
    /// assert_eq!(magnet_sensor.stepwise_read(&mut i2c), Ok(false));
    /// assert_eq!(magnet_sensor.stepwise_read(&mut i2c), Ok(true));
    /// assert_eq!(magnet_sensor.detected, true);
    /// assert_eq!(magnet_sensor.status, 0b100);
    /// assert_eq!(magnet_sensor.raw_angle, 4095);
    /// ```
    ///
    /// # Errors
    ///
    /// This function will return an error if the I2C connection is not working.
    pub fn stepwise_read(&mut self, i2c: &mut I2c) -> Result<bool, I2CError> {
        match self.read_phase {
            ReadPhase::Query => {
                i2c.i2c_query(0x36, 0x0Cu8)?;

                self.read_phase = ReadPhase::Read1;
                Ok(false)
            }
            ReadPhase::Read1 => {
                self.read_phase_buf = i2c.i2c_read_one(0x36)? as u16;

                self.read_phase = ReadPhase::Read2;
                Ok(false)
            }
            ReadPhase::Read2 => {
                let v = i2c.i2c_read_one(0x36)? as u16;

                self.raw_angle = ((self.read_phase_buf << 8) + v) & 0x0FFF;
                self.read_phase = ReadPhase::Query;
                Ok(true)
            }
        }
    }

    /// Read
    ///  - agc
    ///  - magnitude
    ///  - detected
    ///  - status
    ///  - detected
    ///  - low
    ///  - heigh
    /// from the sensor
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::magnet_sensor::MagnetSensor;
    ///
    /// let mut magnet_sensor = ;
    /// let mut i2c = ;
    /// assert_eq!(magnet_sensor.poll_setup(&mut i2c), );
    /// assert_eq!(magnet_sensor, );
    /// assert_eq!(i2c, );
    /// ```
    ///
    /// # Errors
    ///
    /// This function will return an error if .
    pub fn poll_setup(&mut self, i2c: &mut I2c) -> Result<(), I2CError> {
        let mut buf = [0u8; 5];
        self.agc = 0;
        self.magnitude = 0;

        i2c.i2c_read_some(0x36, 0x1Au8, 3, &mut buf)?;

        self.agc = buf[0] as u8;
        self.magnitude = (((buf[1] as u16) << 8) + (buf[2] as u16)) & 0x0FFF;

        i2c.i2c_read_some(0x36, 0x0Bu8, 1, &mut buf)?;
        self.detected = false;
        self.status = (buf[0] & 0b111000) >> 3;
        self.detected = (self.status & 0b100) != 0;
        self.low = (self.status & 0b10) != 0;
        self.heigh = (self.status & 0b1) != 0;
        Ok(())
    }
}
