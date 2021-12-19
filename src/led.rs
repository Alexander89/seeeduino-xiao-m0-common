//! wraps a pin into an led struct.
//!
//! It has nearly no more functions than a better name
//! and you don't need the `unproven` feature to use toggle.
//!
//! # Examples
//!
//! ```
//! use seeeduino_xiao_m0_common::led::Led;
//! let led = Led::init(pins.a1);
//! led.on();
//! led.toggle();
//! ```
//!
use embedded_hal::digital::v2::PinState;
use xiao_m0::hal::{
    gpio::{
        v2::{Pin, PushPullOutput},
        PinId,
    },
    prelude::*,
};

pub struct Led<P: PinId> {
    pin: Pin<P, PushPullOutput>,
    state: PinState,
}

impl<P: PinId> Led<P> {
    /// creates an LED wrapper with nearly no more functions than a better name.
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::led::Led;
    /// let led = Led::init(pins.a1.into());
    /// led.on();
    /// ```
    pub fn init(pin: Pin<P, PushPullOutput>) -> Self {
        Self {
            pin,
            state: PinState::Low,
        }
    }

    /// set the led to state on
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::led::Led;
    ///
    /// let mut led = Led::init(pins.a1.into());
    /// led.on();
    /// ```
    pub fn on(&mut self) {
        let _ = self.pin.set_low();
        self.state = PinState::Low;
    }

    /// set the led to state of
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::led::Led;
    ///
    /// let mut led = Led::init(pins.a1.into());
    /// led.off();
    /// ```
    pub fn off(&mut self) {
        let _ = self.pin.set_high();
        self.state = PinState::High;
    }

    /// set the led to state off
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::led::Led;
    ///
    /// let mut led = Led::init(pins.a1.into());
    /// led.off();
    /// led.toggle();
    /// // should be on now
    /// ```
    pub fn toggle(&mut self) {
        self.state = !self.state;
        let _ = self.pin.set_state(self.state);
    }
}
