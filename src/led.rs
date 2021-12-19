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
    pub fn init(pin: Pin<P, PushPullOutput>) -> Self {
        Self {
            pin,
            state: PinState::Low,
        }
    }

    pub fn on(&mut self) {
        let _ = self.pin.set_low();
        self.state = PinState::Low;
    }
    pub fn off(&mut self) {
        let _ = self.pin.set_high();
        self.state = PinState::High;
    }
    pub fn toggle(&mut self) {
        self.state = !self.state;
        let _ = self.pin.set_state(self.state);
    }
}
