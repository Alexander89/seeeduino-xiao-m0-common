//! This module represents an usb serial interface to send data to a host.
//!
//! # Critical Section
//!
//! This module use a interruption lock internally. No additional protection of the bus is required.
//! if you use `static mut` the rust could not protect you from yourself. De careful doing this.
//!
//! # Examples
//!
//! This example will sen a dot to the host again and again
//!
//! ```
//! use seeeduino_xiao_m0_common::usb_serial::UsbSerial;
//! static mut USB_SERIAL = None;
//! #[entry]
//! fn main() {
//!     let usb = UsbSerial::init(
//!         &mut clocks,
//!         peripherals.USB,
//!         &mut peripherals.PM,
//!         pins.usb_dm,
//!         pins.usb_dp,
//!         &mut core.NVIC,
//!     )
//!     let mut usb_serial = unsafe {
//!         USB_SERIAL = Some(usb);
//!         USB_SERIAL.as_mut.unwrap()
//!     }
//!
//!     loop {
//!         usb_serial.serial_write_len(b".", 1);
//!         for _ in 0..0xfffff { nop() }
//!     }
//! }
//!
//! #[interrupt]
//! fn USB() {
//!     usb_serial.poll();
//!     let input = usb_serial.read_poll();
//! }
//! ```

#![allow(dead_code)]
use core::cell::RefCell;

use cortex_m::interrupt::{self as interrupt_cs, Mutex};
use usb_device::{
    class::UsbClass,
    class_prelude::UsbBusAllocator,
    device::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use xiao_m0::hal::{clock::GenericClockController, target_device::NVIC, usb::UsbBus};
use xiao_m0::{pac, UsbDm, UsbDp};

use string_helper::u32_to_str;

static mut BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;

pub struct UsbSerial {
    bus: UsbDevice<'static, UsbBus>,
    serial: Mutex<RefCell<SerialPort<'static, UsbBus>>>,
}

impl UsbSerial {
    /// poll must be triggered in the USB() interrupt.
    /// It is required to push the data into the bus.
    ///
    /// # Examples
    ///
    /// ```
    /// static mut usb_serial = None;
    /// fn main() {
    ///     use seeeduino_xiao_m0_common::usb_serial::UsbSerial;
    ///     let mut usb_serial = Some(UsbSerial::init(
    ///         &mut clocks,
    ///         peripherals.USB,
    ///         &mut peripherals.PM,
    ///         pins.usb_dm,
    ///         pins.usb_dp,
    ///         &mut core.NVIC,
    ///     ));
    /// }
    ///
    /// // ...
    /// #[interrupt]
    /// fn USB() {
    ///     usb_serial.poll();
    ///     let input = usb_serial.read_poll();
    /// }
    /// ```
    pub fn poll(&mut self) {
        cortex_m::interrupt::free(|cs| {
            let mut serial = self.serial.borrow(cs).borrow_mut();
            let mut classes: [&mut dyn UsbClass<UsbBus>; 1] = [&mut *serial];
            self.bus.poll(&mut classes);
        })
    }

    /// read_poll must be triggered in the USB() interrupt.
    /// It is required to handle the incoming data from the host and clear the usb buffer.
    ///
    /// # Examples
    ///
    /// ```
    /// static mut usb_serial = None;
    /// fn main() {
    ///     use seeeduino_xiao_m0_common::usb_serial::UsbSerial;
    ///     let mut usb_serial = Some(UsbSerial::init(
    ///         &mut clocks,
    ///         peripherals.USB,
    ///         &mut peripherals.PM,
    ///         pins.usb_dm,
    ///         pins.usb_dp,
    ///         &mut core.NVIC,
    ///     ));
    /// }
    ///
    /// // ...
    /// #[interrupt]
    /// fn USB() {
    ///     usb_serial.poll();
    ///     let input = usb_serial.read_poll();
    /// }
    /// ```
    pub fn read_poll(&mut self) -> core::result::Result<(usize, [u8; 100]), ()> {
        cortex_m::interrupt::free(|cs| {
            let mut buf = [0u8; 100];
            let mut serial = self.serial.borrow(cs).borrow_mut();
            match serial.read(&mut buf) {
                Ok(count) if count > 0 => Ok((count, buf)),
                _ => Err(()),
            }
        })
    }

    /// write data into the usb buffer. The poll is required to send the data.
    ///
    /// It is recommended to use the interrupt to poll the usb bus
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::usb_serial::UsbSerial;
    ///
    /// let mut usb_serial = UsbSerial::init(
    ///     &mut clocks,
    ///     peripherals.USB,
    ///     &mut peripherals.PM,
    ///     pins.usb_dm,
    ///     pins.usb_dp,
    ///     &mut core.NVIC,
    /// );
    /// usb_serial.serial_write(bytes);
    ///
    /// // ...
    /// #[interrupt]
    /// fn USB() {
    ///     usb_serial.poll();
    ///     let input = usb_serial.read_poll();
    /// }
    /// ```
    pub fn serial_write(&mut self, bytes: &[u8]) {
        self.serial_write_len(&bytes, bytes.len())
    }

    /// write a number into the usb buffer. The poll is required to send the data.
    ///
    /// It is recommended to use the interrupt to poll the usb bus
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::usb_serial::UsbSerial;
    ///
    /// let mut usb_serial = mk_usb();
    /// usb_serial.serial_write_num(123);
    ///
    /// // ...
    /// #[interrupt]
    /// fn USB() {
    ///     usb_serial.poll();
    ///     let input = usb_serial.read_poll();
    /// }
    /// ```
    #[allow(dead_code)]
    pub fn serial_write_num(&mut self, num: usize) {
        let (_, bytes) = u32_to_str(num as u32);
        self.serial_write_len(&(bytes as [u8; 12]), 12)
    }

    /// write a a buffer with a given length into the usb buffer.
    /// The poll is required to send the data.
    ///
    /// It is recommended to use the interrupt to poll the usb bus
    ///
    /// # Examples
    ///
    /// ```
    /// use seeeduino_xiao_m0_common::usb_serial::UsbSerial;
    ///
    /// let mut usb_serial = mk_usb();
    /// usb_serial.serial_write_len(b"hello world", 5);
    ///
    /// // ...
    /// #[interrupt]
    /// fn USB() {
    ///     usb_serial.poll();
    ///     let input = usb_serial.read_poll();
    /// }
    /// ```
    #[allow(dead_code)]
    pub fn serial_write_len(&mut self, bytes: &[u8], len: usize) {
        cortex_m::interrupt::free(|cs| {
            let mut serial = self.serial.borrow(cs).borrow_mut();
            let _ = serial.write(&bytes[0..len]);
        });
    }
}

impl UsbSerial {
    pub fn init(
        clocks: &mut GenericClockController,
        usb: pac::USB,
        pm: &mut pac::PM,
        dm: impl Into<UsbDm>,
        dp: impl Into<UsbDp>,
        nvic: &mut NVIC,
    ) -> UsbSerial {
        interrupt_cs::free(|_| {
            let bus_allocator = unsafe {
                BUS_ALLOCATOR = Some(xiao_m0::usb_allocator(
                    usb, clocks, pm, //&mut peripherals.PM,
                    dm, //pins.usb_dm,
                    dp, // pins.usb_dp,
                ));
                BUS_ALLOCATOR.as_mut().unwrap()
            };

            let serial = Mutex::new(RefCell::new(SerialPort::new(bus_allocator)));
            let bus = UsbDeviceBuilder::new(bus_allocator, UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Halemba")
                .product("Serial port")
                .serial_number("TEST")
                .device_class(USB_CLASS_CDC)
                .build();
            unsafe {
                nvic.set_priority(pac::interrupt::USB, 2);
                NVIC::unmask(pac::interrupt::USB);
            };
            UsbSerial { bus, serial }
        })
    }
}
