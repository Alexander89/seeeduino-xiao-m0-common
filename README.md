# rotation to uart (AS5600 HALL)

This project is the connector to translate an AS5600 hall sensor output to uart.

1. The current step is send via uart as u8
2. Additional information are traced over usb

## UART

As soon the position changed, the current step is emitted. In addition, every 2.5 sec 
the not changed value is emitted again.

If the value do not change for longer than 2.5 sec. The Setup parameters are read, too.  

## Todo

configure the number of steps per resolution with uart and/or usb

## Hardware: Seeeduino XIAO

This crate provides a type-safe API for working with the [Seeed Studio
Seeeduino XIAO](http://wiki.seeedstudio.com/Seeeduino-XIAO/).

## Prerequisites

- Install the cross compile toolchain `rustup target add thumbv6m-none-eabi`
- Install the [cargo-hf2 tool](https://crates.io/crates/cargo-hf2) however your
  platform requires

## Uploading the software

- Be in the project directory
- Put your device in bootloader mode by bridging the `RST` pads _twice_ in
  quick succession. The orange LED will pulse when the device is in bootloader
  mode.
- Build and upload in one step: `cargo hf2 --release`
  - Note that if you're using an older `cargo-hf2` that you'll need to specify
    the VID/PID when flashing: `cargo hf2 --vid 0x2886 --pid 0x002f --release`

Check out [the
repository](https://github.com/atsamd-rs/atsamd/tree/master/boards/xiao_m0/examples)
for examples.
