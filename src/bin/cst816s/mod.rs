//! Original work from https://github.com/jonlamb-gh/pinetime-rs/blob/master/pinetime-drivers/src/cst816s.rs [60e677ca545e64e05891c60fc1f9a2f0c540ddde]
//! Hynitron CST816S touch panel driver
//!
//! Pins:
//! * P0.10 : Reset
//! * P0.28 : Interrupt (signal to the CPU when a touch event is detected)
//! * P0.06 : I²C SDA
//! * P0.07 : I²C SCL
//!
//! I²C
//! Device address : 0x15
//! Frequency : from 10Khz to 400Khz

use crate::hal::{
    gpio::{p0, Floating, Input, Output, Pin, PushPull},
    gpiote::GpioteChannel,
    prelude::{OutputPin, _embedded_hal_blocking_delay_DelayMs as DelayMs},
    twim::{self, Error, Twim},
};
use core::fmt;

/// CST816S I2C address
pub const ADDRESS: u8 = 0x15;

/// The TWI device should work @ up to 400Khz but there is a HW bug which prevent it from
/// respecting correct timings. According to erratas heet, this magic value makes it run
/// at ~390Khz with correct timings.
pub const MAX_FREQUENCY: u32 = 0x06200000;

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[repr(u8)]
pub enum Gesture {
    SlideDown = 0x01,
    SlideUp = 0x02,
    SlideLeft = 0x03,
    SlideRight = 0x04,
    SingleTap = 0x05,
    DoubleTap = 0x0B,
    LongPress = 0x0C,
}

impl Gesture {
    /*
    fn as_u8(self) -> u8 {
        self as u8
    }
    */

    fn from_u8(val: u8) -> Option<Self> {
        use Gesture::*;
        match val {
            0x01 => SlideDown,
            0x02 => SlideUp,
            0x03 => SlideLeft,
            0x04 => SlideRight,
            0x05 => SingleTap,
            0x0B => DoubleTap,
            0x0C => LongPress,
            _ => return None,
        }
        .into()
    }
}

impl fmt::Display for Gesture {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
pub struct TouchData {
    pub x: u16,
    pub y: u16,
    pub gesture: Option<Gesture>,
    pub is_touching: bool,
}

impl TouchData {
    // TODO - make a proper wrapper type
    // add Event, TouchId, Pressure
    fn from_le_bytes(bytes: &[u8; 7]) -> Self {
        let gesture = Gesture::from_u8(bytes[1]);
        let num_touch_points = bytes[2] & 0x0F;
        let x_msb = bytes[3] & 0x0F;
        let x_lsb = bytes[4];
        let x = (x_lsb as u16) | ((x_msb as u16) << 8);
        let y_msb = bytes[5] & 0x0F;
        let y_lsb = bytes[6];
        let y = (y_lsb as u16) | ((y_msb as u16) << 8);
        TouchData {
            x,
            y,
            gesture,
            is_touching: num_touch_points > 0,
        }
    }
}

impl fmt::Display for TouchData {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{}, {} : {:?} : {}",
            self.x, self.y, self.gesture, self.is_touching
        )
    }
}

pub type ResetPin = p0::P0_10<Output<PushPull>>;
pub type InterruptPin = p0::P0_28<Input<Floating>>;

/// CST816S driver
pub struct Cst816s<TWIM> {
    twim: Twim<TWIM>,
    reset_pin: ResetPin,
    _int_pin: Pin<Input<Floating>>,
    buffer: [u8; 7],
}

impl<TWIM> Cst816s<TWIM>
where
    TWIM: twim::Instance,
{
    pub fn new(
        twim: Twim<TWIM>,
        reset_pin: ResetPin,
        int_pin: InterruptPin,
        channel: &GpioteChannel<'_>,
    ) -> Self {
        let int_pin = int_pin.degrade();
        channel.input_pin(&int_pin).lo_to_hi().enable_interrupt();
        Cst816s {
            twim,
            reset_pin,
            _int_pin: int_pin,
            buffer: [0; 7],
        }
    }

    pub fn init<T: DelayMs<u8>>(&mut self, delay: &mut T) -> Result<(), Error> {
        self.reset_pin.set_high().unwrap();
        delay.delay_ms(50);
        self.reset_pin.set_low().unwrap();
        delay.delay_ms(5);
        self.reset_pin.set_high().unwrap();
        delay.delay_ms(50);

        let _ = self.read_register(Register::Wakeup0)?;
        delay.delay_ms(5);
        let _ = self.read_register(Register::Wakeup1)?;
        delay.delay_ms(5);

        // [2] EnConLR - Continuous operation can slide around
        // [1] EnConUD - Slide up and down to enable continuous operation
        // [0] EnDClick - Enable Double-click action
        self.write_register(Register::Motion, 0b00000101)?;

        // [7] EnTest - Interrupt pin to test, enable automatic periodic issued after a low pulse.
        // [6] EnTouch - When a touch is detected, a periodic pulsed Low.
        // [5] EnChange - Upon detecting a touch state changes, pulsed Low.
        // [4] EnMotion - When the detected gesture is pulsed Low.
        // [0] OnceWLP - Press gesture only issue a pulse signal is low.
        self.write_register(Register::IrqCtl, 0b01110000)?;

        Ok(())
    }

    /*
    pub fn sleep<T: DelayMs<u8>>(&mut self, delay: &mut T) -> Result<(), Error> {
        self.reset_pin.set_low().unwrap();
        delay.delay_ms(5);
        self.reset_pin.set_high().unwrap();
        delay.delay_ms(50);
        self.write_register(Register::PowerMode, 0x03)?;
        Ok(())
    }
    */

    pub fn read_touch_data(&mut self) -> Option<TouchData> {
        let addr = [0];
        match self
            .twim
            .copy_write_then_read(ADDRESS, &addr, &mut self.buffer)
        {
            Err(_e) => None,
            Ok(()) => Some(TouchData::from_le_bytes(&self.buffer)),
        }
    }

    fn read_register(&mut self, register: Register) -> Result<u8, Error> {
        let tx = [register.addr()];
        let mut rx = [0_u8; 1];
        self.twim.copy_write_then_read(ADDRESS, &tx, &mut rx)?;
        Ok(rx[0])
    }

    fn write_register(&mut self, register: Register, value: u8) -> Result<(), Error> {
        let tx = [register.addr(), value];
        self.twim.write(ADDRESS, &tx)?;
        Ok(())
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[repr(u8)]
enum Register {
    Wakeup0 = 0x15,
    Wakeup1 = 0xA7,
    Motion = 0xEC,
    IrqCtl = 0xFA,
    //PowerMode = 0xA5,
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}
