//! Original work from https://github.com/jonlamb-gh/pinetime-rs/blob/master/pinetime-drivers/src/cst816s.rs [60e677ca545e64e05891c60fc1f9a2f0c540ddde]
//! Hynitron CST816S touch panel driver

use core::fmt;

use embassy_nrf::{
    gpio::Output,
    gpiote,
    twim::{self, Twim},
};
use embassy_time::Duration;
use embedded_hal::delay::DelayNs;

const DEFAULT_READ_TIMEOUT: Duration = Duration::from_millis(5);

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
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

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug, defmt::Format)]
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

/// CST816S driver
pub struct Cst816s<'a, TWIM>
where
    TWIM: twim::Instance,
{
    slave_address: u8,
    twim: Twim<'a, TWIM>,
    reset_pin: Output<'a>,
    int_channel: gpiote::InputChannel<'a>,
    buffer: [u8; 7],
}

impl<'a, TWIM> Cst816s<'a, TWIM>
where
    TWIM: twim::Instance,
{
    pub fn new(
        salve_address: u8,
        twim: Twim<'a, TWIM>,
        reset_pin: Output<'a>,
        int_channel: gpiote::InputChannel<'a>,
    ) -> Self {
        Cst816s {
            slave_address: salve_address,
            twim,
            reset_pin,
            int_channel,
            buffer: [0; 7],
        }
    }

    pub fn init<T: DelayNs>(&mut self, delay: &mut T) -> Result<(), twim::Error> {
        self.reset_pin.set_high();
        delay.delay_ms(50);
        self.reset_pin.set_low();
        delay.delay_ms(5);
        self.reset_pin.set_high();
        delay.delay_ms(50);

        let _ = self.read_register(Register::WakeUp0)?;
        delay.delay_ms(5);
        let _ = self.read_register(Register::WakeUp1)?;
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

    pub async fn wait_event(&mut self) -> Result<TouchData, twim::Error> {
        self.int_channel.wait().await;
        self.read_touch_data()
    }

    fn read_touch_data(&mut self) -> Result<TouchData, twim::Error> {
        let addr = [0];
        match self.twim.blocking_write_read_timeout(
            self.slave_address,
            &addr,
            &mut self.buffer,
            DEFAULT_READ_TIMEOUT,
        ) {
            Err(e) => Err(e),
            Ok(()) => Ok(TouchData::from_le_bytes(&self.buffer)),
        }
    }

    fn read_register(&mut self, register: Register) -> Result<u8, twim::Error> {
        let tx = [register.addr()];
        let mut rx = [0_u8; 1];
        self.twim.blocking_write_read_timeout(
            self.slave_address,
            &tx,
            &mut rx,
            DEFAULT_READ_TIMEOUT,
        )?;
        Ok(rx[0])
    }

    fn write_register(&mut self, register: Register, value: u8) -> Result<(), twim::Error> {
        let tx = [register.addr(), value];
        self.twim
            .blocking_write_timeout(self.slave_address, &tx, DEFAULT_READ_TIMEOUT)?;
        Ok(())
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
#[repr(u8)]
enum Register {
    WakeUp0 = 0x15,
    WakeUp1 = 0xA7,
    Motion = 0xEC,
    IrqCtl = 0xFA,
    //PowerMode = 0xA5,
}

impl Register {
    fn addr(self) -> u8 {
        self as u8
    }
}
