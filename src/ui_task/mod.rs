mod display_line_buffer_provider;
mod extensions;
mod touch_handler;
mod ui_task;
use embassy_nrf::{gpio::AnyPin, gpiote, peripherals, PeripheralRef};
pub(crate) use ui_task::ui_task_runner;

pub(crate) struct BacklightControl {
    pub pin: AnyPin,
    pub pwm: PeripheralRef<'static, peripherals::PWM0>,
}

pub(crate) struct DisplayHardwareInterface<'a> {
    pub reset: AnyPin,
    pub blk: BacklightControl,
    pub cs: AnyPin,
    pub dc: AnyPin,
    pub mosi: AnyPin,
    pub clk: AnyPin,
    pub spi: PeripheralRef<'a, peripherals::SPIM4>,
}

pub(crate) struct TouchHardwareInterface<'a> {
    pub address: u8,
    pub level_shifter_enable: AnyPin,
    pub reset: AnyPin,
    pub scl: AnyPin,
    pub sda: AnyPin,
    pub int: gpiote::InputChannel<'a>,
    pub i2c: PeripheralRef<'a, peripherals::SERIAL2>,
}
