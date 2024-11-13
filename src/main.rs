#![no_std]
#![no_main]
extern crate alloc;

mod cst816s;
mod ui_task;
use core::ptr::addr_of_mut;

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{self, Pin as _, Pull};
use embassy_nrf::gpiote;
use embassy_nrf::pac::reset::{self};
use embassy_nrf::peripherals::{self};
use embassy_nrf::twim::{Frequency, Twim};
use embassy_nrf::{self, pac, twim};
use embassy_nrf::{bind_interrupts, uarte, Peripheral};
use embassy_time::{Duration, Timer};
use embedded_alloc::LlffHeap as Heap;

use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SERIAL1  => twim::InterruptHandler<peripherals::SERIAL1>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// CST816S I2C address
pub const CST816S_ADDRESS: u8 = 0x15;

/// nPM1300
pub const NPM1300_ADDRESS: u8 = 0x6B;
pub const NPM1300_DEFAULT_TIMEOUT: Duration = Duration::from_millis(10);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 40 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let p = embassy_nrf::init(Default::default());
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;

    info!("Hello rs-watch!");

    let div = pac::clock::vals::Hclk::DIV1; // Desired Main clock divider (aka 128MHz)
    pac::CLOCK.hfclkctrl().write(|w| w.set_hclk(div));
    let cpu_clock_speed = 128 / (pac::CLOCK.hfclkctrl().read().hclk() as u8 + 1);
    info!("Running at {}MHz", cpu_clock_speed);

    info!("Waking-up NET core...");

    pac::RESET
        .network()
        .forceoff()
        .write(|w| w.set_forceoff(reset::vals::Forceoff::RELEASE));

    info!("Initializing PM...");
    let mut config = twim::Config::default();
    config.frequency = Frequency::K400;
    let mut sensor_i2c = twim::Twim::new(p.SERIAL1, Irqs, p.P0_10, p.P0_07, config);
    let mut read_buf = [0u8];

    match sensor_i2c.blocking_write_read_timeout(
        NPM1300_ADDRESS,
        &0xE0_00u16.to_be_bytes(), // Error register
        &mut read_buf,
        NPM1300_DEFAULT_TIMEOUT,
    ) {
        Ok(_) => {
            info!("nPM found!");

            info!("Turning LDO1 on...");
            pm_write_reg(&mut sensor_i2c, 0x08_0C, 23).ok(); // LDSW1VOUTSEL register, set V_LDO1 to 3V3
            pm_write_reg(&mut sensor_i2c, 0x08_08, 1).ok(); // LDSW1LDOSEL register, set LDSW1 to LDO1 (VDD-Display = V_LDO1)
            pm_write_reg(&mut sensor_i2c, 0x08_00, 1).ok(); // TASKLDSW1SET register, set enable for LDO1SW = 1

            loop {
                match sensor_i2c.blocking_write_read_timeout(
                    NPM1300_ADDRESS,
                    &0x08_04u16.to_be_bytes(), // LDSWSTATUS
                    &mut read_buf,
                    NPM1300_DEFAULT_TIMEOUT,
                ) {
                    Ok(_) => {
                        warn!("Current LDO status {}", read_buf[0]);
                        if read_buf[0] & 0b1_0010 == 0b1_0010 {
                            info!("LDO1 is on!");
                            break;
                        } else {
                            info!("Still waiting for LDO1...");
                            Timer::after_millis(2).await; // Backoff a bit
                        }
                    }
                    Err(_) => {
                        error!("Communication error while setting LDO1");
                        break;
                    }
                }
            }
        }
        Err(_) => warn!("nPM module not found!"),
    };

    let touch_int: gpiote::InputChannel<'_> = gpiote::InputChannel::new(
        p.GPIOTE_CH0,
        gpio::Input::new(p.P1_00, Pull::Up),
        gpiote::InputChannelPolarity::HiToLo,
    );

    let display_hw = ui_task::DisplayHardwareInterface {
        #[cfg(feature = "hw-board-dk")]
        reset: p.P0_21.degrade(),
        #[cfg(feature = "hw-board-zs")]
        reset: p.P0_03.degrade(),
        blk: ui_task::BacklightControl {
            pin: p.P0_23.degrade(),
            pwm: p.PWM0.into_ref(),
        },
        cs: p.P0_12.degrade(),
        dc: p.P0_11.degrade(),
        mosi: p.P0_09.degrade(),
        clk: p.P0_08.degrade(),
        spi: p.SPIM4.into_ref(),
    };

    let touch_hw = ui_task::TouchHardwareInterface {
        address: CST816S_ADDRESS,
        level_shifter_enable: p.P1_01.degrade(),
        reset: p.P0_20.degrade(),
        #[cfg(feature = "hw-board-dk")]
        scl: p.P1_08.degrade(),
        #[cfg(feature = "hw-board-zs")]
        scl: p.P1_03.degrade(),
        #[cfg(feature = "hw-board-dk")]
        sda: p.P1_07.degrade(),
        #[cfg(feature = "hw-board-zs")]
        sda: p.P1_02.degrade(),
        int: touch_int,
        i2c: p.SERIAL2.into_ref(),
    };

    info!("Spawning UI task...");
    unwrap!(spawner.spawn(ui_task::ui_task_runner(display_hw, touch_hw)));

    loop {
        trace!("Main loop has still nothing to do...");
        Timer::after_secs(10).await;
    }
}

fn pm_write_reg<TWIM>(
    sensor_i2c: &mut Twim<'_, TWIM>,
    reg: u16,
    value: u8,
) -> Result<(), twim::Error>
where
    TWIM: twim::Instance,
{
    let reg = reg.to_be_bytes();
    let w = [reg[0], reg[1], value];

    sensor_i2c.blocking_write_timeout(NPM1300_ADDRESS, &w, NPM1300_DEFAULT_TIMEOUT)
}
