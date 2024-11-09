#![no_std]
#![no_main]
extern crate alloc;

mod cst816s;

use core::ptr::addr_of_mut;

use cst816s::{Cst816s, TouchData};
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{self, select};
use embassy_nrf::gpio::{self, AnyPin, Output, Pin as _, Pull};
use embassy_nrf::gpiote;
use embassy_nrf::pac::reset::{self};
use embassy_nrf::peripherals::{self, PWM0};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::twim::{Frequency, Twim};
use embassy_nrf::{self, pac, pwm, twim};
use embassy_nrf::{bind_interrupts, spim, uarte, Peripheral, PeripheralRef};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_alloc::LlffHeap as Heap;

use embedded_graphics::pixelcolor::raw::RawU16;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

use embedded_graphics::primitives::Rectangle;
use embedded_hal_bus::spi::ExclusiveDevice;
use gc9a01::mode::DisplayConfiguration;
use gc9a01::prelude::{DisplayResolution240x240, DisplayRotation};
use gc9a01::{Gc9a01, SPIDisplayInterface};
// Display info: https://www.buydisplay.com/240x240-round-ips-tft-lcd-display-1-28-capacitive-touch-circle-screen
// use mipidsi::models::GC9A01;
// use mipidsi::Display;
use slint::platform::software_renderer::{
    LineBufferProvider, MinimalSoftwareWindow, RepaintBufferType, Rgb565Pixel,
};
use slint::platform::{Platform, PointerEventButton, WindowEvent};
use {defmt_rtt as _, panic_probe as _};

slint::include_modules!();

bind_interrupts!(struct Irqs {
    SERIAL0 => uarte::InterruptHandler<peripherals::SERIAL0>;
    SERIAL1  => twim::InterruptHandler<peripherals::SERIAL1>;
    SERIAL2  => twim::InterruptHandler<peripherals::SERIAL2>;
    SPIM4  => spim::InterruptHandler<peripherals::SPIM4>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// CST816S I2C address
pub const CST816S_ADDRESS: u8 = 0x15;

/// nPM1300
pub const NPM1300_ADDRESS: u8 = 0x6B;
pub const NPM1300_DEFAULT_TIMEOUT: Duration = Duration::from_millis(10);

struct ZsWatchHwPlatform {
    window: alloc::rc::Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    // timer: embassy_time::Timer,
    // ... maybe more devices
}

impl Platform for ZsWatchHwPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        Instant::now()
            .duration_since(Instant::from_micros(0))
            .into()
    }
    // optional: You can put the event loop there, or in the main function, see later
    // fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
    //     todo!();
    // }
}

struct DisplayWrapper<'a, T> {
    display: &'a mut T,
    line_buffer: &'a mut [Rgb565Pixel],
}

impl<T: DrawTarget<Color = Rgb565>> LineBufferProvider for DisplayWrapper<'_, T> {
    type TargetPixel = Rgb565Pixel;
    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Render into the line
        render_fn(&mut self.line_buffer[range.clone()]);

        // Send the line to the screen using DrawTarget::fill_contiguous
        self.display
            .fill_contiguous(
                &Rectangle::new(
                    Point::new(range.start as _, line as _),
                    Size::new(range.len() as _, 1),
                ),
                self.line_buffer[range.clone()]
                    .iter()
                    .map(|p| RawU16::new(p.0).into()),
            )
            .map_err(drop)
            .unwrap();
    }
}

fn set_display_brightness(pwm: &mut SimplePwm<'_, PWM0>, brightness_pct: u8) {
    let brightness_pct = 100 - brightness_pct.min(100); // TODO: remove this as soon as it is clear why the PWM's duty get's 100% with a value of 0
    let duty = (((pwm.max_duty() as u32) * (brightness_pct.min(100) as u32)) / 100) as u16;
    pwm.set_duty(0, duty);
}

#[embassy_executor::task]
pub async fn ui_task_runner(
    display_hw: DisplayHardwareInterface<'static>,
    touch_hw: TouchHardwareInterface<'static>,
) {
    info!("Hello UI task...");

    info!("Initializing display &  touch ...");
    // TODO Use a PWM to drive the backlight
    let mut backlight = SimplePwm::new_1ch(display_hw.blk.pwm, display_hw.blk.pin);
    backlight.set_prescaler(pwm::Prescaler::Div128);
    backlight.set_duty(0, 0);
    backlight.set_max_duty(1024);
    backlight.set_ch0_drive(embassy_nrf::gpio::OutputDrive::HighDrive);
    set_display_brightness(&mut backlight, 0);

    let mut display_reset: Output = embassy_nrf::gpio::Output::new(
        display_hw.reset,
        embassy_nrf::gpio::Level::Low,
        embassy_nrf::gpio::OutputDrive::Standard,
    );
    let display_dc: Output = embassy_nrf::gpio::Output::new(
        display_hw.dc,
        embassy_nrf::gpio::Level::Low,
        embassy_nrf::gpio::OutputDrive::HighDrive,
    );
    let mut touch_enable: Output = embassy_nrf::gpio::Output::new(
        touch_hw.level_shifter_enable,
        embassy_nrf::gpio::Level::Low,
        embassy_nrf::gpio::OutputDrive::Standard,
    );
    let touch_reset: Output = embassy_nrf::gpio::Output::new(
        touch_hw.reset,
        embassy_nrf::gpio::Level::High,
        embassy_nrf::gpio::OutputDrive::Standard,
    );

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M32;
    config.mode = spim::MODE_0;
    let spim: spim::Spim<'_, peripherals::SPIM4> = spim::Spim::new_txonly(
        display_hw.spi,
        Irqs,
        display_hw.clk,
        display_hw.mosi,
        config,
    );

    let display_cs: Output = embassy_nrf::gpio::Output::new(
        display_hw.cs,
        embassy_nrf::gpio::Level::High,
        embassy_nrf::gpio::OutputDrive::HighDrive,
    );
    let exclusive_spim = ExclusiveDevice::new(spim, display_cs, Delay)
        .expect("The SPIM creation should be successful");

    let interface = SPIDisplayInterface::new(exclusive_spim, display_dc);

    let mut config = twim::Config::default();
    config.frequency = Frequency::K400;
    // config.scl_pullup = true;
    // config.sda_pullup = true;
    config.scl_high_drive = true;
    config.sda_high_drive = true;
    let i2c = twim::Twim::new(touch_hw.i2c, Irqs, touch_hw.sda, touch_hw.scl, config);
    let mut touch_controller = Cst816s::new(CST816S_ADDRESS, i2c, touch_reset, touch_hw.int);

    let mut delay = Delay;

    info!("Enabling touch level shifter...");
    touch_enable.set_high(); // Enable touch interface
    Timer::after(Duration::from_millis(10)).await; // Give the HW some time to settle
    info!("Initializing touch controller...");
    if let Err(e) = touch_controller.init(&mut delay) {
        error!("Failed to initialized the touch controller! => {}", e);
    }

    let mut display = Gc9a01::new(
        interface,
        DisplayResolution240x240,
        DisplayRotation::Rotate180,
    )
    .into_buffered_graphics();

    info!("Resetting display...");
    display.reset(&mut display_reset, &mut delay).unwrap();
    info!("Initializing...");
    display.init(&mut delay).unwrap();
    display.flush().unwrap();

    info!("Creating UI...");

    // Note that we use `ReusedBuffer` as parameter for MinimalSoftwareWindow to indicate
    // that we just need to re-render what changed since the last frame.
    // What's shown on the screen buffer is not in our RAM, but actually within the display itself.
    // Only the changed part of the screen will be updated.
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);
    // let window = MinimalSoftwareWindow::new(Default::default());

    slint::platform::set_platform(alloc::boxed::Box::new(ZsWatchHwPlatform {
        window: window.clone(),
    }))
    .unwrap();

    info!("Instantiate RsWatchUi");
    let ui = RsWatchUi::new();
    info!("ui is ok {}", ui.is_ok());
    let _ui = ui.expect("Unable to create the main window");

    // ... setup callback and properties on `ui` ...

    info!("Setting windows size...");

    const DISPLAY_WIDTH: u32 = 240;
    const DISPLAY_HEIGHT: u32 = 240;
    let mut line_buffer =
        [slint::platform::software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH as usize];

    window.set_size(slint::PhysicalSize::new(DISPLAY_WIDTH, DISPLAY_HEIGHT));

    info!("Backlight on...");
    set_display_brightness(&mut backlight, 80);

    let mut touch_notified_to_ui = false;
    let mut last_touch: Option<Instant> = None;
    loop {
        // Let Slint run the timer hooks and update animations.
        slint::platform::update_timers_and_animations();

        let tasks = select(touch_controller.wait_event(), async {
            // Draw the scene if something needs to be drawn.
            window.draw_if_needed(|renderer| {
                renderer.render_by_line(DisplayWrapper {
                    display: &mut display,
                    line_buffer: &mut line_buffer,
                });
                display.flush().unwrap();
            });

            let sleep_duration = if !window.has_active_animations() {
                // Try to put the MCU to sleep
                if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                    info!(
                        "Sleep until next UI update in {}ms...",
                        duration.as_millis()
                    );
                    Some(Duration::from_millis(duration.as_millis() as u64))
                } else {
                    None
                }
            } else {
                None
            };

            Timer::after(sleep_duration.unwrap_or({
                // If Slint doesn't give us time,
                // we give embassy anyways some time to do other stuff at the cost of a slightly less responsive UI
                Duration::from_millis(3)
            }))
            .await;
        });

        if let select::Either::First(result) = tasks.await {
            match result {
                Ok(touch_data) => {
                    let event: WindowEvent;
                    (touch_notified_to_ui, event) = evaluate_touch_data(
                        touch_data,
                        window.scale_factor(),
                        touch_notified_to_ui,
                    );
                    match event {
                        WindowEvent::PointerPressed {
                            position: _,
                            button: _,
                        } => last_touch = Some(Instant::now()),
                        WindowEvent::PointerReleased {
                            position: _,
                            button: _,
                        } => last_touch = None,
                        _ => {}
                    }
                    window.dispatch_event(event);
                }
                Err(e) => error!("Touch read error => {}", e),
            };
        } else if let Some(touch_instant) = last_touch {
            // Check if the touch wasn't released for long, it seems like we miss interrupts.
            // In this way we can be sure that we properly detect release actions.
            if Instant::now()
                .checked_duration_since(touch_instant)
                .unwrap_or(Duration::from_secs(0))
                > Duration::from_millis(50)
            {
                if let Ok(touch_data) = touch_controller.read_touch_data() {
                    let event: WindowEvent;
                    (touch_notified_to_ui, event) = evaluate_touch_data(
                        touch_data,
                        window.scale_factor(),
                        touch_notified_to_ui,
                    );
                    match event {
                        WindowEvent::PointerPressed {
                            position: _,
                            button: _,
                        } => {
                            debug!("It appears that we didn't miss the release event");
                            last_touch = Some(Instant::now());
                        }
                        WindowEvent::PointerReleased {
                            position: _,
                            button: _,
                        } => {
                            warn!("It appears we did really miss the release event");
                            last_touch = None;
                        }
                        _ => {}
                    }
                    window.dispatch_event(event);
                }
            }
        }
    }
}

fn evaluate_touch_data(
    touch_data: TouchData,
    scale_factor: f32,
    mut touch_notified_to_ui: bool,
) -> (bool, WindowEvent) {
    let touch_position =
        slint::PhysicalPosition::new(touch_data.x as _, touch_data.y as _).to_logical(scale_factor);
    let event = if touch_data.is_touching {
        if !touch_notified_to_ui {
            touch_notified_to_ui = true;
            slint::platform::WindowEvent::PointerPressed {
                position: touch_position,
                button: PointerEventButton::Left,
            }
        } else {
            slint::platform::WindowEvent::PointerMoved {
                position: touch_position,
            }
        }
    } else {
        touch_notified_to_ui = false;
        slint::platform::WindowEvent::PointerReleased {
            position: touch_position,
            button: PointerEventButton::Left,
        }
    };
    (touch_notified_to_ui, event)
}

struct BacklightControl {
    pin: AnyPin,
    pwm: PeripheralRef<'static, PWM0>,
}

struct DisplayHardwareInterface<'a> {
    reset: AnyPin,
    blk: BacklightControl,
    cs: AnyPin,
    dc: AnyPin,
    mosi: AnyPin,
    clk: AnyPin,
    spi: PeripheralRef<'a, peripherals::SPIM4>,
}

struct TouchHardwareInterface<'a> {
    level_shifter_enable: AnyPin,
    reset: AnyPin,
    scl: AnyPin,
    sda: AnyPin,
    int: gpiote::InputChannel<'a>,
    i2c: PeripheralRef<'a, peripherals::SERIAL2>,
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 24 * 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let p = embassy_nrf::init(Default::default());
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;

    // let mut uart = uarte::Uarte::new(p.SERIAL0, Irqs, p.P1_00, p.P1_01, config);
    // info!("uarte initialized!");
    // Message must be in SRAM
    // let mut buf = [0; 8];
    // buf.copy_from_slice(b"Hello!\r\n");
    // unwrap!(uart.write(&buf).await);
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

    let display_hw = DisplayHardwareInterface {
        #[cfg(feature = "hw-board-dk")]
        reset: p.P0_21.degrade(),
        #[cfg(feature = "hw-board-zs")]
        reset: p.P0_03.degrade(),
        blk: BacklightControl {
            pin: p.P0_23.degrade(),
            pwm: p.PWM0.into_ref(),
        },
        cs: p.P0_12.degrade(),
        dc: p.P0_11.degrade(),
        mosi: p.P0_09.degrade(),
        clk: p.P0_08.degrade(),
        spi: p.SPIM4.into_ref(),
    };

    let touch_hw = TouchHardwareInterface {
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
    unwrap!(spawner.spawn(ui_task_runner(display_hw, touch_hw)));

    loop {
        debug!("Main loop has still nothing to do...");
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
