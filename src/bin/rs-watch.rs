#![no_std]
#![no_main]
extern crate alloc;

mod cst816s;

use core::ptr::addr_of_mut;

use cst816s::Cst816s;
use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{self, select};
use embassy_nrf::gpio::{self, AnyPin, Output, Pin as _, Pull};
use embassy_nrf::gpiote;
use embassy_nrf::pac::reset::{self};
use embassy_nrf::peripherals::{self, PWM0};
use embassy_nrf::pwm::SimplePwm;
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
use slint::platform::Platform;
use {defmt_rtt as _, panic_probe as _};

slint::include_modules!();

bind_interrupts!(struct Irqs {
    SERIAL0 => uarte::InterruptHandler<peripherals::SERIAL0>;
    SERIAL2  => twim::InterruptHandler<peripherals::SERIAL2>;
    SERIAL3  => spim::InterruptHandler<peripherals::SERIAL3>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// CST816S I2C address
pub const CST816S_ADDRESS: u8 = 0x15;

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
    let spim: spim::Spim<'_, peripherals::SERIAL3> = spim::Spim::new_txonly(
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

    let config = twim::Config::default();
    // config.scl_pullup = true;
    // config.sda_pullup = true;
    // config.scl_high_drive = true;
    // config.sda_high_drive = true;
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
        DisplayRotation::Rotate90,
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

            if !window.has_active_animations() {
                // Try to put the MCU to sleep
                if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                    info!("Sleep...");
                    Timer::after(Duration::from_millis(duration.as_millis() as u64)).await;
                }
            } else {
                // Give embassy some time to do other stuff, even though the UI want's to get refreshed
                Timer::after(Duration::from_millis(5)).await;
            }
        });

        if let select::Either::First(result) = tasks.await {
            match result {
                Ok(event) => info!("Touch detected {:?}", event),
                Err(e) => error!("Touch read error => {}", e),
            };
        }
    }
}

struct _DisplayHardwareInterface<SPI: embedded_hal::spi::SpiDevice> {
    reset: AnyPin,
    blk: AnyPin,
    cs: AnyPin,
    dc: AnyPin,
    spi: SPI,
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
    spi: PeripheralRef<'a, peripherals::SERIAL3>,
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
        const HEAP_SIZE: usize = 16 * 1024;
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

    info!("Waking-up NET core...");

    pac::RESET
        .network()
        .forceoff()
        .write(|w| w.set_forceoff(reset::vals::Forceoff::RELEASE));

    info!("Configuring SPI");

    let mut config = spim::Config::default();
    config.frequency = spim::Frequency::M32;

    let touch_int: gpiote::InputChannel<'_> = gpiote::InputChannel::new(
        p.GPIOTE_CH0,
        gpio::Input::new(p.P1_00, Pull::Up),
        gpiote::InputChannelPolarity::HiToLo,
    );

    let display_hw = DisplayHardwareInterface {
        reset: p.P0_21.degrade(), // nrf5340 DK
        // reset: p.P0_03.degrade(), //zs-WatchHw
        blk: BacklightControl {
            pin: p.P0_23.degrade(),
            pwm: p.PWM0.into_ref(),
        },
        cs: p.P0_12.degrade(),
        dc: p.P0_11.degrade(),
        mosi: p.P0_09.degrade(),
        clk: p.P0_08.degrade(),
        spi: p.SERIAL3.into_ref(),
    };

    let touch_hw = TouchHardwareInterface {
        level_shifter_enable: p.P1_01.degrade(),
        reset: p.P0_20.degrade(),
        scl: p.P1_08.degrade(), // nrf5340 DK
        // scl: p.P1_03.degrade(),//zs-WatchHw
        sda: p.P1_07.degrade(), // nrf5340 DK
        // sda: p.P1_02.degrade(), //zs-WatchHw
        int: touch_int,
        i2c: p.SERIAL2.into_ref(),
    };

    info!("Spawning UI task...");
    unwrap!(spawner.spawn(ui_task_runner(display_hw, touch_hw)));

    loop {
        // info!("Main loop has still nothing to do...");
        Timer::after_secs(1).await;
    }
}

// struct MySpim<'a, T: Instance> {
//     spim: spim::Spim<'a, T>,
// }

// impl<'a> embedded_hal::spi::SpiDevice for MySpim<'a, T: Instance> {}
