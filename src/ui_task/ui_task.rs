extern crate alloc;

use crate::cst816s::Cst816s;
use crate::ui_task::display_line_buffer_provider::DisplayWrapper;
use crate::ui_task::touch_handler::TouchHandler;
use defmt::*;
use embassy_futures::select::{self, select3};
use embassy_nrf::gpio::Output;
use embassy_nrf::peripherals::{self, PWM0};
use embassy_nrf::pwm::SimplePwm;
use embassy_nrf::twim::Frequency;
use embassy_nrf::{self, pwm, twim};
use embassy_nrf::{bind_interrupts, spim, uarte};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Delay, Duration, Instant, Timer};

use embedded_hal_bus::spi::ExclusiveDevice;
use gc9a01::mode::DisplayConfiguration;
use gc9a01::prelude::{DisplayResolution240x240, DisplayRotation};
use gc9a01::{Gc9a01, SPIDisplayInterface};
// Display info: https://www.buydisplay.com/240x240-round-ips-tft-lcd-display-1-28-capacitive-touch-circle-screen
// use mipidsi::models::GC9A01;
// use mipidsi::Display;
use super::{DisplayHardwareInterface, TouchHardwareInterface};
use slint::platform::software_renderer::{
    LineBufferProvider, MinimalSoftwareWindow, RepaintBufferType,
};
use slint::platform::Platform;

use {defmt_rtt as _, panic_probe as _};

slint::include_modules!();

bind_interrupts!(struct Irqs {
    SERIAL0 => uarte::InterruptHandler<peripherals::SERIAL0>;
    SERIAL2  => twim::InterruptHandler<peripherals::SERIAL2>;
    SPIM4  => spim::InterruptHandler<peripherals::SPIM4>;
});

const DISPLAY_WIDTH: u16 = 240;
const DISPLAY_HEIGHT: u16 = 240;

struct RsWatchPlatform {
    window: alloc::rc::Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    // timer: embassy_time::Timer,
    // ... maybe more devices
}

impl Platform for RsWatchPlatform {
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
}

// If `Platform` would implement an `async` version of the event loop, we wouldn't require this trait
trait UiEventLoopAsync {
    async fn run_async_event_loop(
        &self,
        window: alloc::rc::Rc<MinimalSoftwareWindow>,
        line_buffer_provider: impl LineBufferProvider,
    ) -> Result<(), slint::PlatformError>;
}

#[derive(Clone, Copy, Debug, Format)]
pub(crate) enum UiOperationRequestsMessage {
    // Change the display brightness
    SetBrightness { brightness_pct: u8 },
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

    let mut display = Gc9a01::new(
        interface,
        DisplayResolution240x240,
        DisplayRotation::Rotate180,
    );

    let mut config = twim::Config::default();
    config.frequency = Frequency::K400;
    // config.scl_pullup = true;
    // config.sda_pullup = true;
    config.scl_high_drive = true;
    config.sda_high_drive = true;
    let i2c = twim::Twim::new(touch_hw.i2c, Irqs, touch_hw.sda, touch_hw.scl, config);
    let mut touch_controller = Cst816s::new(
        touch_hw.address,
        i2c,
        touch_reset,
        touch_hw.int,
        display.get_screen_rotation().into(),
        DISPLAY_WIDTH,
        DISPLAY_HEIGHT,
    );

    let mut delay = Delay;

    info!("Enabling touch level shifter...");
    touch_enable.set_high(); // Enable touch interface
    Timer::after(Duration::from_millis(10)).await; // Give the HW some time to settle
    info!("Initializing touch controller...");
    if let Err(e) = touch_controller.init(&mut delay) {
        error!("Failed to initialized the touch controller! => {}", e);
    }

    info!("Resetting display...");
    display.reset(&mut display_reset, &mut delay).unwrap();
    info!("Initializing...");
    display.init(&mut delay).unwrap();

    info!("Creating UI...");

    static UI_REQUESTS_CHANNEL: Channel<ThreadModeRawMutex, UiOperationRequestsMessage, 8> =
        Channel::new();

    // Note that we use `ReusedBuffer` as parameter for MinimalSoftwareWindow to indicate
    // that we just need to re-render what changed since the last frame.
    // What's shown on the screen buffer is not in our RAM, but actually within the display itself.
    // Only the changed part of the screen will be updated.
    let window = MinimalSoftwareWindow::new(RepaintBufferType::ReusedBuffer);

    let mut line_buffer =
        [slint::platform::software_renderer::Rgb565Pixel(0); DISPLAY_WIDTH as usize];

    slint::platform::set_platform(alloc::boxed::Box::new(RsWatchPlatform {
        window: window.clone(),
    }))
    .unwrap();

    info!("Instantiate RsWatchUi");
    let ui = RsWatchUi::new();
    info!("ui is ok {}", ui.is_ok());
    let ui = ui.expect("Unable to create the main window");

    // ... setup callback and properties on `ui` ...
    ui.on_menu_item_click(|i| info!("Clicked menu #{}", i));
    ui.on_brightness_setting_changed(|b| {
        UI_REQUESTS_CHANNEL
            .try_send(UiOperationRequestsMessage::SetBrightness {
                brightness_pct: b.max(1f32).min(90f32) as u8,
            })
            .expect("Could not queue the SetBrightness message"); // TODO do better error handling here
    });

    let mut touch_handler = TouchHandler::new(window.clone());
    info!("Setting windows size...");

    window.set_size(slint::PhysicalSize::new(
        DISPLAY_WIDTH as u32,
        DISPLAY_HEIGHT as u32,
    ));

    info!("Backlight on...");
    set_display_brightness(&mut backlight, 80);

    loop {
        // Let Slint run the timer hooks and update animations.
        slint::platform::update_timers_and_animations();

        let tasks = select3(
            touch_controller.wait_event(),
            ui.run_async_event_loop(
                window.clone(),
                DisplayWrapper {
                    display: &mut display,
                    line_buffer: &mut line_buffer,
                },
            ),
            UI_REQUESTS_CHANNEL.receive(),
        );

        let select_result = tasks.await;

        match select_result {
            select::Either3::First(result) => {
                touch_handler.handle_new_touch_data(result);
            }
            select::Either3::Second(_) => {
                touch_handler.check_missed_event(&mut touch_controller);
            }
            select::Either3::Third(op) => match op {
                UiOperationRequestsMessage::SetBrightness { brightness_pct } => {
                    set_display_brightness(&mut backlight, brightness_pct)
                }
            },
        }
    }
}

impl UiEventLoopAsync for RsWatchUi {
    async fn run_async_event_loop(
        &self,
        window: alloc::rc::Rc<MinimalSoftwareWindow>,
        line_buffer_provider: impl LineBufferProvider,
    ) -> Result<(), slint::PlatformError> {
        // Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(line_buffer_provider);
        });

        let sleep_duration = if !window.has_active_animations() {
            // Try to put the MCU to sleep
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                debug!(
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

        Ok(())
    }
}

fn set_display_brightness(pwm: &mut SimplePwm<'_, PWM0>, brightness_pct: u8) {
    let brightness_pct = 100 - brightness_pct.min(100); // TODO: remove this as soon as it is clear why the PWM's duty get's 100% with a value of 0
    let duty = (((pwm.max_duty() as u32) * (brightness_pct.min(100) as u32)) / 100) as u16;
    pwm.set_duty(0, duty);
}
