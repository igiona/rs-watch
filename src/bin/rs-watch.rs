#![no_std]
#![no_main]
extern crate alloc;

use core::ptr::addr_of_mut;

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::pac::reset::network;
use embassy_nrf::peripherals::SERIAL0;
use embassy_nrf::{bind_interrupts, uarte};
use embassy_time::{Duration, Instant, Timer};
use embedded_alloc::LlffHeap as Heap;
use slint::platform::software_renderer::MinimalSoftwareWindow;
use slint::platform::Platform;
use {defmt_rtt as _, panic_probe as _};

slint::include_modules!();

bind_interrupts!(struct Irqs {
    SERIAL0 => uarte::InterruptHandler<SERIAL0>;
});

#[global_allocator]
static HEAP: Heap = Heap::empty();

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

#[embassy_executor::task]
pub async fn ui_task_runner() {
    let window = MinimalSoftwareWindow::new(Default::default());
    slint::platform::set_platform(alloc::boxed::Box::new(ZsWatchHwPlatform {
        window: window.clone(),
    }))
    .unwrap();

    let ui = RsWatchUi::new().expect("Unable to create the main window");

    window.set_size(slint::PhysicalSize::new(240, 240));
    loop {
        // Let Slint run the timer hooks and update animations.
        slint::platform::update_timers_and_animations();

        // Check the touch screen or input device using your driver.
        // TODO
        // if let Some(event) = check_for_touch_event(/*...*/) {
        //     // convert the event from the driver into a `slint::platform::WindowEvent`
        //     // and pass it to the window.
        //     window.dispatch_event(event);
        // }

        // Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            // see next section about rendering.

            // TODO: on the nRF5340 we could even try to use a full-size framebuffer instead of line-by-line approach
            // See https://docs.rs/slint/latest/slint/docs/mcu/index.html#the-renderer
            renderer.render_by_line(&mut buffer_provider);
        });

        if !window.has_active_animations() {
            // Try to put the MCU to sleep
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                Timer::after(Duration::from_millis(duration.as_millis() as u64)).await;
                continue;
            }
        }
        // Five embassy some time to do other stuff, even though the UI want's to get refreshed
        Timer::after(Duration::from_millis(5)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize the allocator BEFORE you use it
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(addr_of_mut!(HEAP_MEM) as usize, HEAP_SIZE) }
    }

    let p = embassy_nrf::init(Default::default());
    let mut config = uarte::Config::default();
    config.parity = uarte::Parity::EXCLUDED;
    config.baudrate = uarte::Baudrate::BAUD115200;

    let mut uart = uarte::Uarte::new(p.SERIAL0, Irqs, p.P1_00, p.P1_01, config);

    info!("uarte initialized!");

    // Message must be in SRAM
    let mut buf = [0; 8];
    buf.copy_from_slice(b"Hello!\r\n");

    unwrap!(uart.write(&buf).await);
    info!("wrote hello in uart!");

    // embassy_nrf::pac::reset::network::FORCEOFF
    // let fo = network::FORCEOFF {};
    // unsafe {
    // ((0x5000_5000 +0x614) as *mut u32).write_volatile(0);
    // let on = ((0x5000_5000 +0x614) as *mut u32).read_volatile();
    //     info!("cpu-net is : {}", on);
    // }
    unsafe { &*embassy_nrf::pac::RESET::ptr() }
        .network
        .forceoff
        .write(|w| w.forceoff().variant(network::forceoff::FORCEOFF_A::RELEASE));
    //unsafe { &*embassy_nrf::pac::RESET::ptr() }.network.forceoff.write(|w| w.forceoff().bit(network::forceoff::FORCEOFF_A::RELEASE.into()));

    unwrap!(spawner.spawn(ui_task_runner()));

    let mut buf = [0; 1];
    loop {
        info!("reading...");
        unwrap!(uart.read(&mut buf).await);
        info!("writing...");
        unwrap!(uart.write(&buf).await);
    }
}
