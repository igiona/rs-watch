#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::pac::reset::network;
use embassy_nrf::peripherals::SERIAL0;
use embassy_nrf::{bind_interrupts, uarte};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    SERIAL0 => uarte::InterruptHandler<SERIAL0>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    
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
    unsafe {
    ((0x5000_5000 +0x614) as *mut u32).write_volatile(0);
    let on = ((0x5000_5000 +0x614) as *mut u32).read_volatile();
        info!("cpu-net is : {}", on);
    }

    let mut buf = [0; 1];
    loop {
        info!("reading...");
        unwrap!(uart.read(&mut buf).await);
        info!("writing...");
        unwrap!(uart.write(&buf).await);
    }
}
