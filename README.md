# rs-watch
rs-watch - the Open Source Rust &amp; embassy based Smartwatch, based on the ZSWatch HW.

# Development
## Setup

```shell
rustup target add thumbv8m.main-none-eabihf
```

## nrf5340-DK

In order to communicate the first time with the MCU, you need to perform a full erase of the chip to clean the permission settings:

```shell
probe-rs erase --chip nRF5340_xxAA --allow-erase-all
```

Regardless of this command times-out, without cutting the power to the MCU, flash the pre-built network core application:

```shell
probe-rs download --chip nRF5340_xxAA --binary-format hex --probe 1209:4853 binaries/zswatch_nrf5340_CPUNET.hex 
```

Now, you can perform the standard `cargo run` command to flash the application core (the `chip` and other parameters are stored in `.cargo/config.toml` as default):

```shell
cargo run
```
