# rs-watch
rs-watch - the Open Source Rust &amp; embassy based Smartwatch, based on the ZSWatch HW.

# Development
## Setup

```shell
rustup target add thumbv8m.main-none-eabihf
```

## nrf5340-DK

In order to communicate the first time with the MCU, you need to perform a full erase of the chip to clean the permission settings

```shell
probe-rs erase --chip nRF5340_xxAA --allow-erase-all
```