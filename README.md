# rs-watch
rs-watch - the Open Source rust &#x1F980; + [embassy](https://github.com/embassy-rs) &#x1FAF6; + [Slint](https://github.com/slint-ui) &#x1F680; based smartwatch, running on the [ZSWatch-HW](https://github.com/jakkra/ZSWatch-HW).

## Disclaimer
All credits for the ZSWatch-HW goes to contributors of that project, as well as all HW related question.<br>
They have a Discord channel for questions, as well as a very [good documentation](https://github.com/jakkra/ZSWatch/wiki).

The primary goal of this project is not to become a fully featured watch, but to make it a platform to learn the technologies it's build upon.

In most of the cases, I'm no trying to re-invent the wheel, hence I gladly make use of already freely available software and drivers.<br>
Whenever needed, updates and fixes are done to best suite the `async` nature of the rs-watch firmware.

# Current state

- [x] Display & Brightness control
- [x] Touch controller
- [x] Slint-UI integration
- [ ] (ongoing) UI design and example implementations
- [ ] BLE integration (using trouble)
- [ ] External flash
- [ ] RTC
- [ ] Integration of other sensors
- [ ] Power management
- [ ] ... and more

TODO video

# Development

## FW Debugging
## Setup

```shell
rustup target add thumbv8m.main-none-eabihf
cargo install probe-rs
```

Required VsCode slint extension:
```shell
code --install-extension Slint.slint
```

## First HW bring-up

In order to communicate for the first time with the MCU, you need to perform a full erase of the chip to clean the permission settings:

```shell
probe-rs erase --chip nRF5340_xxAA --allow-erase-all
```

Regardless of this command times-out, **without cutting the power on the MCU**, flash the pre-built network core application:

```shell
probe-rs download --chip nRF5340_xxAA --binary-format hex --probe 1209:4853 binaries/zswatch_nrf5340_CPUNET.hex 
```

## ZSWatch-HW
### Build and run

In order to build & run the code for the DK:
```shell
cargo run --bin rs-watch --release

```
## nrf5340-DK

### Display
The following display with integrated touch-controlled works well: [ER-TFT1.28-2-5670-5407](https://www.buydisplay.com/240x240-round-ips-tft-lcd-display-1-28-capacitive-touch-circle-screen)

### Pin configuration
The DK doesn't provide access to certain pins (without HW changes).<br>
Therefore these pins are mapped slightly differently as the ZSWatch-HW.

| Description          | ZSWatch-HW | nrf5340-DK |
| :------------------- | :--------: | :--------: |
| Display reset        |   P0.03    |   P0.21    |
| Touch controller SCL |   P1.03    |   P1.08    |
| Touch controller SDA |   P1.02    |   P1.07    |

*All other pins are matching the ZSWatch-HW (v5)*

### Build and run
In order to build & run the code for the DK:
```shell
cargo run --bin rs-watch --release --no-default-features --features hw-board-dk
```

Now, you can perform the standard `cargo run` command to flash the application core (the `chip` and other parameters are stored in `.cargo/config.toml` as default):

```shell
cargo run
```

## Drivers
### Touch controller
The driver for the `cst816s` touch controller is based on the work done on the [pinetime-rs](https://github.com/jonlamb-gh/pinetime-rs/) project.<br>
I only made the required adaptions to make it `embassy` and `async` friendly.