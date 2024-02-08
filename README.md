# Rust Example testing 

(Rust Integration may be an overly ambitious name. And to be clear, this is about using `embedded-hal` programs on
`stm32` MCUs.)

##  Contents
- [Summary](#summary)
- [Building](#building)
- [Loading](#loading)

## Summary

The idea of this `Rust` crate-like setup is to have sets of examples that are run 
with four different `Cargo.toml` strategies.
One strategy uses a recent release versions of both HAL crates and driver device crates. 
The  second  uses a recent release versions of HAL crates and github versions of driver crates. 
The   third  uses  github versions of HAL crates and a recent release versions of driver crates. 
The  fourth  uses  github versions of both HAL crates and driver crates. 
(Disabled driver-testing, hal-testing, and release-testing. The dev-testing strategy is sufficient for now.)

The examples themselves are the same for all strategies, with the exception that breaking 
updates might be accommodated.  However, in general it is not necessary to accommodate
breaking changes. They will simply show as failing in some tests. 
For instance, the hal module name change from `stm32` to `pac` is changed to the newer
convention in the example code. Until the change is moved into release the tests using 
the release version show as failing. 

The `setup()` functions in each example do all board/MCU/hal 
setup so the application part of each example is generic code that works with all setups. 

The stategies require different `Cargo.toml` (and `Cargo.lock`) files. 
These are in directories `release-testing`, `driver-testing`, `hal-testing`, and `dev-testing`.
(`Cargo` workspaces do not work for this because they have a common `Cargo.lock` file.
Basically, the logic here is completely reversed from workspaces. In this setup almost 
everything is the same except the `Cargo` files for the strategies.)
 
The main work of checking that examples build is done by the CI workflow. 
See https://github.com/pdgilbert/rust-integration-testing/actions  for the latest results.
The first set of jobs runs all working examples with each of the four strategies on each of
the board (MCU/hardware) setups. If the first stategy (release) fails and last stategy (dev)
passes then there is an update/fix/change in the github version of a hal that is not yet
in the release version. In the reverse case there is a change in the gihub version that has 
not yet been incorporated into the example code.

There is a second set of jobs, one for each example, that run only on `bluepill`.
This is for testing examples without stopping at the first one that fails. It includes
some examples that are not yet working and are not included in the first set of jobs.

No attempt is made to link/flash/run/debug so `.cargo/config` and `memory.x` are not needed.
A very brief indication of how to do these is below.

The link above shows build results for all examples on several boards and with various 
device HALs. Some of them have
been tested on hardware. (Any test board marked as `none-` below means I do not have a
board and so it is certain I have not tested on hardware.)

The `driver-examples` are based on https://github.com/eldruin/driver-examples and more 
information about them is available at that web site and in blog posts referenced there.

The `radio-sx127x` examples are based on https://github.com/rust-iot/rust-radio-sx127x and
fork https://github.com/pdgilbert/rust-radio-sx127x. These example are of special interest
because the `radio-sx127x` crate is based on `embedded-hal 1.0.0-alpha-4` and uses a
compatability plugin `embedded-hal-compat` to work with hal and other crates that are
using `embedded-hal-0.2.4`.

## Building

The examples can be built manually by setting one of these lines:
```
               environment variables for cargo                       openocd         embed        test board and processor
  _____________________________________________________________     _____________  _____________   ___________________________
  export HAL=stm32f0xx MCU=stm32f030xc TARGET=thumbv6m-none-eabi    PROC=stm32f0x  CHIP=STM32F0x  # none-stm32f030      Cortex-M0
  export HAL=stm32f0xx MCU=stm32f042   TARGET=thumbv6m-none-eabi    PROC=stm32f0x  CHIP=STM32F0x  # none-stm32f042      Cortex-M0
  export HAL=stm32f1xx MCU=stm32f103   TARGET=thumbv7m-none-eabi    PROC=stm32f1x  CHIP=STM32F103C8  # bluepill         Cortex-M3
  export HAL=stm32f1xx MCU=stm32f100   TARGET=thumbv7m-none-eabi    PROC=stm32f1x  CHIP=STM32F1x  # none-stm32f100      Cortex-M3
  export HAL=stm32f1xx MCU=stm32f101   TARGET=thumbv7m-none-eabi    PROC=stm32f1x  CHIP=STM32F1x  # none-stm32f101      Cortex-M3
  export HAL=stm32f3xx MCU=stm32f303xc TARGET=thumbv7em-none-eabihf PROC=stm32f3x  CHIP=STM32F3x  # discovery-stm32f303 Cortex-M3
  export HAL=stm32f4xx MCU=stm32f401   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  CHIP=STM32F4x  # blackpill-stm32f401 Cortex-M4
  export HAL=stm32f4xx MCU=stm32f411   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  CHIP=STM32F4x  # blackpill-stm32f411 Cortex-M4
  export HAL=stm32f4xx MCU=stm32f411   TARGET=thumbv7em-none-eabihf PROC=stm32f4x  CHIP=STM32F4x  # nucleo-64           Cortex-M4
  export HAL=stm32f7xx MCU=stm32f769   TARGET=thumbv7em-none-eabihf PROC=stm32f7x  CHIP=STM32F7x  # none-stm32f769      Cortex-M7
  (                    MCU=stm32f722    no adc breaks some examples  }
  export HAL=stm32g0xx MCU=stm32g081   TARGET=thumbv6m-none-eabi    PROC=stm32g0   CHIP=STM32G071 # none-stm32g071      Cortex-M0
  #export HAL=stm32g4xx MCU=stm32g473   TARGET=thumbv7em-none-eabihf PROC=stm32g4x  CHIP=STM32G4x  # none-stm32g473      Cortex-M4
  export HAL=stm32g4xx MCU=stm32g474   TARGET=thumbv7em-none-eabihf PROC=stm32g4x  CHIP=STM32G4x  # none-stm32g474      Cortex-M4
  export HAL=stm32h7xx MCU=stm32h742   TARGET=thumbv7em-none-eabihf PROC=          CHIP=          # none-stm32h742      Cortex-M7
  #export HAL=stm32l0xx MCU=stm32l042     TARGET=thumbv6m-none-eabi    PROC=stm32l0   CHIP=STM32L0   # none-stm32l042      Cortex-M0
  export  HAL=stm32l0xx MCU=stm32l0x2kztx TARGET=thumbv6m-none-eabi    PROC=stm32l0   CHIP=STM32L072KZTx # none-stm32l072  Cortex-M0
  #export HAL=stm32l0xx MCU=stm32l053r8tx TARGET=thumbv6m-none-eabi    PROC=stm32l0   CHIP=STM32L053R8Tx   # none-stm32l053 Cortex-M0
  export HAL=stm32l1xx MCU=stm32l100   TARGET=thumbv7m-none-eabi    PROC=stm32l1   CHIP=STM32L1   # discovery-stm32l100 Cortex-M3
  export HAL=stm32l1xx MCU=stm32l151   TARGET=thumbv7m-none-eabi    PROC=stm32l1   CHIP=STM32L1   # heltec-lora-node151 Cortex-M3
  export HAL=stm32l4xx MCU=stm32l422   TARGET=thumbv7em-none-eabi   PROC=stm32l4x  CHIP=STM32L4x  # none-stm32l4x1      Cortex-M4
```
then to build
```
cargo build --no-default-features --target $TARGET --features $MCU,$HAL --example xxx
```
where `xxx` is replaced by one of the example names, such as
```
cargo build --no-default-features --target $TARGET --features $MCU,$HAL --example ads1015-adc-display
cargo build --no-default-features --target $TARGET --features $MCU,$HAL --example max17043-battery-monitor-display
cargo build --no-default-features --target $TARGET --features $MCU,$HAL --example lora_spi_send

```
See directories in `examples/` or `Cargo.toml` for example names.

## Loading

The build testing here does not include linking, flashing, running, or any actual
hardware testing. Details about this process can be found elsewhere, briefly:

If `cargo-embed`, `probe-rs` and an appropriate probe are in place then 
```
cargo embed  --target $TARGET  --features $HAL,$MCU  --chip $CHIP --example xxx
```
where `xxx` is one of the examples, such as
```
cargo embed  --target $TARGET  --features $HAL,$MCU  --chip $CHIP --example veml6070-uv-display-bp
```

If `openocd`, `gdb`, `.cargo/config` with needed runners, and an appropriate probe are 
in place then in one window run
```
openocd -f interface/$INTERFACE.cfg -f target/$PROC.cfg 
```
where INTERFACE is set for your probe, for example, `export INTERFACE=stlink-v2` for a typical cheap dongle
or `export INTERFACE=stlink-v2-1` for a slightly newer version.
In another window do
```
cargo  run --target $TARGET --features $HAL,$MCU --example xxx  [ --release]
```
The `--release` will be needed if code is too big for memory.

For examples that need a serial connection run a terminal session such as 
```
minicom -D /dev/ttyUSB0 -b9600
```
where 0 is replaced by the active USB connection number which can be found with ` dmesg | grep -i tty `

## License

Licensed under either of

 * Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

at your option.

### Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
