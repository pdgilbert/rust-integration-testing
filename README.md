# Rust Example testing 

(Rust Integration may be an overly ambitious name. And to be clear, this is about using `embedded-hal` programs on
`stm32` MCUs.)

##  Contents

See the auto-generated menu in the github README display (above right).

## Summary

The idea of this `Rust` crate-like repository is to have examples that run on different MCUs
with the same application code. Setup functions do all board/MCU/hal initialization
so the application part of each example is generic code that works with all MCUs. 

Results from compiling different examples are reported in the repository 'Actions' tab.
(https://github.com/pdgilbert/rust-integration-testing/actions)
This gives a way to see what examples actually compile and which do not. 
The workflow jobs are organized in two groups. One group, indicated `CI(...)`, runs 
"all examples that should be working" and reports success or failure for different hal/board setups. 
It stops at the first example that fails, so is not very useful until things are mostly working.
The second group, indicated `Ex?(example, board-hal)`, runs individual job steps for each example.
This is more time consuming than the `CI()` approach so it is only done for a subset of boards/hals.
The "?" in "Ex?" is replaced in the jobs as follows:
- `ExM` are examples from the `examples/misc` directory of the repository. 
Many of these are simple and self contained. If they do not work then the hal is probably broken,
or I have the setup messed up.
- `ExD` are examples from the `examples/driver-examples` directory of the repository.
These are examples using crates from https://github.com/eldruin and 
based on example from https://github.com/eldruin/driver-examples.
- `ExI` are examples from the `examples/misc-i2c-drivers` directory of the repository. 
- `ExR` are examples from the `examples/rtic` directory of the repository. 
- `ExL` are examples from the `examples/radio-sx127x` directory of the repository which has LoRa examples. 
- `ExP` are examples from the `examples/projects` directory of the repository. 
These are a bit more ambitious projects.

The jobs provide information from `cargo build` and from `cargo tree` for debugging purposes.

## Setup Notes

Different MCUs and boards have different hardware capabilities and the hal crates must reflect this. 
To accommodate these differences there are setup functions for each board/MCU. 
These pass a standardized set of devices to the application code. The standardized set (serial ports, 
I2C pins, ...) is organized to reduce  hardware changes in my testing. 
The setup is selected by environment variables discussed further below.

The examples in `examples/misc` mostly contain their own copy of the setup functions, so they are
self contained  and easier to read. 
Other examples use functions defined in `src/` (eg `setup_all_stm32g4xx.rs`) to simplify
support and maintain consistence. 

## Hardware Notes

No automatic attempt is made to link/flash/run/debug.  The repository includes files to do this 
(e.g. `.cargo/config` and `memory.x`) but these are not used in the github workflows.
A very brief indication of how to do these is further below.

Many examples have been tested on hardware at some stage, but not always with the current 
version of software. (Any test with the board marked as `none-` means I do not have a
board and so it is certain I have not tested on hardware.) 
Some of the examples contain notes about hardware testing.

## Testing Strategies

(Note that only the dev-testing strategy below is active at the moment.
The driver-testing, hal-testing, and release-testing strategies are disabled until
things become more stable with `embedded-hal 1.0`. )

The repository uses four different `Cargo.toml` strategies.
One strategy uses a recent release versions of both HAL crates and driver device crates. 
The  second  uses a recent release versions of HAL crates and github versions of driver crates. 
The   third  uses  github versions of HAL crates and a recent release versions of driver crates. 
The  fourth  uses  github versions of both HAL crates and driver crates. 

The strategies require different `Cargo.toml` (and `Cargo.lock`) files. 
These are in directories `release-testing`, `driver-testing`, `hal-testing`, and `dev-testing`.
(`Cargo` workspaces do not work for this because they have a common `Cargo.lock` file.
Basically, the logic here is completely reversed from workspaces. In this setup almost 
everything is the same except the `Cargo` files for the strategies.)

The examples themselves are the same for all strategies. 

If the first strategy (release) fails and last strategy (dev)
passes then there is an update/fix/change in the github version of a hal that is not yet
in the release version. In the reverse case there is a change in the github version that has 
not yet been incorporated into the example code.


## Notes on Some Crates and Hals

Examples in `examples/driver-examples` directory use crates from https://github.com/eldruin. 
The originals from which they are derived are very well documented at
https://github.com/eldruin/driver-examples. I found this a very good starting place for a novice. 

The `radio-sx127x` examples are based on https://github.com/rust-iot/rust-radio-sx127x.

In the transition to `embedded-hal 1.0` there is some use of forks and branches that are ahead of the
main branch of official repositories. Hopefully it is a temporary situation.
This can be checked in the Cargo.toml file.

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
  export HAL=stm32g4xx MCU=stm32g431xB TARGET=thumbv7em-none-eabihf PROC=stm32g4x  CHIP=STM32G4x  # weact-stm32g431CBU6 Cortex-M4
  #export HAL=stm32g4xx MCU=stm32g473   TARGET=thumbv7em-none-eabihf PROC=stm32g4x  CHIP=STM32G4x  # none-stm32g473      Cortex-M4
  export HAL=stm32g4xx MCU=stm32g474xE TARGET=thumbv7em-none-eabihf PROC=stm32g4x  CHIP=STM32G4x  # weact-stm32g474CEU6 Cortex-M4
  #export HAL=stm32h7xx MCU=stm32h742   TARGET=thumbv7em-none-eabihf PROC=          CHIP=          # none-stm32h742      Cortex-M7
  #export HAL=stm32h7xx MCU=stm32h743   TARGET=thumbv7em-none-eabihf PROC=          CHIP=          # devEBox-stm32h743VIT6 Cortex-M7
  export HAL=stm32h7xx MCU=stm32h750   TARGET=thumbv7em-none-eabihf PROC=          CHIP=          # devEBox-stm32h750VBT6 Cortex-M7
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

## Contributing

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.
