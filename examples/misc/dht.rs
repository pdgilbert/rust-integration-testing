//!  Measure the temperature and humidity from a DHT11 sensor and print with hprintln (to
//!  a gdb session).  The DHT11 data pin is connectted to pin A8 on the MCU board and has
//!  a pull up resistor. (18K ohms used in some testing.)
//!  The largest part of this file is the setup() functions used for each hal.
//!  These make the application code common.
//!
//!  Note that '--release' is needed when doing a run test on actual hardware. Otherwise
//!  code is too slow for the timeout set in the crate and run gives 'Error Timeout'.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::Reading;
#[cfg(feature = "dht22")]
use dht_sensor::dht22::Reading;
use dht_sensor::*;

use embedded_hal::blocking::delay::DelayMs;

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

// See dht-sensor git discussion in issues #1  and #2
//https://github.com/michaelbeaumont/dht-sensor/issues/1
//Regarding pulling the pin high to avoid confusing the sensor when initializing.
//Also more in comments in dht-sensor crate file src/lib.rs

#[cfg(feature = "stm32f0xx")]
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

// open_drain_output is really input and output

#[cfg(feature = "stm32f0xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);

    let mut pa8 = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high().ok();

    let mut delay = Delay::new(cp.SYST, &rcc);

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

// open_drain_output is really input and output

#[cfg(feature = "stm32f1xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);

    // delay is used by `dht-sensor` to wait for signals
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    let mut gpioa = p.GPIOA.split();
    let mut pa8 = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    //let mut pa8 = cortex_m::interrupt::free(|cs| pa8.into_open_drain_output(cs));

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high();

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32f3xx")]
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
    let mut pa8 = gpioa
        .pa8
        .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high().ok();

    // delay is used by `dht-sensor` to wait for signals
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")] // Use HSE oscillator
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();

    //let clocks =  p.RCC.constrain().cfgr.freeze();
    // next gives panicked at 'assertion failed: !sysclk_on_pll ||
    //                  sysclk <= sysclk_max && sysclk >= sysclk_min'
    //let clocks = p.RCC.constrain().cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();
    let clocks = rcc
        .cfgr
        .hclk(48.mhz())
        .sysclk(48.mhz())
        .pclk1(24.mhz())
        .pclk2(24.mhz())
        .freeze();

    hprintln!("sysclk freq: {}", clocks.sysclk().0).unwrap();
    let mut pa8 = p.GPIOA.split().pa8.into_open_drain_output();

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high();

    // delay is used by `dht-sensor` to wait for signals
    let mut delay = Delay::new(cp.SYST, &clocks); //SysTick: System Timer

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")] // Use HSE oscillator
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let mut pa8 = p.GPIOA.split().pa8.into_open_drain_output();

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high();

    // delay is used by `dht-sensor` to wait for signals
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;

    let mut pa8 = p
        .GPIOA
        .split(ccdr.peripheral.GPIOA)
        .pa8
        .into_open_drain_output();

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high().ok();

    // delay is used by `dht-sensor` to wait for signals
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());

    //let clocks =  p.RCC.constrain().cfgr.freeze();
    // next gives panicked at 'assertion failed: !sysclk_on_pll ||
    //                  sysclk <= sysclk_max && sysclk >= sysclk_min'
    //let clocks = p.RCC.constrain().cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();
    let mut pa8 = p.GPIOA.split(&mut rcc).pa8.into_open_drain_output();

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high().ok();

    // delay is used by `dht-sensor` to wait for signals
    //let mut delay = Delay::new(cp.SYST, clocks);   //SysTick: System Timer
    let mut delay = cp.SYST.delay(rcc.clocks);

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32l1xx")]
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    prelude::*,
    rcc, // for ::Config but note name conflict with next
    stm32::{CorePeripherals, Peripherals},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());

    //let clocks = p.RCC.constrain().cfgr.use_hse(8.mhz()).sysclk(168.mhz()).freeze();
    let mut pa8 = p.GPIOA.split(&mut rcc).pa8.into_open_drain_output();

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high().ok();

    // delay is used by `dht-sensor` to wait for signals
    //let mut delay = Delay::new(cp.SYST, clocks);   //SysTick: System Timer

    let mut delay = cp.SYST.delay(rcc.clocks);

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA8, OpenDrain, Output},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (PA8<Output<OpenDrain>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    let mut pa8 = gpioa
        .pa8
        .into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

    // Pulling the pin high to avoid confusing the sensor when initializing.
    pa8.set_high();

    // delay is used by `dht-sensor` to wait for signals
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (pa8, delay) //DHT data will be on A8
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (mut dht_data, mut delay) = setup(); //dht_data is usually pa8 in setup functions

    hprintln!("Reading sensor...").unwrap();

    // single read before loop for debugging purposes
    //
    //let r = Reading::read(&mut delay, &mut dht_data);
    //match r {
    //        Ok(Reading {
    //            temperature,
    //            relative_humidity,
    //        }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
    //        Err(e) => hprintln!("Error {:?}", e).unwrap(),
    //}

    loop {
        match Reading::read(&mut delay, &mut dht_data) {
            Ok(Reading {
                temperature,
                relative_humidity,
            }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
            Err(e) => hprintln!("Error {:?}", e).unwrap(),
        }

        // (Delay at least 500ms before re-polling, 1 second or more advised)
        // Delay 5 seconds
        delay.delay_ms(5000_u16);
    }
}
