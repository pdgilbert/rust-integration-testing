//!  Compiling in debug mode may disturb the timing-sensitive parts crate dht_sensor. 
//!   If Timeout error occurs THEN COMPILE --release.
//!
//!  Tested on Blackpill stm32f401 July 11, 2024. Works using  --release
//!    Using dht-sensor v0.2.1 (https://github.com/michaelbeaumont/dht-sensor?branch=main#ba6627b7)
//!    which calles  embedded-hal v0.2.7
//! 
//!  Tested on weact-stm32g474 July 11, 2024. Works using  --release
//!    Using dht-sensor v0.2.1 (https://github.com/michaelbeaumont/dht-sensor?branch=main#ba6627b7)
//!    which calles  embedded-hal v0.2.7
//!
//!  Not yet hardware tested sinc updates June 2025.
//!
//!  Examples dht, dht_rtic, and oled_dht are similar and might be consolidated sometime.
//!  
//!  Measure the temperature and humidity from a DHT11 sensor and print with hprintln (to
//!  a gdb session).  The DHT11 data pin is connectted to pin A8 on the MCU board and has
//!  a pull up resistor. (18K ohms used in some testing.)
//!  The DHT uses 3 pin connections (and has an unused 4th pin). The pins are VCC, GND, and data.
//!  The DHT data pin is connected to the MCU pin PA8 in most (all) cases. 
//!  The data pin needs a (10K) pull up resistor. The pin is specified in src/dht.rs and would be 
//!  changed there in the case of an MCU that requires a different pin to be used.
//!
//!  Note that '--release' is needed when doing a run test on actual hardware. Otherwise
//!  code is too slow for the timeout set in the crate and run gives 'Error Timeout'.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::{blocking::read, Reading};
#[cfg(feature = "dht22")]
use dht_sensor::dht22::{blocking::read, Reading};
//use dht_sensor::*;

// See dht-sensor git discussion in issues #1  and #2
//https://github.com/michaelbeaumont/dht-sensor/issues/1
//Regarding pulling the pin high to avoid confusing the sensor when initializing.
//Also more in comments in dht-sensor crate file src/lib.rs


type DhtType = DhtPin<Output<OpenDrain>>;

use embedded_hal::delay::DelayNs;
pub use nb::block;

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal as hal;

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal as hal;

#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal as hal;

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal as hal;

use hal::{
    pac::Peripherals,
    gpio::{Output, OpenDrain,  GpioExt},
    prelude::*,
};


#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    timer::TimerExt, 
    gpio::{gpioa::PA8 as DhtPin},
};

#[cfg(feature = "stm32f1xx")]
pub fn setup(dp: Peripherals) ->  (DhtType, impl DelayNs) {
   let mut flash = dp.FLASH.constrain();
   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr
        //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
        //.sysclk(64.mhz()) // system clock 8 MHz default, max 72MHz
        //.pclk1(32.mhz())  // system clock 8 MHz default, max 36MHz ?
        .freeze(&mut flash.acr);

   let mut gpioa = dp.GPIOA.split();
   let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let delay = dp.TIM2.delay::<1000000_u32>(&clocks);

   (dht, delay)
}

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    gpio::{gpioa::PA8 as DhtPin},
};

#[cfg(feature = "stm32f4xx")]
pub fn setup(dp: Peripherals) ->  (DhtType, impl DelayNs) {
   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   let gpioa = dp.GPIOA.split();
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let delay = dp.TIM2.delay::<1000000_u32>(&clocks);

   (dht, delay)
}


#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    gpio::{gpiob::PB7 as DhtPin},
    time::ExtU32,
    timer::Timer,
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32g4xx")]
pub fn setup(dp: Peripherals) ->  (DhtType, impl DelayNs ) {
   let mut rcc = dp.RCC.constrain();

   let gpiob = dp.GPIOB.split(&mut rcc);
   let mut dht = gpiob.pb7.into_open_drain_output();
   let _ = dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   let clocks = rcc.clocks; 

   let timer1 = Timer::new(dp.TIM2, &clocks);
   let delay = DelayFromCountDownTimer::new(timer1.start_count_down(100.millis()));

   (dht, delay)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    gpio::{gpioa::PA8 as DhtPin},
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32h7xx")]
pub fn setup(dp: Peripherals) ->  (DhtType, impl DelayNs) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
   
   let timer = dp.TIM2.timer(1.Hz(), ccdr.peripheral.TIM2, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   (dht, delay)
}


#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let (mut dht, mut delay) = setup(dp);
    
    // delay to ensure time between setup set_high() and sensor read.
    delay.delay_ms(1000); 

    hprintln!("Reading sensor...").unwrap();

    // single read before loop for debugging purposes
    
    let r = read(&mut delay, &mut dht);
    match r {
            Ok(Reading {
                temperature,
                relative_humidity,
            }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
            Err(e) => hprintln!("Error {:?}", e).unwrap(),
    }
    delay.delay_ms(5000); 

    loop {
        match read(&mut delay, &mut dht) {
            Ok(Reading {
                temperature,
                relative_humidity,
            }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
            Err(e) => hprintln!("Error {:?}", e).unwrap(),
        }

        // (Delay at least 500ms before re-polling, 1 second or more is advised)
        // Delay 5 seconds
        delay.delay_ms(5000); 
    }
}
