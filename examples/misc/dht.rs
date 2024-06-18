//!   NOT HARDWARE TESTED SINCE EMBEDDED-HAL V1.0.0 CHANGES
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
use dht_sensor::dht11::{read, Reading};
#[cfg(feature = "dht22")]
use dht_sensor::dht22::{read, Reading};
//use dht_sensor::*;

// See dht-sensor git discussion in issues #1  and #2
//https://github.com/michaelbeaumont/dht-sensor/issues/1
//Regarding pulling the pin high to avoid confusing the sensor when initializing.
//Also more in comments in dht-sensor crate file src/lib.rs


//use dht_sensor::Delay;  // trait, whereas timer::Delay is a type

use rust_integration_testing_of_examples::setup::{Peripherals, OpenDrainType, DelayNs, GpioExt, prelude::*,};
use rust_integration_testing_of_examples::setup::{CorePeripherals};


//type DhtType = PA8<Output<OpenDrain>>;
type DhtType = OpenDrainType;


#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    pac::TIM2,
    timer::{Delay as halDelay},  //TimerExt, 
    //timer::DelayMs,
    //timer::SysTimerExt,  // trait
    //rcc::Clocks,
};

#[cfg(feature = "stm32f1xx")]
type DelayMsType =  halDelay<TIM2, 1000000_u32>;  // this fails, I think because dht want DelayMs not DelayNs ???
//type DelayMsType =  TimerExt::delay_ms<TIM2, 1000000_u32>;

#[cfg(feature = "stm32f1xx")]
pub fn setup(dp: Peripherals, _cp: CorePeripherals) ->  (DhtType, DelayMsType) {
   let mut flash = dp.FLASH.constrain();
   let rcc = dp.RCC.constrain();
   let mut afio = dp.AFIO.constrain();
   let clocks = rcc.cfgr
        //.use_hse(8.mhz()) // high-speed external clock 8 MHz on bluepill
        //.sysclk(64.mhz()) // system clock 8 MHz default, max 72MHz
        //.pclk1(32.mhz())  // system clock 8 MHz default, max 36MHz ?
        .freeze(&mut flash.acr);

   let mut gpioa = dp.GPIOA.split();
   let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   //let delay = cp.SYST.delay(&clocks);  //SysDelay  System Timer  delay (SysTick)
   let delay = dp.TIM2.delay::<1000000_u32>(&clocks);

   (dht, delay)
}


#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    pac::TIM2,
    timer::Delay as halDelay,
    //timer::SysTimerExt,  // trait
    //rcc::Clocks,
};

#[cfg(feature = "stm32f4xx")]
type DelayMsType =  halDelay<TIM2, 1000000_u32>;

#[cfg(feature = "stm32f4xx")]
pub fn setup(dp: Peripherals, _cp: CorePeripherals) ->  (DhtType, DelayMsType) {
   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze();

   let gpioa = dp.GPIOA.split();
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.

   //let delay = cp.SYST.delay(&clocks);  //SysDelay  System Timer  delay (SysTick)
   let delay = dp.TIM2.delay(&clocks);

   (dht, delay)
}


#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    pac::TIM2,
    //delay::Delay as DelayMsType,
    time::ExtU32,
    timer::{Timer, CountDownTimer},
    delay::DelayFromCountDownTimer,
};

// A DelayMs type is needed for dht read (as of Jan 2024).  impl DelayNs does not work.
// This does work, but possbly should not.
#[cfg(feature = "stm32g4xx")]
pub type DelayMsType = DelayFromCountDownTimer<CountDownTimer<TIM2>>;

#[cfg(feature = "stm32g4xx")]
pub fn setup(dp: Peripherals, _cp: CorePeripherals) ->  (DhtType, DelayMsType ) {
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
    pac::TIM2,
    timer::{Timer,},
    //delay::Delay as DelayMsType,
    delay::DelayFromCountDownTimer,
//      //timer::TimerExt,
//      //rcc::CoreClocks as Clocks,
};

#[cfg(feature = "stm32h7xx")]
type DelayMsType = DelayFromCountDownTimer<Timer<TIM2>>;

#[cfg(feature = "stm32h7xx")]
pub fn setup(dp: Peripherals, _cp: CorePeripherals) ->  (DhtType, DelayMsType) {
   let pwr = dp.PWR.constrain();
   let vos = pwr.freeze();
   let rcc = dp.RCC.constrain();
   let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
   let clocks = ccdr.clocks;

   let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
   let mut dht = gpioa.pa8.into_open_drain_output();
   dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
   
   //let delay = cp.SYST.delay(clocks);  //SysDelay  System Timer  delay (SysTick)
   let timer = dp.TIM2.timer(1.Hz(), ccdr.peripheral.TIM2, &clocks);
   let delay = DelayFromCountDownTimer::new(timer);

   (dht, delay)
}


#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let (mut dht, mut delay) = setup(dp, cp);
    
    //let mut delay: impl DelayNs = cp.SYST.delay(clocks);

    // This syntax works with stm32h7xx but not with stm32f4xx, as of eh-rc3
    // let mut delay = Delay::new(cp.SYST, clocks); 

    hprintln!("Reading sensor...").unwrap();

    // single read before loop for debugging purposes
    //
    //let r = Reading::read(&mut delay, &mut dht);
    //match r {
    //        Ok(Reading {
    //            temperature,
    //            relative_humidity,
    //        }) => hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap(),
    //        Err(e) => hprintln!("Error {:?}", e).unwrap(),
    //}

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
