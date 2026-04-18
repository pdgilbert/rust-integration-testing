//! Continuously read temperature from AHT30 and with semihosting hprintln.
//!
//! The "semi" examples simplify testing of the sensor crate alone, without display complications.

//! Using crate aht30 which supports aht10, aht20, aht25, aht30, aht40, 

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_semihosting::hprintln;
//use cortex_m::asm;

use aht30::{AHT20_DEFAULT_ADDR as AHT30_DEFAULT_ADDR, Aht20 as Aht30};

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

/////////////////////   hals

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    timer::{SysTimerExt},
};
 
#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    timer::SysTimerExt,
};

#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    delay::SYSTDelayExt,
};

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
   timer::Timer,
   delay::{DelayExt, DelayFromCountDownTimer},
   pac::{TIM2, TIM5},
};


///////////////////// 

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, DelayNs,};
use rust_integration_testing_of_examples::setup::{CorePeripherals};

#[entry]
fn main() -> ! {
    hprintln!("AHT aht30-aht30-semi example");

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (i2c1, _i2c2, _led, mut delay, clocks) = setup::i2c1_i2c2_led_delay_clocks_from_dp(dp);

    let mut delay2 = cp.SYST.delay(&clocks); 

    hprintln!("delay.delay_ms(2000)");
    delay.delay_ms(2000);    

    hprintln!("delay2.delay_ms(2000)");
    delay2.delay_ms(2000);    

    hprintln!("Start the sensor...");

    //  asm::bkpt();   
    let mut aht = Aht30::new(AHT30_DEFAULT_ADDR, i2c1, delay);
    aht.calibrate().expect("sensor calibrate failed.");
    hprintln!("Sensor started.");
    let checksum = true;   // enable checking

    loop {        
        hprintln!("aht.measure()");
        let (humidity, temperature)  = aht.read(checksum).expect("read error").decode();
        hprintln!("{:.3}C  {}% RH", temperature, humidity);

        delay2.delay_ms(5000); 
    }
}
