//! Continuously read temperature from AHT20 and with semihosting hprintln.
//!
//! The "semi" examples simplify testing of the sensor crate alone, without display complications.

//! Using crate embedded-aht20

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_semihosting_05::hprintln;
use cortex_m::asm;

use aht20_driver::{AHT20, SENSOR_ADDRESS}; 

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

/////////////////////   hals

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
    hprintln!("AHT20-em example");

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (i2c1, _i2c2, _led, mut delay, clocks) = setup::i2c1_i2c2_led_delay_clocks_from_dp(dp);

    let mut delay2 = cp.SYST.delay(&clocks); 

    hprintln!("delay.delay_ms(2000)");
    delay.delay_ms(2000);    

    hprintln!("delay2.delay_ms(2000)");
    delay2.delay_ms(2000);    

    hprintln!("Start the sensor...");
    // Start the sensor.   address 0x38 cannot be changed

 // asm::bkpt();  
    let mut aht20_uninit = AHT20::new(i2c1, SENSOR_ADDRESS);
    let mut aht = aht20_uninit.init(&mut delay).unwrap();

    hprintln!("Sensor started.");

    loop {
        hprintln!("loop i");      
        
        hprintln!("aht.measure()");
        let th = aht.measure(&mut delay).unwrap();  

        hprintln!("{:.2}C  {:.2}% RH", th.temperature, th.humidity);

        delay2.delay_ms(5000); 
    }
}
