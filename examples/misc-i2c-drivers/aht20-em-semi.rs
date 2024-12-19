//! Continuously read temperature from AHT20 and with semihosting hprintln.
//!
//! The "semi" examples simplify testing of the sensor crate alone, without display complications.

//! Using crate embedded-aht20

//! Status Dec 17, 2024
//!   Compiles with and without --release using stm32f1xx_hal,  stm32f4xx_hal, and stm32g4xx_hal
//! 
//!   Too large to load without  --release on bluepill 128K flash.
//!   Panics at Aht20::new() with  --release on bluepill 128K flash
//! 
//!   Fails running  (disappears Aht20::new()) with and without  --release
//!        using stm32f4xx_hal on blackpill  stm32f411
//! 
//!   Runs with  --release using stm32g4xx_hal on stm32g474xE.
//!   Fails running  (disappears at aht.measure()) on stm32g474xE without  --release

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_semihosting_05::hprintln;
use cortex_m::asm;

use embedded_aht20::{Aht20, DEFAULT_I2C_ADDRESS}; 

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
    hprintln!("AHT20-em example");

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (mut i2c1, _i2c2, _led, mut delay, clocks) = setup::i2c1_i2c2_led_delay_clocks_from_dp(dp);

    let mut delay2 = cp.SYST.delay(&clocks); 

    hprintln!("delay.delay_ms(2000)");
    delay.delay_ms(2000);    

    hprintln!("delay2.delay_ms(2000)");
    delay2.delay_ms(2000);    

    hprintln!("Start the sensor...");
    // Start the sensor.   address 0x38 cannot be changed

  asm::bkpt();  
    let mut aht  = Aht20::new(&mut i2c1, DEFAULT_I2C_ADDRESS, &mut delay).expect("sensor initialization failed.");
    //let mut aht  = Aht20::new(&mut i2c2, DEFAULT_I2C_ADDRESS, Delay {}).expect("sensor initialization failed.");
    hprintln!("Sensor started.");

    loop {        
        hprintln!("aht.measure()");
        let th = aht.measure().unwrap();   // Read humidity and temperature.
        hprintln!("{:.3}C  {}% RH", th.temperature.celcius(), th.relative_humidity);

        delay2.delay_ms(5000); 
    }
}
