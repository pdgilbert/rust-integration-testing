
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::dht_i2c_led_delay::{setup_dp, I2cType, LED, LedType, DelayType, DelayMs, Peripherals};

pub fn setup() ->  (I2cType, LedType, DelayType) {    
   let dp = Peripherals::take().unwrap();
   let (_dht, i2c, led,delay) = setup_dp(dp);

   (i2c, led, delay)
}
