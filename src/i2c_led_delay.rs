// CONSIDER MERGING INTO setups  but used in many examples.
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::dp::{Peripherals};
pub use crate::dht_i2c_led_usart_delay::{setup_dht_i2c_led_usart_delay_using_dp, I2cType, LED, LedType, DelayUs};
pub use crate::delay::{Delay1Type as DelayType};

pub fn setup() ->  (I2cType, LedType, DelayType) {    
   let dp = Peripherals::take().unwrap();
   let (_dht, i2c, led, _usart, delay) = setup_dht_i2c_led_usart_delay_using_dp(dp);

   (i2c, led, delay)
}
