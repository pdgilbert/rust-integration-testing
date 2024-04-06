
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//   //////////////////////////////////////////////////////////////////////

//pub use  crate::setup_all_ALL::*;

#[cfg(feature = "stm32f4xx")]
pub use  crate::setup_all_stm32f4xx::*;

#[cfg(feature = "stm32g4xx")]
pub use  crate::setup_all_stm32g4xx::*;

#[cfg(feature = "stm32h7xx")]
pub use  crate::setup_all_stm32h7xx::*;


//   //////////////////////////////////////////////////////////////////////


pub fn pin_i2c_led_tx_delay_clocks_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {    
   let (pin, i2c, led, tx, delay, clocks) =  all_from_dp(dp);
   (pin, i2c, led, tx, delay, clocks) 
}

pub fn pin_i2c_led_tx_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay) {    
   let (pin, i2c, led, tx, delay, _clocks) =  all_from_dp(dp);
   (pin, i2c, led, tx, delay) 
}

pub fn pin_i2c_led_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, Delay) {    
   let (pin, i2c, led, _tx, delay, _clocks) =  all_from_dp(dp);
   (pin, i2c, led, delay) 
}

pub fn i2c_led_delay_from_dp(dp: Peripherals) ->  (I2cType, LedType, Delay) {    
   let (_pin, i2c, led, _tx, delay, _clocks) =  all_from_dp(dp);
   (i2c, led, delay) 
}

pub fn i2c_led_tx_from_dp(dp: Peripherals) ->  (I2cType, LedType, TxType) {    
   let (_pin, i2c, led, tx, _delay, _clocks) = all_from_dp(dp);
   (i2c, led, tx)
}

pub fn pin_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, Delay) {    
   let (pin, _i2c, _led, _tx, delay, _clocks) =  all_from_dp(dp);
   (pin, delay) 
}

