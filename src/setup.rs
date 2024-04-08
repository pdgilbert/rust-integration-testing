
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//   //////////////////////////////////////////////////////////////////////

//pub use  crate::setup_all_ALL::*;

#[cfg(feature = "stm32f1xx")]
pub use  crate::setup_all_stm32f1xx::*;

#[cfg(feature = "stm32f3xx")]
pub use  crate::setup_all_stm32f3xx::*;

#[cfg(feature = "stm32f4xx")]
pub use  crate::setup_all_stm32f4xx::*;

#[cfg(feature = "stm32f7xx")]
pub use  crate::setup_all_stm32f7xx::*;

#[cfg(feature = "stm32g0xx")]
pub use  crate::setup_all_stm32g0xx::*;

#[cfg(feature = "stm32g4xx")]
pub use  crate::setup_all_stm32g4xx::*;

#[cfg(feature = "stm32h7xx")]
pub use  crate::setup_all_stm32h7xx::*;

#[cfg(feature = "stm32l0xx")]
pub use  crate::setup_all_stm32l0xx::*;

#[cfg(feature = "stm32l1xx")]
pub use  crate::setup_all_stm32l1xx::*;

#[cfg(feature = "stm32l4xx")]
pub use  crate::setup_all_stm32l4xx::*;


//   //////////////////////////////////////////////////////////////////////

// This enforces a common pin usage in all examples, which simplifies rewiring for hardware tests

pub fn pin_i2c1_i2c2_led_tx_spi_spiext_delay_clocks_from_dp(dp: Peripherals) ->  
                           (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType,  RxType,  SpiType, SpiExt, Delay, Clocks) {    
   let (pin, i2c1, i2c2, led, tx, rx, spi, spiext, delay, clocks) =  all_from_dp(dp);
   (pin, i2c1, i2c2, led, tx, rx, spi, spiext, delay, clocks) 
}

pub fn led_spi_spiext_delay_from_dp(dp: Peripherals) ->  
                           (LedType, SpiType, SpiExt, Delay) {    
   let (_pin, _i2c1, _i2c2, led, _tx, _rx, spi, spiext, delay, _clocks) =  all_from_dp(dp);
   (led, spi, spiext, delay) 
}

pub fn pin_i2c1_i2c2_led_tx_delay_clocks_from_dp(dp: Peripherals) ->  
                           (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {    
   let (pin, i2c1, i2c2, led, tx, _rx, _spi, _spiext, delay, clocks) =  all_from_dp(dp);
   (pin, i2c1, i2c2, led, tx, delay, clocks) 
}


pub fn i2c1_i2c2_led_delay_clocks_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {    
   let (_pin, i2c1, i2c2, led, _tx, _rx, _spi, _spiext, delay, clocks) =  all_from_dp(dp);
   (i2c1, i2c2, led, delay, clocks) 
}

pub fn i2c1_i2c2_delay_clocks_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, Delay, Clocks) {    
   let (_pin, i2c1, i2c2, _led, _tx, _rx, _spi, _spiext, delay, clocks) =  all_from_dp(dp);
   (i2c1, i2c2, delay, clocks) 
}

pub fn i2c1_i2c2_led_delay_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay) {    
   let (_pin, i2c1, i2c2, led, _tx, _rx, _spi, _spiext, delay, _clocks) =  all_from_dp(dp);
   (i2c1, i2c2, led, delay) 
}

pub fn i2c1_i2c2_led_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType) {    
   let (_pin, i2c1, i2c2, led, _tx, _rx, _spi, _spiext, _delay, _clocks) =  all_from_dp(dp);
   (i2c1, i2c2, led) 
}

pub fn i2c1_i2c2_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type) {    
   let (_pin, i2c1, i2c2, _led, _tx, _rx, _spi, _spiext, _delay, _clocks) =  all_from_dp(dp);
   (i2c1, i2c2) 
}

pub fn pin_i2c_led_tx_delay_clocks_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {    
   let (pin, i2c, _i2c2, led, tx, _rx, _spi, _spiext, delay, clocks) =  all_from_dp(dp);
   (pin, i2c, led, tx, delay, clocks) 
}

pub fn pin_i2c_led_tx_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay) {    
   let (pin, i2c, _i2c2, led, tx, _rx, _spi, _spiext, delay, _clocks) =  all_from_dp(dp);
   (pin, i2c, led, tx, delay) 
}

pub fn pin_i2c_led_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, Delay) {    
   let (pin, i2c, _i2c2, led, _tx, _rx, _spi, _spiext, delay, _clocks) =  all_from_dp(dp);
   (pin, i2c, led, delay) 
}

pub fn i2c_led_delay_from_dp(dp: Peripherals) ->  (I2cType, LedType, Delay) {    
   let (_pin, i2c, _i2c2, led, _tx, _rx, _spi, _spiext, delay, _clocks) =  all_from_dp(dp);
   (i2c, led, delay) 
}

pub fn i2c_led_tx_from_dp(dp: Peripherals) ->  (I2cType, LedType, TxType) {    
   let (_pin, i2c, _i2c2, led, tx, _rx, _spi, _spiext, _delay, _clocks) = all_from_dp(dp);
   (i2c, led, tx)
}

pub fn i2c_delay_from_dp(dp: Peripherals) ->  (I2cType, Delay) {    
   let (_pin, i2c, _i2c2, _led, _tx, _rx, _spi, _spiext, delay, _clocks) =  all_from_dp(dp);
   (i2c, delay) 
}

pub fn i2c_led_from_dp(dp: Peripherals) ->  (I2cType, LedType) {    
   let (_pin, i2c, _i2c2, led, _tx, _rx, _spi, _spiext, _delay, _clocks) =  all_from_dp(dp);
   (i2c, led) 
}

pub fn pin_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, Delay) {    
   let (pin, _i2c, _i2c2, _led, _tx, _rx, _spi, _spiext, delay, _clocks) =  all_from_dp(dp);
   (pin, delay) 
}

pub fn led_from_dp(dp: Peripherals) ->  LedType {    
   let (_pin, _i2c, _i2c2, led, _tx, _rx, _spi, _spiext, _delay, _clocks) = all_from_dp(dp);
   led
}

