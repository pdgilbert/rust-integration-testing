
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

pub fn pin_i2c1_i2c2_led_tx1_rx1_tx2_rx2_spi_spiext_delay_clocks_adc1_from_dp(dp: Peripherals) ->  
          (OpenDrainType, I2c1Type, I2c2Type, LedType, Tx1Type,  Rx1Type, Tx2Type,  Rx2Type,
           SpiType, SpiExt, Delay, Clocks, AdcSensor1Type) {    
   //(pin, i2c1, i2c2, led, tx1, rx1, tx2, rx2, spi, spiext, delay, clocks, adc1) =  
   all_from_dp(dp)
}

pub fn tx1_rx1_tx2_rx2_from_dp(dp: Peripherals) -> (Tx1Type,  Rx1Type, Tx2Type,  Rx2Type) {    
   let (_pin, _i2c1, _i2c2, _led, tx1, rx1, tx2, rx2,  _spi, _spiext, _delay, _clocks, _adc1) =  all_from_dp(dp);
   (tx1, rx1, tx2, rx2)
}

//    ///// filter no  tx2, rx2  below

pub fn pin_i2c1_led_tx1_rx1_delay_adc1_from_dp(dp: Peripherals) ->  
                (OpenDrainType, I2c1Type, LedType, TxType,  RxType, Delay, AdcSensor1Type) {    
   let (pin, i2c1, _i2c2, led, tx1, rx1, _tx2, _rx2, _spi, _spiext, delay, _clocks, adc1) =  all_from_dp(dp);
   (pin, i2c1, led, tx1, rx1, delay, adc1) 
}

pub fn pin_i2c1_led_delay_adc1_from_dp(dp: Peripherals) ->  
                (OpenDrainType, I2c1Type, LedType, Delay, AdcSensor1Type) {    
   let (pin, i2c1, _i2c2, led, _tx1, _rx1, _tx2, _rx2, _spi, _spiext, delay, _clocks, adc1) =  all_from_dp(dp);
   (pin, i2c1, led, delay, adc1) 
}


//    ///// filter   no  adc  or  tx2, rx2  below

pub fn no_adc_from_dp(dp: Peripherals) ->  
        (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType,  RxType,  SpiType, SpiExt, Delay, Clocks) {    
   let (pin, i2c1, i2c2, led, tx1, rx1, _tx2, _rx2, spi, spiext, delay, clocks, _adc1) =  all_from_dp(dp);
   (pin, i2c1, i2c2, led, tx1, rx1, spi, spiext, delay, clocks) 
}

pub fn i2c1_i2c2_led_spi_spiext_delay_from_dp(dp: Peripherals) ->  
                           (I2c1Type, I2c2Type, LedType, SpiType, SpiExt, Delay) {    
   let (_pin, i2c1, i2c2, led, _tx1, _rx1, spi, spiext, delay, _clocks) =  no_adc_from_dp(dp);
   (i2c1, i2c2, led, spi, spiext, delay) 
}

pub fn led_tx_rx_spi_spiext_delay_from_dp(dp: Peripherals) ->  
                           (LedType, TxType,  RxType,  SpiType, SpiExt, Delay) {    
   let (_pin, _i2c1, _i2c2, led, tx1, rx1, spi, spiext, delay, _clocks) =  no_adc_from_dp(dp);
   (led, tx1, rx1, spi, spiext, delay) 
}

pub fn led_spi_spiext_delay_from_dp(dp: Peripherals) ->  
                           (LedType, SpiType, SpiExt, Delay) {    
   let (_pin, _i2c1, _i2c2, led, _tx1, _rx1, spi, spiext, delay, _clocks) =  no_adc_from_dp(dp);
   (led, spi, spiext, delay) 
}

pub fn pin_i2c1_i2c2_led_tx_delay_clocks_from_dp(dp: Peripherals) ->  
                           (OpenDrainType, I2c1Type, I2c2Type, LedType, TxType, Delay, Clocks) {    
   let (pin, i2c1, i2c2, led, tx1, _rx1, _spi, _spiext, delay, clocks) =  no_adc_from_dp(dp);
   (pin, i2c1, i2c2, led, tx1, delay, clocks) 
}


pub fn i2c1_i2c2_led_delay_clocks_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay, Clocks) {    
   let (_pin, i2c1, i2c2, led, _tx1, _rx1, _spi, _spiext, delay, clocks) =  no_adc_from_dp(dp);
   (i2c1, i2c2, led, delay, clocks) 
}

pub fn i2c1_i2c2_delay_clocks_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, Delay, Clocks) {    
   let (_pin, i2c1, i2c2, _led, _tx1, _rx1, _spi, _spiext, delay, clocks) =  no_adc_from_dp(dp);
   (i2c1, i2c2, delay, clocks) 
}

pub fn i2c1_i2c2_led_delay_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType, Delay) {    
   let (_pin, i2c1, i2c2, led, _tx1, _rx1, _spi, _spiext, delay, _clocks) =  no_adc_from_dp(dp);
   (i2c1, i2c2, led, delay) 
}

pub fn i2c1_i2c2_led_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type, LedType) {    
   let (_pin, i2c1, i2c2, led, _tx1, _rx1, _spi, _spiext, _delay, _clocks) =  no_adc_from_dp(dp);
   (i2c1, i2c2, led) 
}

pub fn i2c1_i2c2_from_dp(dp: Peripherals) ->  (I2c1Type, I2c2Type) {    
   let (_pin, i2c1, i2c2, _led, _tx1, _rx1, _spi, _spiext, _delay, _clocks) =  no_adc_from_dp(dp);
   (i2c1, i2c2) 
}

pub fn pin_i2c_led_tx_delay_clocks_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay, Clocks) {    
   let (pin, i2c, _i2c2, led, tx1, _rx1, _spi, _spiext, delay, clocks) =  no_adc_from_dp(dp);
   (pin, i2c, led, tx1, delay, clocks) 
}

pub fn pin_i2c_led_tx_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, TxType, Delay) {    
   let (pin, i2c, _i2c2, led, tx1, _rx1, _spi, _spiext, delay, _clocks) =  no_adc_from_dp(dp);
   (pin, i2c, led, tx1, delay) 
}

pub fn i2c_led_tx_from_dp(dp: Peripherals) ->  (I2cType, LedType, TxType) {    
   let (_pin, i2c, _i2c2, led, tx, _rx1, _spi, _spiext, _delay, _clocks) =  no_adc_from_dp(dp);
   (i2c, led, tx) 
}


//    ///// filter   no  adc or spi  or  tx_rx  or  tx2, rx2   below

fn no_tx_rx_spi_from_dp(dp: Peripherals) ->  (OpenDrainType, I2c1Type, I2c2Type, LedType, Delay, Clocks) {    
   let (pin, i2c1, i2c2, led, _tx1, _rx1, _spi, _spiext, delay, clocks) = no_adc_from_dp(dp);
   (pin, i2c1, i2c2, led, delay, clocks)
}

pub fn pin_i2c_led_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, I2cType, LedType, Delay) {    
   let (pin, i2c, _i2c2, led, delay, _clocks) =  no_tx_rx_spi_from_dp(dp);
   (pin, i2c, led, delay) 
}

pub fn i2c_led_delay_from_dp(dp: Peripherals) ->  (I2cType, LedType, Delay) {    
   let (_pin, i2c, _i2c2, led, delay, _clocks) =  no_tx_rx_spi_from_dp(dp);
   (i2c, led, delay) 
}

pub fn i2c_delay_from_dp(dp: Peripherals) ->  (I2cType, Delay) {    
   let (_pin, i2c, _i2c2, _led, delay, _clocks) =  no_tx_rx_spi_from_dp(dp);
   (i2c, delay) 
}

pub fn i2c_led_from_dp(dp: Peripherals) ->  (I2cType, LedType) {    
   let (_pin, i2c, _i2c2, led, _delay, _clocks) =  no_tx_rx_spi_from_dp(dp);
   (i2c, led) 
}

pub fn pin_delay_from_dp(dp: Peripherals) ->  (OpenDrainType, Delay) {    
   let (pin, _i2c, _i2c2, _led, delay, _clocks) =  no_tx_rx_spi_from_dp(dp);
   (pin, delay) 
}

pub fn led_from_dp(dp: Peripherals) ->  LedType {    
   let (_pin, _i2c, _i2c2, led, _delay, _clocks) = no_tx_rx_spi_from_dp(dp);
   led
}

