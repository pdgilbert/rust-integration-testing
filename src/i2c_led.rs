
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::dp::{Peripherals};
pub use crate::onewire_i2c_led;
pub use crate::onewire_i2c_led::{I2cType, Clocks};
pub use crate::led::{setup_led, LED, LedType};

pub use crate::delay::DelayNs;
pub use crate::delay::{Delay2Type as Delay};

pub fn setup(dp: Peripherals) ->  (I2cType, LedType, Clocks) {    
   // This is a shortcut. 
   // Really should be done as in onewire_i2c_led::setup(dp) but omitting onewire
   let (_pin, i2c, led, _delay, clocks) = onewire_i2c_led::setup_from_dp(dp);

   (i2c, led, clocks)
}
//  Usage  (see example misc/temperature_display.rs - when ssd1306 is changed for eh-1
//    let cp = CorePeripherals::take().unwrap();
//    let dp = Peripherals::take().unwrap();
//    let (i2c, mut led, clocks) = i2c_led::setup(dp);
//
//    let mut delay = Delay::new(cp.SYST, clocks); 

