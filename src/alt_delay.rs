#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use embedded_hal::delay::DelayNs;

// A delay is used in some sensor initializations and read operationes (eg dht). 
// Systick is used by monotonic (for spawn), so delay needs to use a timer other than Systick
// asm::delay is not an accurate timer but gives a delay at least number of indicated clock cycles.

use cortex_m::asm::delay; // argment in clock cycles so (5 * CLOCK) cycles gives aprox 5 second delay

//should be set for board not for HAL

#[cfg(feature = "stm32f0xx")]
pub const ALTCLOCK: u32 = 8_000_000;   // CHECK

#[cfg(feature = "stm32f1xx")]
pub const ALTCLOCK: u32 = 8_000_000;   // really 8_000_000;  but not providing enough delay for DHT-11
// dht seems to be sensitive to both too fast and too slow.  
// 5_000_000 and 7_000_000 work (usually/often) for dht_rtic
// 7_000_000 and 8_000_000 work (sometimes/rarely) for ccs811-co2-voc

#[cfg(feature = "stm32f3xx")]
pub const  ALTCLOCK: u32 = 8_000_000;

#[cfg(feature = "stm32f4xx")]
pub const  ALTCLOCK: u32 = 16_000_000;

#[cfg(feature = "stm32f7xx")]
pub const  ALTCLOCK: u32 = 8_000_000;

#[cfg(feature = "stm32g0xx")]
pub const  ALTCLOCK: u32 = 16_000_000;

#[cfg(feature = "stm32g4xx")]
pub const  ALTCLOCK: u32 = 16_000_000;

#[cfg(feature = "stm32h7xx")]
pub const  ALTCLOCK: u32 = 8_000_000;

#[cfg(feature = "stm32l0xx")]
pub const  ALTCLOCK: u32 = 8_000_000;

#[cfg(feature = "stm32l1xx")]
pub const  ALTCLOCK: u32 = 8_000_000;

#[cfg(feature = "stm32l4xx")]
pub const  ALTCLOCK: u32 = 8_000_000;

pub struct AltDelay {}

impl DelayNs for AltDelay {
    fn delay_ns(&mut self, t:u32) {
        delay((t as u32) * (ALTCLOCK / 1_000_000_000)); 
    }
    fn delay_us(&mut self, t:u32) {
        delay((t as u32) * (ALTCLOCK / 1_000_000)); 
    }
    fn delay_ms(&mut self, ms: u32) {
        delay((ms as u32) * (ALTCLOCK / 1000)); 
    }
}
