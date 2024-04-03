//! Receive message with LoRa using crate radio_sx127x (on SPI).
//! See example lora_spi_send for more details.

#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_reset;

use cortex_m_rt::entry;
use cortex_m_semihosting::*;

use embedded_hal::delay::DelayNs;

use radio::Receive;
use radio_sx127x::prelude::PacketInfo;

use rust_integration_testing_of_examples::lora;
use rust_integration_testing_of_examples::led;
use rust_integration_testing_of_examples::led::LED;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::{
   pac::{Peripherals},
};

fn to_str(x: &[u8]) -> &str {
    match core::str::from_utf8(x) {
        Ok(str) => &str,
        Err(_error) => "problem converting u8 to str ",
    }
}

#[entry]
fn main() -> ! {
    let mut lora = lora::setup_lora_from_dp(Peripherals::take().unwrap()); //delay is available in lora
    let mut led  = led::setup_led_from_dp(Peripherals::take().unwrap()); 
    led.off();

    lora.start_receive().unwrap(); // should handle error

    let mut buff = [0u8; 1024];
    let mut n: (usize, PacketInfo);
    //let mut info = PacketInfo::default();

    loop {
        let poll = lora.check_receive(false);
        // false (the restart option) specifies whether transient timeout or CRC errors should be
        // internally handled (returning Ok(false) or passed back to the caller as errors.

        match poll {
            Ok(v) if v => {
                n = lora.get_received(&mut buff).unwrap();
                //hprintln!("RX complete ({:?}, length: {})", info, n).unwrap();
                //hprintln!("{:?}", &buff[..n.0]).unwrap();
                // for some reason the next prints twice?
                hprintln!("{}", to_str(&buff[..n.0])).unwrap()
            }

            Ok(_v) => (), // hprint!(".").unwrap(),   // print "." if nothing received

            Err(err) => hprintln!("poll error {:?} ", err).unwrap(),
        };

        lora.delay_ms(100);
    }
}
