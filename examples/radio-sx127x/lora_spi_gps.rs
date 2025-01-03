//! Serial interface read GPS on usart and transmit with LoRa using crate radio_sx127x (on SPI).
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
use embedded_io::{Read};

//use radio::Transmit;
use radio_sx127x::Transmit;

use radio_sx127x::{
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
};

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED};

//use rust_integration_testing_of_examples::lora::{CONFIG_PA, CONFIG_RADIO, CONFIG_LORA, CONFIG_CH, FREQUENCY, MODE};
use rust_integration_testing_of_examples::lora::{CONFIG_RADIO};


#[entry]
fn main() -> ! {
    let dp =Peripherals::take().unwrap();
    let (mut led, _tx, mut rx_gps, spi, spiext, delay) = setup::led_tx_rx_spi_spiext_delay_from_dp(dp); 
    led.off();
 
    let mut lora = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, &CONFIG_RADIO
       ).unwrap(); // should handle error

    // byte buffer   Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
    let mut buf2: heapless::Vec<u8, 80> = heapless::Vec::new();

    //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();  //0 of 80
    //hprintln!("buf2   at {} of {}",   buf2.len(),   buf2.capacity()).unwrap();  //0 of 80
    buffer.clear();
    buf2.clear();

    let mut received: [u8; 80] = [0; 80];

    //hprintln!("going into write/read loop ^C to exit ...").unwrap();

    //let e: u8 = 9; // replace char errors with "9"
    let mut good = false; // true while capturing a line

    //let mut size: usize;   // buffer size should not be needed
    //size = buffer.len();   //packet size
    //hprintln!("read buffer {} of {}", size, buffer.capacity()).unwrap();
    hprintln!("entering transmit loop").unwrap();

    loop {
       let _len = rx_gps.read(&mut received);   //stm32f4xx_hal fails here with 
                                                //method not found in `Rx<USART2>`
                                                //see https://github.com/stm32-rs/stm32f4xx-hal/issues/721

       //let byte = match block!(rx_gps.read()) {
       //     Ok(byt) => byt,
       //     Err(_error) => e,
       //};
       
       //  LOGIC OF THIS NEEDS FIXING
        let byte = 1;  //FAKE

        if byte == 36 {
            //  $ is 36. start of a line
            buffer.clear();
            good = true; //start capturing line
        };

        if good {
            if buffer.push(byte).is_err() || byte == 13 {
                //transmit if end of line. \r is 13, \n is 10

                //hprintln!("{:?}", &buffer).unwrap();

                // this transmits the whole GPS message string

                match lora.start_transmit(&buffer) {
                    Ok(b) => b, // b is ()
                    Err(_err) => {
                        hprintln!("Error returned from lora.start_transmit().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };

                // this transmits GPS N and E coordinates in hundredths of degrees

                if &buffer[0..6] == [36, 71, 80, 82, 77, 67] {
                    // if message id is $GPRMC

                    for v in buffer[19..31].iter() {
                        buf2.push(*v).unwrap();
                    } // [19..31] is north/south.
                    for v in b"   ".iter() {
                        buf2.push(*v).unwrap();
                    }
                    for v in buffer[32..45].iter() {
                        buf2.push(*v).unwrap();
                    } // [32..45] is east/west

                    //hprintln!("{:?}", &buf2).unwrap();
                    hprint!(".").unwrap(); // print "."  on transmit of $GPRMC message (but not others)

                    match lora.start_transmit(&buf2) {
                        Ok(b) => b, // b is ()
                        Err(_err) => {
                            hprintln!("Error returned from lora.start_transmit().").unwrap();
                            panic!("should reset in release mode.");
                        }
                    };
                };

                // Note hprintln! requires semihosting. If hprintln! (thus also match section below) are
                // removed then this example works on battery power with no computer attached.
                // (tested only on blackpill with stm32f411 )

                // The first transmission often return false and prints "TX not complete", but works after that.
                // If this continually returns "TX not complete" then the radio should probably be reset,
                //  but should avoid panic_reset after first transmission.

                match lora.check_transmit() {
                    Ok(b) => {
                        if !b {
                            hprintln!("TX not complete").unwrap();
                            // if multible times then panic!("should reset in release mode.");
                        }
                    }
                    Err(_err) => {
                        hprintln!("Error returned from lora.check_transmit().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };

                buffer.clear();
                buf2.clear();
                good = false;
                lora.delay_ms(5000);
            };
        };
    }
}
