//! TO DO
//!        - add lora transmition
//!        - share bus for aht
//!        - read and display temp, humidity from aht and write to ens160 for improved calculation.
//!        - use rtic
//! 
//! Continuously measure the eCO2 and eVOC in the air and print it to an SSD1306 OLED display.
//! Ssd1306 is on i2c1. Ens160 and AHT21 are on i2c2.
//! An ens160+AHT2x board has aht2x, typically aht21 which can be read and used to improve
//! the calculations. 
//! 
//! Regarding sensor power supply issue see
//!  https://community.home-assistant.io/t/issue-with-ens160-aht21-ens160-readings-unavailable/697522/2
//!
//! Compare ens160-co2-voc-iaq-display, ccs811-gas-voc-display, lora_spi_gps,  and  aht20-display
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]

//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::{hprintln};

use cortex_m_rt::entry;

//use shared_bus::cortex_m::prelude::_embedded_hal_blocking_i2c_Read;
//use shared_bus::cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
//use shared_bus::cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;

//use core::cell::RefCell;
//use cortex_m::interrupt::Mutex;
//use shared_bus::{I2cProxy};

/////////////////////   ens
use ens160::{Ens160, AirQualityIndex, ECo2};

///////////////////// NEED aht21 NOT aht20
//use aht20_async::{Aht20};   consider async, but need one that is working
//use aht20::{Aht20};


/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space
//with font 6x10, VPIX=12 = 10 high + 2 space ; 8X13  VPIX=14 = 13 + 1

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

/////////////////////   hals

use embedded_hal::{
   //i2c::I2c as I2cTrait,
   delay::DelayNs,
};

use heapless;


/////////////////////   lora

use radio::Transmit;  // trait needs to be in scope to find  methods start_transmit and check_transmit.

use radio_sx127x::{
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
};

//use rust_integration_testing_of_examples::lora::{CONFIG_PA, CONFIG_RADIO, CONFIG_LORA, CONFIG_CH, FREQUENCY, MODE};
use rust_integration_testing_of_examples::lora::{CONFIG_RADIO};

/////////////////////  

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED};

//////////////////////////////////////////////////////////////////////////////////// 

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("example");
    //hprintln!("ens160-co2-voc-iaq-display example").unwrap();

    let dp =Peripherals::take().unwrap();
    let (i2c1, i2c2, mut led, spi, spiext, delay) = setup::i2c1_i2c2_led_spi_spiext_delay_from_dp(dp); 
    led.off();

    /////////////////////   lora
    let mut lora = Sx127x::spi(spi, spiext.cs,  spiext.busy, spiext.ready, spiext.reset, delay, &CONFIG_RADIO
       ).unwrap(); // should handle error   //delay is available in lora

    // byte buffer   Nov 2020 limit data.len() < 255 in radio_sx127x  .start_transmit
    let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
    let mut buf2: heapless::Vec<u8, 80> = heapless::Vec::new();

    //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();  //0 of 80
    //hprintln!("buf2   at {} of {}",   buf2.len(),   buf2.capacity()).unwrap();  //0 of 80
    buffer.clear();
    buf2.clear();

    //    let mut delay_syst = cp.SYST.delay(&clocks); 

    /////////////////////   ssd
    //hprintln!("ssd start").unwrap();

    let interface = I2CDisplayInterface::new(i2c1);
    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();
    //display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

    display.clear_buffer();
    Text::with_baseline(
        "Display initialized ...",
        //Point::new(0, i as i32 * VPIX),
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .unwrap();

    //let manager = shared_bus::BusManagerSimple::new(i2c2);
    //let manager: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();


    /////////////////////   ens

    //hprintln!("ens start").unwrap();
    //let mut ens = Ens160::new(manager.acquire_i2c(), 0x53);  //0x52 Ens160,  0x53 Ens160+Aht21
    let mut ens = Ens160::new(i2c2, 0x53);  //0x52 Ens160,  0x53 Ens160+Aht21
 
    let z = ens.reset();

    match z {
            Ok(v)   =>  {hprintln!("v:{:?}, ",v).unwrap()},
            Err(e)  =>  {hprintln!(" ens.reset() Error {:?}. ", e).unwrap(); 
                         //panic!("Error reading"),
                         //(25, 40)  //supply default values
                        },
    };

    lora.delay_ms(250);
    let z = ens.operational();

    match z {
            Ok(v)   =>  {hprintln!("v:{:?}, ",v).unwrap()},
            Err(e)  =>  {hprintln!(" ens.operational() Error {:?}. ", e).unwrap(); 
                         //panic!("Error reading"),
                         //(25, 40)  //supply default values
                        },
    };

    /////////////////////   aht
    //let mut aht = Aht20::new(manager.acquire_i2c(), AltDelay{}); //NEEDS delay, shared bus, aht21 not 20

    ///////////////////// initialize loop variables

    lora.delay_ms(1000);

    let mut lines: [heapless::String<32>; 4] = [heapless::String::new(), heapless::String::new(),
                                                heapless::String::new(), heapless::String::new(), ];


    // if these are initialized they can be reset in the loop and values used after t he loop
    let mut tvoc: u16;
    let mut eco2: ECo2;
    let mut aqi1: AirQualityIndex ;
    let mut aqi2: AirQualityIndex ;
 //   let mut temp: i16 = -32767;    // initialize with imposible values
 //   let mut humd: u16 =  20000;    // initialize with imposible values


    /////////////////////    measure and display in loop

    //hprintln!("loop start").unwrap();
    loop {
        // Blink LED to check that everything is actually running.
        // If the LED is off, something went wrong.
        // led.blink(100_u16, &mut delay);  delay consumed by lora above
        led.on();
        lora.delay_ms(200);
        led.off();

//        // Read humidity and temperature.
//        let (h, t) = aht.read().unwrap();
//        //let (h, t) = aht.end_read().unwrap();


        /////////////////////    measure from ens160

        if let Ok(status) = ens.status() {
            if status.data_is_ready() {
                tvoc = ens.tvoc().unwrap();
                eco2 = ens.eco2().unwrap();
                aqi1 = AirQualityIndex::try_from(eco2).unwrap();  // from eco2
                aqi2 = ens.air_quality_index().unwrap();  // directly
         //       (temp, humd) = ens.temp_and_hum().unwrap();
                //hprintln!("tvoc:{:?}, ", tvoc).unwrap();
                //hprintln!("eco2:{:?}, ", eco2).unwrap();
                //hprintln!("aqi1:{:?}, ", aqi1).unwrap();
                //hprintln!("aqi2:{:?}, ", aqi2).unwrap();
                //hprintln!("temp:{:?}, ", temp).unwrap();
                //hprintln!("humd:{:?}, ", humd).unwrap();


        /////////////////////   send over lora   THIS IS NOT COMPLETE
               buffer.clear();
               write!(buffer, "TVOC {}ppb {:?}ppm", tvoc, eco2).unwrap();
               
               match lora.start_transmit(&buffer) {
                    Ok(b) => b, // b is ()
                    Err(_err) => {
                        hprintln!("Error returned from lora.start_transmit().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };

                match lora.check_transmit() {
                    Ok(b) => {
                        if !b {
                            hprintln!("transmit not complete").unwrap();
                            // if multible times then panic!("should reset in release mode.");
                        }
                    }
                    Err(_err) => {
                        hprintln!("Error returned from lora.check_transmit().").unwrap();
                        panic!("should reset in release mode.");
                    }
                };
                lora.delay_ms(5000);


        /////////////////////   display

    // temp_and_hum(&mut self) -> Result<(i16, u16), E> 
    //The units are scaled by 100. For example, a temperature value of 2550 represent2 25.50
    // and a humidity value of 5025 represents 50.25% RH.

                for line in lines.iter_mut() {line.clear();}
                
                // The TVOC level is expressed in parts per billion (ppb) in the range 0-65000.
                // The eCO2 level is expressed in parts per million (ppm) in the range 400-65000.
                write!(lines[0], "TVOC {}ppb {:?}ppm", tvoc, eco2).unwrap();
                write!(lines[1], "AQI 1: {:?}  ", aqi1).unwrap();
                write!(lines[2], "AQI 2: {:?}  ", aqi2).unwrap();
                //write!(lines[2], "{:?} deg  {:?} %RH", temp, humd).unwrap();  // /100

                display.clear_buffer();
                for i in 0..lines.len() {
                    //with font 6x10, VPIX= 12 = 10 high + 2 space
                    Text::with_baseline(
                           &lines[i], 
                           Point::new(0, i as i32 * VPIX), 
                           text_style, 
                           Baseline::Top,
                    ).draw(&mut display).unwrap();
                }
                display.flush().unwrap();
            }
        }
    
        lora.delay_ms(5000);
        //hprintln!("loop end").unwrap();
    }
    //let i2c = ens.release(); // destruct driver to re-use bus
}
