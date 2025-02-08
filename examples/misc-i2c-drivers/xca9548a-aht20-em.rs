//! Continuously read temperature from multiple sensors and display on SSD1306 OLED.
//! The display is on i2c2 and the senssors are multiplexed on i2c1 using  xca9548a.
//!
//! With aht20_bl each sensor consumes a delay so these delays are generated with AltDelay.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

// Need to run with debug console if hprintln is uncommented, otherwise stalls waiting to print.
use cortex_m_semihosting_05::hprintln;

use cortex_m_rt::entry;
use core::fmt::Write;

use embedded_aht20::{Aht20, DEFAULT_I2C_ADDRESS}; 

use xca9548a::{SlaveAddr as XcaSlaveAddr, Xca9548a, I2cSlave}; 

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

use embedded_graphics::{
    //mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    //mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    mono_font::{iso_8859_1::FONT_6X10 as FONT, MonoTextStyleBuilder},      // need iso for degree symbol
    //mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder},
    //mono_font::{iso_8859_1::FONT_9X15 as FONT, MonoTextStyleBuilder}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

type DisplaySizeType = ssd1306::prelude::DisplaySize128x64;

const ROTATION: DisplayRotation = DisplayRotation::Rotate0;   // 0, 90, 180, 270
const DISPLAYSIZE: DisplaySizeType = ssd1306::prelude::DisplaySize128x64;
const PPC: usize = 12;  // verticle pixels per character plus space for FONT_6X10 
const DISPLAY_LINES: usize = 12;     // in characters for 128x64   Rotate90
const DISPLAY_COLUMNS: usize = 12;  // in characters   Rotate90
const R_VAL: heapless::String<DISPLAY_COLUMNS> = heapless::String::new();

type  ScreenType = [heapless::String<DISPLAY_COLUMNS>; DISPLAY_LINES];

///////////////////////////////////////////////////////////////

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, I2c1Type, DelayNs}; 
use rust_integration_testing_of_examples::alt_delay::AltDelay; //cortex_m::asm::delay with traits


#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
   delay::DelayExt,   
   prelude::*,
   i2c::{Error},
}; 



///////////////////////////////////////////////////////////////

fn show_message<S>(
    text: &str,   
    //text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: ssd1306::size::DisplaySize,  //trait
{
   
   // workaround. build here because text_style cannot be shared
   let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

   disp.clear_buffer();
   Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
           .draw(&mut *disp)
           .unwrap();

   disp.flush().unwrap();
   ()
}

fn show_screen<S>(
    screen: &ScreenType,   
    //text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: ssd1306::size::DisplaySize,  //trait
{
   //hprintln!("in show_screen").unwrap();
   
   // workaround. build here because text_style cannot be shared
   let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

   disp.clear_buffer();
   for  i in 0..DISPLAY_LINES {  // 0..2 is [0, 1] ;  0..=2 is [0, 1, 2]
     // hprintln!("display line {}", i).unwrap();
      if 0 != screen[i].len() {                         // 12 point per char verticle
         Text::with_baseline( &screen[i], Point::new(0, (i*PPC).try_into().unwrap()), text_style, Baseline::Top)
              .draw(&mut *disp)
              .unwrap();
      };
   };

   disp.flush().unwrap();
   ()
}


#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay1) = setup::i2c1_i2c2_led_delay_from_dp(dp);

    led.off();

    led.blink(2000_u16, &mut delay1); // Blink LED to indicate setup finished.

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(i2c2);

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, ROTATION)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

    Text::with_baseline(   "xca5948a \nxca9548a-aht20-em", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();

    led.blink(500_u16, &mut delay1); // Blink LED to indicate Ssd1306 initialized.

    let mut screen: ScreenType = [R_VAL; DISPLAY_LINES];

    
    /////////////////////  xca   multiple devices on i2c bus

//    let slave_address = 0b010_0000; // example slave address
//    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent

    let switch1 = Xca9548a::new(i2c1, XcaSlaveAddr::default());

//    // Enable channel 0
//    switch1.select_channels(0b0000_0001).unwrap();
//
//    // write to device connected to channel 0 using the I2C switch
//    if switch1.write(slave_address, &write_data).is_err() {
//        //hprintln!("Error write channel 0!").unwrap();
//    }
//
//    // read from device connected to channel 0 using the I2C switch
//    let mut read_data = [0; 2];
//    if switch1.read(slave_address, &mut read_data).is_err() {
//        //hprintln!("Error read channel 0!").unwrap();
//    }
//
//    // write_read from device connected to channel 0 using the I2C switch
//    if switch1
//        .write_read(slave_address, &write_data, &mut read_data)
//        .is_err()
//    {
//        //hprintln!("Error write_read!").unwrap();
//    }

    show_message(&"Sens xca", &mut display);

    /////////////////////  AHT20s on xca    // Start the sensors.

    type SensType<'a> =Aht20<I2cSlave<'a,  Xca9548a<I2c1Type>, I2c1Type>, AltDelay>;

    const SENSER: Option::<SensType> = None;      //const gives this `static lifetime
    let mut sensors: [Option<SensType>; 8] = [SENSER; 8];

    // Split the device and pass the virtual I2C devices to sensor driver
    let switch1parts = switch1.split();

    let parts  = [switch1parts.i2c0, switch1parts.i2c1, switch1parts.i2c2, switch1parts.i2c3,
                  switch1parts.i2c4, switch1parts.i2c5, switch1parts.i2c6, switch1parts.i2c7];

hprintln!("prt in parts");
    let mut i = 0;  // not very elegant
    for  prt in parts {
       hprintln!("screen[0].clear()");
       screen[0].clear();
       hprintln!("prt {}", i);
       let z = Aht20::new(prt, DEFAULT_I2C_ADDRESS, AltDelay{});
       hprintln!("match z");
       match z {
           Ok(v)    => {sensors[i] = Some(v);
                        write!(screen[0], "J{} in use", i).unwrap();
                       },
           Err(_e)  => {write!(screen[0], "J{} unused", i).unwrap();},
       }
       show_screen(&screen, &mut display);
       delay1.delay_ms(500);
       
       i += 1;
    };

    screen[0].clear();
    write!(screen[0], "    °C %RH").unwrap();

hprintln!("loop");
    loop {   // Read humidity and temperature.
      let mut ln = 1;  // screen line to write. Should make this roll if number of sensors exceed DISPLAY_LINES

      for  i in 0..sensors.len() {
          match   &mut sensors[i] {
               None       => {},  //skip
   
               Some(sens) => {screen[ln].clear();
                              match sens.measure() {
                                   Ok(m)      => {//hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
                                                  write!(screen[ln], "J{} {:.2} {:.2}",
                                                          i, m.temperature.celsius(), m.relative_humidity).unwrap();
                                                 },
                                   Err(e)     => {//sens.reset().unwrap(); MAY NEED RESET WHEN THERE ARE ERRORS
                                                  //hprintln!("read error {:?}", e).unwrap();
                                                  write!(screen[ln], "J{} read error. Reset{:?}", 0, e).unwrap();
                                                 }
                                   };
                              show_screen(&screen, &mut display);
                              ln += 1;
                              ln = ln % DISPLAY_LINES;
                              delay1.delay_ms(500);
                              },
           };          
       };
       delay1.delay_ms(5000);
    }
}
 
