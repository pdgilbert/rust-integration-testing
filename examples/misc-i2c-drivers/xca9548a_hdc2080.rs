//! Continuously read temperature from multiple hdc2080s and display on SSD1306 OLED.
//! The display is on i2c2 and the sensors are multiplexed on i2c1 using  xca9548a.
//!
//! 
//!  Beware that switch1parts.i2c2 is the second multiplexed device on the i2c.
//!  
//!  With HAL stm32f4xx
//!               i2c1 (scl, sda)= (pb8,  pb9)
//!               i2c2 (scl, sda)= (pb10, pb3)
//!


#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use nb::block;

use hdc20xx::{Hdc20xx, SlaveAddr as HdcSlaveAddr, mode::OneShot};

use core::fmt::Write;

//use embedded_io::{Read, Write, WriteRead};

use xca9548a::{Error::I2C, Error as XcaError, SlaveAddr as XcaSlaveAddr, Xca9548a, I2cSlave}; 

use cortex_m_rt::entry;


// Need to run with debug console if printing is uncommented. 
// Running standalone stalls waiting to print.
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

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

//  note that larger font size increases memory and may require building with --release
//  
//  for 128x32 display
//  &FONT_9X15 128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.1 characters high
//  &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
//  &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8 =   4  characters high
//  &FONT_4X6  128 pixels/ 4 per font =  32  characters wide.  32/6 =  5.3 characters high

const ROTATION: DisplayRotation = DisplayRotation::Rotate90;   // 0, 90, 180, 270

//type DisplaySizeType = ssd1306::prelude::DisplaySize128x32;
//const DISPLAYSIZE: DisplaySizeType = ssd1306::prelude::DisplaySize128x32;

type DisplaySizeType = ssd1306::prelude::DisplaySize128x64;
const DISPLAYSIZE: DisplaySizeType = ssd1306::prelude::DisplaySize128x64;

const PPC: usize = 12;  // verticle pixels per character plus space for FONT_6X10 

//const DISPLAY_LINES: usize = 3;     // in characters for 128x32 Rotate0
//const DISPLAY_LINES: usize = 6;     // in characters for 128x64   Rotate0
const DISPLAY_LINES: usize = 12;     // in characters for 128x64   Rotate90

//const DISPLAY_COLUMNS: usize = 32;  // in characters   Rotate0
const DISPLAY_COLUMNS: usize = 12;  // in characters   Rotate90
const R_VAL: heapless::String<DISPLAY_COLUMNS> = heapless::String::new();

type  ScreenType = [heapless::String<DISPLAY_COLUMNS>; DISPLAY_LINES];


///////////////////////////////////////////////////////////////

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, I2c1Type, I2c2Type, DelayNs, I2c, I2C1};
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


    //////////////////////////////////////////////////////////////////////////////////
    //  delay2  from SYST.delay and clocks not used here but is done something like this
    //#[cfg(feature = "stm32f4xx")]
    //use stm32f4xx_hal::{
    //   timer::SysTimerExt,  // trait for cp.SYST.delay
    //   timer::SysDelay,
    //   timer::Delay, 
    //};
    //type Delay = Delay<TIM5, 1000000_u32>;
    //let delay2 = cp.SYST.delay(&clocks);
    //let () = delay1; opaque type impl DelayNs  does not work for AHT10, in AHT10::new()
    //let () = delay2;  type `Delay`
    // may need
    //use embedded_hal_02::blocking::delay::DelayMs;
    //////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////
    //  AltDelay{} is used further below to generate a DelayMs for each AHT10.
    // It can also be used like this
    //let mut delayx  = AltDelay{};
    // //If there is no need to disambuguate it can be used like this
    // //   delayx.delay_ms(1000);
    // //But to disambuguate or to give DelayMs or DelayNs
    //DelayMs::delay_ms(&mut delayx, 1000);
    //DelayNs::delay_ms(&mut delayx, 1000);
    /////////////////////////////////////////////////////////

    led.off();

    led.blink(2000_u16, &mut delay1); // Blink LED to indicate setup finished.

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(i2c2);

    //common display sizes are 128x64 and 128x32
    let mut display = Ssd1306::new(interface, DISPLAYSIZE, ROTATION)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

    Text::with_baseline(   "xca5948a \n aht10-display", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();

    led.blink(500_u16, &mut delay1); // Blink LED to indicate Ssd1306 initialized.
    //hprintln!("Text::with_baseline").unwrap();

    let mut screen: ScreenType = [R_VAL; DISPLAY_LINES];

    
    /////////////////////  xca   multiple devices on i2c bus
    let slave_address = 0b010_0000; // example slave address
    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent

    let mut switch1 = Xca9548a::new(i2c1, XcaSlaveAddr::default());

    // Enable channel 0
    switch1.select_channels(0b0000_0001).unwrap();

    // write to device connected to channel 0 using the I2C switch
    if switch1.write(slave_address, &write_data).is_err() {
        //hprintln!("Error write channel 0!").unwrap();
    }

    // read from device connected to channel 0 using the I2C switch
    let mut read_data = [0; 2];
    if switch1.read(slave_address, &mut read_data).is_err() {
        //hprintln!("Error read channel 0!").unwrap();
    }

    // write_read from device connected to channel 0 using the I2C switch
    if switch1
        .write_read(slave_address, &write_data, &mut read_data)
        .is_err()
    {
        //hprintln!("Error write_read!").unwrap();
    }

    show_message(&"AHT10s on xca", &mut display);

    /////////////////////  AHT10s on xca    // Start the sensors.
    
use embedded_hal::i2c::{I2c, ErrorType};

//type SensType<'a> =AHT10<I2cSlave<'a,  Xca9548a<I2c2Type>, I2c2Type>, AltDelay>;

//type SensType<'a> = Hdc20xx<I2cSlave<'a, Xca9548a<I2c<I2C1, Error = XcaError<ErrorType>>,  // I2C: ehal::I2c<Error = E>,
//                                         Error = ErrorType>, OneShot>;                     //  E: ehal::Error,

//    type SensType<'a> = Hdc20xx<I2cSlave<'a, Xca9548a<I2c<I2C1, Error = XcaError<ErrorType>>>, I2c<I2C1, Error = XcaError<ErrorType>>>, OneShot>;
//    type SensType<'a> = Hdc20xx<I2cSlave<'a, 
//                                         Xca9548a<I2c<I2C1, Error = XcaError<ErrorType>>>, I2C1> ,
//                                OneShot>;

//    const SENSITER: Option::<SensType> = None;      //const gives this `static lifetime
//    let mut sensors: [Option<SensType>; 16] = [SENSITER; 16];

//    let mut sensors: [SensType; 16];

    // Split the device and pass the virtual I2C devices to AHT10 driver
    let switch1parts = switch1.split();

    let parts  = [switch1parts.i2c0, switch1parts.i2c1, switch1parts.i2c2, switch1parts.i2c3,
                  switch1parts.i2c4, switch1parts.i2c5, switch1parts.i2c6, switch1parts.i2c7];
                //  switch2parts.i2c0, switch2parts.i2c1, switch2parts.i2c2, switch2parts.i2c3,
                //  switch2parts.i2c4, switch2parts.i2c5, switch2parts.i2c6, switch2parts.i2c7];

     let mut  sensors1 = Hdc20xx::new(parts[0], HdcSlaveAddr::default());
     let mut  sensors2 = Hdc20xx::new(parts[1], HdcSlaveAddr::default());

//    let mut i = 0;  // not very elegant
//    for  prt in parts {
//       sensors[i] = Hdc20xx::new(prt, HdcSlaveAddr::default());
//       //hprintln!("i  {}", i).unwrap();
//    let mut z = Hdc20xx::new(prt, HdcSlaveAddr::default());
//       //let z = AHT10::new(prt, AltDelay{});
//
//       screen[0].clear();
//       match z {
//           Ok(mut v) => {v.reset().expect("sensor01 reset failed");  //should handle this 
//                         sensors[i] = Some(v);
//                         //hprintln!("sensor J{} in use", i).unwrap();
//                         write!(screen[0], "J{} in use", i).unwrap();
//                       },
//           Err(_e)   => {//hprintln!("J{} unused", i).unwrap();
//                         write!(screen[0], "J{} unused", i).unwrap();
//                        },
//       }
//       //hprintln!("screen {:?}", screen).unwrap();
//       show_screen(&screen, &mut display);
//       delay1.delay_ms(500);
//       
//       i += 1;
//       //hprintln!("i+1 {}", i).unwrap();
//    };

    screen[0].clear();
    write!(screen[0], "    °C %RH").unwrap();

    //hprintln!("entering loop").unwrap();
    loop {   // Read humidity and temperature.
      let mut ln = 1;  // screen line to write. rolls if number of sensors exceed DISPLAY_LINES
//      for  i in 0..7 {
//          match   &mut sensors[i] {
//               None       => {},  //skip
//   
//               Some(sens) => {screen[ln].clear();
//                              match sens.read() {
        let data = block!(sensors1.read()).unwrap();  // does this need to block?

        screen[0].clear();
        screen[1].clear();
        write!(screen[0], "Temperature: {:.2}ºC  ", data.temperature).unwrap();
        write!(screen[1], "Humidity: {:.2}%  ", data.humidity.unwrap()).unwrap();
//                              match sensors1.read() {
//                                   Ok((h,t)) => {//hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
//                                                 write!(screen[ln], "J{} {:.1} {:.0}", i, t.celsius(), h.rh()).unwrap();
//                                                 write!(screen[ln], "J{} {:.1} {:.0}", 0, t.celsius(), h.rh()).unwrap();
//                                                },
//                                   //Err(e)    => {sens.reset().unwrap();
//                                   Err(e)    => {sensors1.reset().unwrap();
//                                                 //hprintln!("read error {:?}", e).unwrap();
//                                                 write!(screen[ln], "J{} read error. Reset{:?}", i, e).unwrap();
//                                                 write!(screen[ln], "J{} read error. Reset{:?}", 0, e).unwrap();
//                                                }
//                                   };
                              show_screen(&screen, &mut display);
                              ln += 1;
                              ln = ln % DISPLAY_LINES;
                              delay1.delay_ms(500);
//                              },
//           };          
//       };
       delay1.delay_ms(5000);
    }
}
 
