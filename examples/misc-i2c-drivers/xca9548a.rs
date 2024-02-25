//! Continuously read temperature from multiple AHT10s and display on SSD1306 OLED.
//! The AHT10s are multiplexed on i2c2 using  xca9548a.
//!
//! TWO  AHT10s EACH CONSUME a DelayMs so more old delays are needed with eh-1 only stm32g4xx
//! ALSO, for more sensors it would be better to not consume delay. See htu2xd-display.
//! So with two sensors the two delays work
//! 
//! requires two i2c buses. Note also using
//! "https://github.com/andy31415/aht10", branch = "fix_status_check"
//! 
//!  Beware that switch1parts.i2c2 is the second multiplexed device on the i2c.
//! 
//!  The setup() functions make the application code common. They are in src/.
//!  The specific setup() function used will depend on the HAL feature specified (see HAL in README.md).
//!  See the section of setup functions corresponding to the HAL setting for details on pin connections.
//!  As of February 2024 the organization is:
//!   - `i2c1_i2c2_led_delay::setup_from_dp()` called below is from src/i2c1_i2c2_led_delay.rs.
//!   -  setup_from_dp() corresponds to feature stm32f4xx. -  setup_from_dp() 
//!   -  setup_from_dp() does some HAL specific extraction and configuration and
//!   -  setup_from_dp() calls setup_led from src/led.rs to setup the led and pin.
//!   -  setup_from_dp() calls setup_i2c1_i2c2 from src/i2c.rs to setup the i2c's and pins.
//!   -  Both setup_led and setup_i2c1_i2c2 again use the HAL setting for specific configuration.
//!  
//!  With HAL set to stm32f4xx  then
//!   -  setup_from_dp(), setup_led, and setup_i2c1_i2c2 all have sections for feature stm32f4xx. 
//!   -  For feature stm32f4xx setup_i2c1_i2c2 in src/i2c.rs sets i2c pins, possibly
//!               i2c1 (scl, sda)= (pb8,  pb9)
//!               i2c2 (scl, sda)= (pb10, pb3)

//! Compare examples aht20-display, aht10-display, aht10_rtic, dht_rtic, oled_dht, and blink_rtic.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use aht10::{AHT10, Humidity, Temperature, Error as aht10Error};

//use embedded_hal::i2c::Operation::{Read, Write};   // WriteRead, , Write
//use embedded_hal::prelude::_embedded_hal_blocking_i2c_Write;  // need trait for switch.write
use shared_bus::cortex_m::prelude::_embedded_hal_blocking_i2c_Read;
use shared_bus::cortex_m::prelude::_embedded_hal_blocking_i2c_Write;
use shared_bus::cortex_m::prelude::_embedded_hal_blocking_i2c_WriteRead;

use xca9548a::{Error as xca9548aError, SlaveAddr, Xca9548a, I2cSlave};

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;

// Need to run with debug console if printing is uncommented. 
// Running standalone stalls waiting to print.
//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    //mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    //mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    mono_font::{iso_8859_1::FONT_9X15 as FONT, MonoTextStyleBuilder}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAY_LINES: usize = 3;     // in characters
const DISPLAY_COLUMNS: usize = 32;  // in characters
const R_VAL: heapless::String<DISPLAY_COLUMNS> = heapless::String::new();
type  ScreenType = [heapless::String<DISPLAY_COLUMNS>; DISPLAY_LINES];


///////////////////////////////////////////////////////////////

use rust_integration_testing_of_examples::alt_delay::AltDelay; //cortex_m::asm::delay with traits
use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

use embedded_hal::delay::DelayNs;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::{
      pac::{Peripherals, I2C2},
      i2c::{I2c, Error as i2cError},
};


#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    gpio::{AlternateOD, gpioa::{PA8, PA9}},
    //delay::SYSTDelayExt }; // trait for cp.SYST.delay
};

//#[cfg(feature = "stm32h7xx")]
//use stm32g4xx_hal::{   delay::SYSTDelayExt }; // trait for cp.SYST.delay



#[cfg(feature = "stm32f4xx")]
type I2cType = I2c<I2C2>;

#[cfg(feature = "stm32g4xx")]
type I2cType = I2c<I2C2, PA8<AlternateOD<4_u8>>, PA9<AlternateOD<4_u8>>>;


type SensType<'a> =AHT10<I2cSlave<'a,  Xca9548a<I2cType>, I2cType>, AltDelay>;


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
   
   // workaround. build here because text_style cannot be shared
   let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

   hprintln!("in show_screen, screen={:?}", screen).unwrap();
   disp.clear_buffer();
   for  i in 0..2 {  // 3  DISPLAY_LINES
      if 0 != screen[i].len() {                         // 12 point per char verticle
         Text::with_baseline( &screen[i], Point::new(0, (i*12).try_into().unwrap()), text_style, Baseline::Top)
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

    let (i2c1, i2c2, mut led, mut delay1, _clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);


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
    let interface = I2CDisplayInterface::new(i2c1);

    //common display sizes are 128x64 and 128x32
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    //  note that larger font size increases memory and may require building with --release
    //  &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //  &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8 =   4  characters high
    //  &FONT_4X6  128 pixels/ 4 per font =  32  characters wide.  32/6 =  5.3 characters high

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

    Text::with_baseline(   "xca5948a \n aht10-display", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();

    led.blink(500_u16, &mut delay1); // Blink LED to indicate Ssd1306 initialized.
    hprintln!("Text::with_baseline").unwrap();

    let mut screen: ScreenType = [R_VAL; DISPLAY_LINES];

    
    /////////////////////  xca   multiple devices on i2c2 bus
    let slave_address = 0b010_0000; // example slave address
    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent

    let mut switch1 = Xca9548a::new(i2c2, SlaveAddr::default());

    // Enable channel 0
    switch1.select_channels(0b0000_0001).unwrap();

    // write to device connected to channel 0 using the I2C switch
    if switch1.write(slave_address, &write_data).is_err() {
        hprintln!("Error write channel 0!").unwrap();
    }

    // read from device connected to channel 0 using the I2C switch
    let mut read_data = [0; 2];
    if switch1.read(slave_address, &mut read_data).is_err() {
        hprintln!("Error read channel 0!").unwrap();
    }

    // write_read from device connected to channel 0 using the I2C switch
    if switch1
        .write_read(slave_address, &write_data, &mut read_data)
        .is_err()
    {
        hprintln!("Error write_read!").unwrap();
    }

    show_message(&"AHT10s on xca", &mut display);

    /////////////////////  AHT10s on xca    // Start the sensors.
    
    const SENSITER: Option::<SensType> = None;      //const gives this `static lifetime
    let mut sensors: [Option<SensType>; 16] = [SENSITER; 16];

    // Split the device and pass the virtual I2C devices to AHT10 driver
    let switch1parts = switch1.split();

    let parts  = [switch1parts.i2c0, switch1parts.i2c1, switch1parts.i2c2, switch1parts.i2c3,
                  switch1parts.i2c4, switch1parts.i2c5, switch1parts.i2c6, switch1parts.i2c7];
                //  switch2parts.i2c0, switch2parts.i2c1, switch2parts.i2c2, switch2parts.i2c3,
                //  switch2parts.i2c4, switch2parts.i2c5, switch2parts.i2c6, switch2parts.i2c7];

    let mut i = 0;  // not very elegant
    for  prt in parts {
       let z = AHT10::new(prt, AltDelay{});
       screen[0].clear();
       match z {
           Ok(mut v) => {v.reset().expect("sensor01 reset failed");  //should handle this 
                         sensors[i] = Some(v);
                         hprintln!("sensor J{} in use", i).unwrap();
                         write!(screen[0], "J{} in use", i).unwrap();
                       },
           Err(_e)   => {hprintln!("J{} unused", i).unwrap();
                         write!(screen[0], "J{} unused", i).unwrap();
                        },
       }
       hprintln!("screen {:?}", screen).unwrap();
       show_screen(&screen, &mut display);
       delay1.delay_ms(200);
       
       i += 1;
    };

    let mut ln = 0;  // screen line to write. rolls inside loop

    loop {   // Read humidity and temperature.
       for  i in 0..7 {
          match   &mut sensors[i] {
               None       => {},  //skip
   
               Some(sens) => {screen[ln].clear();
                              match sens.read() {
                                   Ok((h,t)) => {hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
                                                 write!(screen[ln], "J{} {:.1}C {:.0}%RH", i, t.celsius(), h.rh()).unwrap();
                                                },
                                   Err(e)    => {sens.reset().unwrap();
                                                 hprintln!("read error {:?}", e).unwrap();
                                                 write!(screen[ln], "J{} read error. Reset{:?}", i, e).unwrap();
                                                }
                                   };
                              show_screen(&screen, &mut display);
                              ln += 1;
                              ln = ln % DISPLAY_LINES;
                              hprintln!("ln+={} screen={:?}", ln, screen).unwrap();
                              delay1.delay_ms(500);
                              },
           };          
       };
       delay1.delay_ms(5000);
    }
}
 
