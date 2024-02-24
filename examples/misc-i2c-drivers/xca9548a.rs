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
//!  Beware that i2c1parts.i2c2 is the second multiplexed device on i2c1, whereas i2c2 is the MCU's second i2c.
//!  Beware that i2c2parts.i2c1 is the first multiplexed device on i2c2,  whereas i2c1 is the MCU's first  i2c.
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

//! Compare examples aht10-display, aht10_rtic, dht_rtic, oled_dht, and blink_rtic.

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


///////////////////////////////////////////////////////////////

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::{
      pac::{Peripherals, CorePeripherals, I2C2},
      i2c::Error as i2cError,
};



#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
   timer::SysTimerExt,  // trait for cp.SYST.delay
   timer::SysDelay,
};

#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{   delay::SYSTDelayExt }; // trait for cp.SYST.delay

#[cfg(feature = "stm32h7xx")]
use stm32g4xx_hal::{   delay::SYSTDelayExt }; // trait for cp.SYST.delay


///////////////////////////////////////////////////////////////

fn show_display<S>(
    s1: Result<(Humidity, Temperature), aht10Error<xca9548aError<i2cError>>>, 
    //s2: Result<(Humidity, Temperature), aht10Error<xca9548aError<i2cError>>>, 
    //text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: ssd1306::size::DisplaySize,  //trait
{
   let mut line: heapless::String<64> = heapless::String::new();
         
   match s1 {
        Ok((h,t)) => {hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
                      write!(line, "Sensor 2: {}C\nRH: {:3}%", t.celsius(), h.rh()).unwrap();
                     },
        Err(e)    => {hprintln!("Error {:?}", e).unwrap();
                      write!(line, "Sensor 2 read error. Resetting.code {:?}", e).unwrap();
                     }
    };
         
//   match s2 {
//        Ok((h,t)) => {hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
//                      write!(line, "Sensor 2: {}C\nRH: {:3}%", t.celsius(), h.rh()).unwrap();
//                     },
//        Err(e)    => {hprintln!("Error {:?}", e).unwrap();
//                      write!(line, "Sensor 2 read error. Resetting.code {:?}", e).unwrap();
//                     }
//    };
   
   //write!(lines[0], "{:.1}°C {:.0}% RH", temperature, relative_humidity).unwrap();
   // write!(lines[1], "{:.1}V {}mA {}mW [{}mW]", v as f32/1000.0, i,  p, pc).unwrap();
  
   show_message(&line, disp);
   ()
}

fn show_message<S>(
    text: &str,   //text_style: MonoTextStyle<BinaryColor>,
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


#[entry]
fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay1, clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);

    let delay2 = cp.SYST.delay(&clocks);
    //let () = delay1; opaque type impl DelayNs  does not work for AHT10, in AHT10::new()
    //let () = delay2;  type `Delay`

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
    //delay.delay1(2000u32);    

    led.blink(500_u16, &mut delay1); // Blink LED to indicate Ssd1306 initialized.
    hprintln!("Text::with_baseline").unwrap();

    // now multiple devices on i2c2 bus
    
    /////////////////////  xca
    let slave_address = 0b010_0000; // example slave address
    let write_data = [0b0101_0101, 0b1010_1010]; // some data to be sent

    let mut i2c1switch = Xca9548a::new(i2c2, SlaveAddr::default());

    // Enable channel 0
    i2c1switch.select_channels(0b0000_0001).unwrap();

    // write to device connected to channel 0 using the I2C switch
    if i2c1switch.write(slave_address, &write_data).is_err() {
        hprintln!("Error write channel 0!").unwrap();
    }

    // read from device connected to channel 0 using the I2C switch
    let mut read_data = [0; 2];
    if i2c1switch.read(slave_address, &mut read_data).is_err() {
        hprintln!("Error read channel 0!").unwrap();
    }

    // write_read from device connected to channel 0 using the I2C switch
    if i2c1switch
        .write_read(slave_address, &write_data, &mut read_data)
        .is_err()
    {
        hprintln!("Error write_read!").unwrap();
    }

    // Start the sensors.

    /////////////////////  AHT10
    type SensType<'a> =AHT10<I2cSlave<'a,  Xca9548a<stm32f4xx_hal::i2c::I2c<I2C2>>, stm32f4xx_hal::i2c::I2c<I2C2>>, SysDelay>;
    const SENSITER: Option::<SensType> = None;

    let mut sensors: [Option<SensType>; 16] = [SENSITER; 16];

    // Split the device and pass the virtual I2C devices to AHT10 driver
    let i2c1parts = i2c1switch.split();
    hprintln!("done i2c1parts").unwrap();

    // possible  to iter() over this, but need multiple delays  TRY ALT DELAY
    //let parts1 = [i2c1parts.i2c0, i2c1parts.i2c1];

    let mut sensor0 = AHT10::new(i2c1parts.i2c0, delay1).expect("sensor0 failed");
    sensor0.reset().expect("sensor00 reset failed");
    hprintln!("done sensor0.reset()").unwrap();

    let z = AHT10::new(i2c1parts.i2c1, delay2);
    match z {
        Ok(mut v) => {
                    v.reset().expect("sensor01 reset failed");  //should handle this 
                    sensors[1] = Some(v);
                    hprintln!("done sensor1.reset()").unwrap();
                   },
        Err(_e)    => hprintln!("1 not available").unwrap(),
    }


  //  let mut sensor2 = AHT10::new(i2c1parts.i2c2, delay2).expect("sensor2 failed");
  //  sensor2.reset().expect("sensor02 reset failed");

    loop {
        // Read humidity and temperature.   Sensor 0
        let s = sensor0.read();
        if s.is_err() { sensor0.reset().unwrap()};    //need delay here
        show_display(s, &mut display);
       
        // Read humidity and temperature.   Sensor 1
        match   &mut sensors[1] {
            None       => {},  //skip

            Some(sens) => {let s = sens.read();
                                if s.is_err() { sens.reset().unwrap()};
                                show_display(s, &mut display);
                               },
        };
               
        // Read humidity and temperature.   Sensor 2
     //   let s2 = sensor2.read();
     //   if s2.is_err() { sensor2.reset().unwrap()};    //need delay here

        //need delay here
        //show_display(s0, s1, &mut display);
    
    }
}
