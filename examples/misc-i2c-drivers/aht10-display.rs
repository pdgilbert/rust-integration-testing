//!   NOT HARDWARE TESTED SUCCESFULLY SINCE EMBEDDED-HAL V1.0.0 CHANGES
//!    FAILS STARTING SENSOR

//! Continuously read temperature from AHT10 and display on SSD1306 OLED.
//! The AHT10 does not work with other devices on the i2c bus 
//! (see https://www.electroschematics.com/temperature-sensor) so this example
//! requires two i2c buses. Note also using
//! "https://github.com/andy31415/aht10", branch = "fix_status_check"
//!
//!  The setup() functions make the application code common. They are in src/.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.

//! Compare examples aht10_rtic, dht_rtic, oled_dht, and blink_rtic.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

//use cortex_m_semihosting_05::hprintln;

use aht10::AHT10;
//use aht10_async::AHT10;

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};


use embedded_graphics::{
    mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::setup;

use rust_integration_testing_of_examples::setup::{Peripherals, CorePeripherals, LED};

use embedded_hal::delay::DelayNs;  //trait for `delay_ms`

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    timer::{SysTimerExt},
};

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    timer::{SysTimerExt},
};

#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    delay::SYSTDelayExt,
};

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
   timer::Timer,
   delay::DelayFromCountDownTimer,
   pac::{TIM2, TIM5},
};


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("AHT10 example");
    //hprintln!("AHT10 example");
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay, clocks) = setup::i2c1_i2c2_led_delay_clocks_from_dp(dp);
    let delay2 = cp.SYST.delay(&clocks); 

    // See more notes in example misc-i2c-drivers/htu2xd-display.rs re delay (and re manager)
    
    led.blink(3000_u16, &mut delay); // Blink LED to indicate setup finished.
    delay.delay_ms(2000_u32);  
    //led.blink(3000_u16, &mut delay2); // Blink LED to indicate setup finished.

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
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
    
    Text::with_baseline(   "aht10-display", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();
    //delay.delay(2000u32);    

    led.blink(500_u16, &mut delay); // Blink LED to indicate Ssd1306 initialized.
    //hprintln!("Text::with_baseline");

    // Start the sensor.
    // HARDWARE DOES NOT SEEM TO ALLOW SHARING THE BUS 
    // See https://www.electroschematics.com/temperature-sensor re default address 0x38  (vs possible alt 0x39)
    //   and "No  other devices on the I2C bus". So use i2c2 bus
    //hprintln!("mut sensor");
    let mut sensor = AHT10::new(i2c2, delay2).expect("sensor failed");
    //hprintln!("mut sensor done");
    Text::with_baseline(   "aht10 initialized", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();

    let _z = sensor.reset();
    //hprintln!("reset()  {:?}", z);
    Text::with_baseline(   "aht10 reset", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        //led.blink(20_u16, &mut delay);

        // Read humidity and temperature.
        // let (h, t) = sensor.read().unwrap();
        //hprintln!("sensor.read()").unwrap();
        let z = sensor.read();
        lines[0].clear();
        lines[1].clear();
        //hprintln!("match z").unwrap();
        // next recovers from sda disconnect/reconnect but not scl disconnect/reconnect
        match z {
            Ok((h,t)) => {//hprintln!("{} deg C, {}% RH", t.celsius(), h.rh()).unwrap();
                          write!(lines[0], "temperature: {}C", t.celsius()).unwrap();
                          write!(lines[1], "relative humidity: {0}%", h.rh()).unwrap();
                         },
            Err(e)    => {//hprintln!("Error {:?}", e).unwrap();
                          write!(lines[0], "sensor read error. Resetting.").unwrap();
                          write!(lines[1], "code {:?}", e).unwrap();
                          sensor.reset().unwrap();
                         }
        }

        //hprintln!(" matched");
      
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::with_baseline(
                line,
                Point::new(0, i as i32 * 16),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .unwrap();
        }
        display.flush().unwrap();
    }
}
