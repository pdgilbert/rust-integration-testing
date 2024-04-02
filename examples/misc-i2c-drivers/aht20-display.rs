//! Continuously read temperature from AHT20 and display on SSD1306 OLED.
//!
//!  Not sharing i2c bus.
//!
//!  The setup() functions make the application code common. They are in src/i2c1_i2c2_led_delay.rs;
//!
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.  Using VCC  3.3v.)

#![deny(unsafe_code)]
#![no_std]
#![no_main]

//use aht20_async::{Aht20};   consider async, but need one that is working
use aht20::{Aht20};

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

/////////////////////   hals

use embedded_hal::{
   delay::DelayNs,
};

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

use hal::{
      pac::{Peripherals, CorePeripherals},
};


#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    timer::SysTimerExt,
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


///////////////////// 

use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;
use rust_integration_testing_of_examples::led::{LED};

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("AHT10 example");
    //hprintln!("AHT10 example").unwrap();

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (i2c1, mut i2c2, mut led, mut delay, clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);
    let mut delay2 = cp.SYST.delay(&clocks); 

    // Blink LED to indicate initializing.
    led.blink(1000_u16, &mut delay);

    //let i2c1_ref_cell = RefCell::new(i2c1);
    //let aht_rcd   = RefCellDevice::new(&i2cset_ref_cell); 
    //let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(i2c1); //default address 0x3C
    //let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
    //let interface = I2CDisplayInterface::new_custom_address(ssd_rcd,   0x3D);  //alt address

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();
    display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
    
    Text::with_baseline(   "aht20-display", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
    display.flush().unwrap();
   
    delay.delay_ms(2000);    

    /////////////////////   aht

    // Start the sensor.
    let mut aht = Aht20::new(&mut i2c2, &mut delay);
    //let mut aht = Aht20::new(&mut aht_rcd, &mut delay).unwrap();  //.expect("aht device failed")
    //let mut aht = Aht20NoDelay::new(i2c2).unwrap();

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        //led.blink(20_u16, &mut delay);

        // Read humidity and temperature.
        let (h, t) = aht.read().unwrap();
        //let (h, t) = aht.end_read().unwrap();

        lines[0].clear();
        lines[1].clear();
        write!(lines[0], "temperature: {}C", t.celsius()).unwrap();
        write!(lines[1], "relative humidity: {0}%", h.rh()).unwrap();
        
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            Text::with_baseline(
                line,
                Point::new(0, i as i32 * VPIX),
                text_style,
                Baseline::Top,
            )
            .draw(&mut display)
            .unwrap();
        }
        display.flush().unwrap();

        delay2.delay_ms(5000); 
    }
}
