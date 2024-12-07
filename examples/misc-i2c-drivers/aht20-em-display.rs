//! Continuously read temperature from AHT20 and display on SSD1306 OLED.
//!
//!  Not sharing i2c bus.
//!
//!  The setup() functions make the application code common. They are in src/i2c1_i2c2_led_delay.rs;
//!
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.  Using VCC  3.3v.)

//! Using crate embedded-aht20

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use embedded_aht20::{Aht20, DEFAULT_I2C_ADDRESS}; 

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

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, DelayNs,};
use rust_integration_testing_of_examples::setup::{CorePeripherals};

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("AHT10 example");
    //hprintln!("AHT10 example").unwrap();

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (i2c1, mut i2c2, mut led, mut delay, clocks) = setup::i2c1_i2c2_led_delay_clocks_from_dp(dp);
    let mut delay2 = cp.SYST.delay(&clocks); 

    // Blink LED to indicate initializing.
    led.blink(1000_u16, &mut delay);

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
    let mut aht  = Aht20::new(&mut i2c2, DEFAULT_I2C_ADDRESS, &mut delay).unwrap();

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        //led.blink(20_u16, &mut delay);

        // Read humidity and temperature.
        let th = aht.measure().unwrap();

        lines[0].clear();
        lines[1].clear();
        write!(lines[0], "temperature: {}C", th.temperature.celcius()).unwrap();
        write!(lines[1], "relative humidity: {0}%", th.relative_humidity).unwrap();
        
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
