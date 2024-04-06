//!   Hardware tested working with embedded-hal v1.0.0 on blackpill stm32f401, April 5, 2024.
//!      - current https://github.com/eldruin/embedded-ccs811-rs uses embedded-hal v0.2.7
//!        so this is using stm32f4xx_hal dual hal support.
//!      - using dongle power and semihosting with hprintln.
//!   Note that reading may take awhile (several minutes) to stabalize when started.
//!   (Semihosting and hprintln need to be removed to run on battery power.)
//!
//! Continuously measure the eCO2 and eTVOC in the air and print it to an SSD1306 OLED display.
//! This uses constant temperature and humidity rather than measuring. 
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
//!   (See note about possible need to update firmware.)
//!
//!  The setup() functions make the application code common. They are in src/.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//! Using I2C1 for oled display and  I2C2 for ccs811.
//! The hardware configuration for the STM32F103 "Bluepill" and  STM32F401 "Blackpill" boards use 
//!   I2C1 for oled:    PB8 for SCL, PB9 for SDA.
//!   I2C2 for ccs811: PB10 for SCL, PB3 for SDA,
//!    ccs811:  VCC 1.8 to 3.7v
//!    WAKE  GND to communicate. VCC puts sensor to sleep. (GND for this example)
//!    RST pull to GND to resets sensor (NC (floating)  for this example)
//!    INT  hardware interrupt output. (no connection for this example)
//!    ADDR altrnate address if available. Low hex 0x5A, high  hex 0x5B.
//!  

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;

/////////////////////   ccs
use embedded_ccs811::{prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode, SlaveAddr};


/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306}; //mode::BufferedGraphicsMode, 

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};


/////////////////////   hals

use embedded_hal::{
   delay::DelayNs,
};

use heapless::String;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

use hal::{
   pac::Peripherals,
   block,
};

/////////////////////  setups

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

use cortex_m_semihosting::{hprintln};

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("CCS811 example");

    let dp = Peripherals::take().unwrap();

    let (i2c1, i2c2, mut led, mut delay, _clock) = i2c1_i2c2_led_delay::setup_from_dp(dp);

    led.off();
    delay.delay_ms(1000);
    led.blink(2000_u16, &mut delay);

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(i2c1); //default address 0x3C
    //let interface = I2CDisplayInterface::new_custom_address(i2c1,   0x3D);  //alt address

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();
    //display.flush().unwrap();

    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

    Text::with_baseline("CCS811 example", 
            Point::new(0, 1 * VPIX), text_style, Baseline::Top,).draw(&mut display).unwrap();
    display.flush().unwrap();

    /////////////////////   ccs
    hprintln!("Ccs811Awake").unwrap();
    let mut ccs811 = Ccs811Awake::new(i2c2, SlaveAddr::default());
    hprintln!("software_reset").unwrap();
    ccs811.software_reset().unwrap();
    delay.delay_ms(500);

    hprintln!("ccs811.start_application").unwrap();
    let mut ccs811 = ccs811.start_application().ok().unwrap();
    hprintln!("set temperature_c").unwrap();
    let temperature_c = 22.0;
    let humidity_perc = 50.0;
    hprintln!("set_environment").unwrap();
    ccs811.set_environment(temperature_c, humidity_perc).unwrap();
    hprintln!("set_mode").unwrap();
    ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

    ///////////////////// set defaults and loop variables
    let default = AlgorithmResult {
        eco2: 9999,
        etvoc: 9999,
        raw_current: 255,
        raw_voltage: 9999,
    };

    let mut lines: [String<32>; 2] = [String::new(), String::new()];

    led.blink(5000_u16, &mut delay);
    hprintln!("loop").unwrap();

    /////////////////////    measure and display in loop
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(200_u16, &mut delay);

        let data = block!(ccs811.data()).unwrap_or(default);

        for line in lines.iter_mut() {
            line.clear();
        }
        write!(lines[0], "eCO2: {} ppm", data.eco2).unwrap();
        write!(lines[1], "eTVOC: {} ppb", data.etvoc).unwrap();
        display.clear_buffer();
        for (i, line) in lines.iter().enumerate() {
            //with font 6x10, 12 = 10 high + 2 space
            Text::with_baseline(line, Point::new(0, i as i32 * VPIX), text_style, Baseline::Top,)
                .draw(&mut display)
                .unwrap();
        }
        delay.delay_ms(5000);  
        display.flush().unwrap();
    }
}
