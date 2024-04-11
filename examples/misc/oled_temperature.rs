//! Measure temperature with different sensors and display on SSD1306 OLED display.
//! The internal mcu temperature is not measured. See misc/temperature.rs for that.
//!
//! Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display.
//! This example borrows from  driver-examples/ads1015-adc-display.rs,  misc/dht.rs
//! See also misc/battery_monitor_ads1015.rs regarding ads1015 addr and FullScaleRange setting options
//!  and regarding separating read and display into functions.
//!
//! The 10K thermistor voltage is measured with an ADS1015 (12-bit) or ADS1115 (16-bit) analog/digital converter.
//!
//! The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//! The specific function used will depend on the HAL setting (see README.md).
//!
//! On "BluePill" (stm32f1xx_hal) the I2C1 connections are SDA on PB9, SCL on PB8. DHT as below.
//! See src/i2c_led_delay.rs for settings on other boards. The section of setup() corresponding 
//! to the HAL setting specifies details of pin connections.
//! If 3.3v is supplied through BluePill from 5v BEWARE of regulator limit.
//! 
//! The SSD1306 OLED display connects to the I2C bus: VCC  (or VDD) to 3.3v, and also to GND, SDA, and SCL. 
//! The ADS1015 analog digital converter connects to the I2C bus: V to 3.3v, and also to GND, SDA, and SCL. 
//!    ADDR is connect to GND. ALERT has no connection.
//! 
//! The 10K thermistor connects to 3.3v and is in series with a 10K fixed resistor which connects to GND.
//! (see for example https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html
//!  or  https://learn.adafruit.com/thermistor/using-a-thermistor
//! Voltage at the thermistor to fixed resistor connection is measured on ADS1015 pin  ???
//!  
//!
//! The TMP36 connections are
//!  ```
//!  ```
//! The DHT11 connections are 3.3v, GND, pull-up resistor (~10K) from 3.3v to data,  data to PA8.
//!  ```
//!  ```
//! POSSIBLY The DS18B20 connections are
//!  ```
//!  ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]


use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

/////////////////////   ads
use ads1x1x::{Ads1x1x, ChannelSelection, DynamicOneShot, FullScaleRange, SlaveAddr};


/////////////////////   ssd
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAY_LINES: usize = 3; 

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space
// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
// DisplaySize128x32:
//    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
//    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
//    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

/////////////////////   hals
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;

use embedded_hal::{
   //i2c::I2c as I2cTrait,
   delay::DelayNs,
};

/////////////////////  setup
use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals, LED, block};


///////////////////////////////

fn show_display<S>(
    thermistor: i16,
    temp1: i16,
    temp2: i16,
    temp3: i16,
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; DISPLAY_LINES] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // it is now possible to use \n in place of separate writes, with one line rather than vector.
    write!(lines[0], "thermistor{:3} C", thermistor).unwrap();
    write!(lines[1], "temp1:    {:5} C", temp1).unwrap();
    write!(lines[2], "t2 {:5} C t3 {:5} C", temp2, temp3).unwrap();

    disp.clear_buffer();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * VPIX), //using font 6x10, 12 = 10 high + 2 space
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("temperature_display example");
    hprintln!("temperature_display example").unwrap();

    let dp = Peripherals::take().unwrap();
    let (i2cset, mut led, mut delay) = setup::i2c_led_delay_from_dp(dp);

    let i2cset_ref_cell = RefCell::new(i2cset);
    let adc_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let ssd_rcd = RefCellDevice::new(&i2cset_ref_cell); 

    led.blink(500_u16, &mut delay);  // to confirm startup

    /////////////////////   ads
    let mut adc = Ads1x1x::new_ads1015(adc_rcd, SlaveAddr::default()); //addr = Gnd

    // to measure [0-5V] use FullScaleRange::Within6_144V
    adc.set_full_scale_range(FullScaleRange::Within6_144V).unwrap();
    //adc.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();


    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
    //let interface = I2CDisplayInterface::new_custom_address(ssd_rcd,   0x3D);  //alt address

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    ///////////////////// 
    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 is off, something went wrong.
        led.blink(50_u16, &mut delay);

        // Read adc
        let thermistor = block!(DynamicOneShot::read(&mut adc, ChannelSelection::SingleA0)).unwrap_or(8091) * 5;  //FIX
        let a1 = block!(DynamicOneShot::read(&mut adc, ChannelSelection::SingleA1)).unwrap_or(8091);
        let a2 = block!(DynamicOneShot::read(&mut adc, ChannelSelection::SingleA2)).unwrap_or(8091);
        let a3 = block!(DynamicOneShot::read(&mut adc, ChannelSelection::SingleA3)).unwrap_or(8091);

    hprintln!("values read").unwrap();
    hprintln!("values  {} {} {} {}", thermistor, a1, a2, a3).unwrap();

        show_display(thermistor, a1, a2, a3, text_style, &mut display);

        delay.delay_ms(5000);
    }
}
