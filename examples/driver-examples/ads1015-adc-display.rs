//! Measure voltages with an ADS1015 analog/digital
//! converter and print them to an SSD1306 OLED display.
//! Further explanations about this device and how this example works at
//! https://blog.eldruin.com/ads1x1x-analog-to-digital-converter-driver-in-rust/
//! An image is [here](https://github.com/eldruin/driver-examples/blob/master/media/ads1015-voltage-divider.jpg).
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//!  ```
//!  BP  <-> ADS1015 <-> Display
//!  GND <-> GND     <-> GND
//!  +5V <-> +5V     <-> +5V
//!  PB9 <-> SDA     <-> SDA
//!  PB8 <-> SCL     <-> SCL
//!  ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use rtt_target::{rprintln, rtt_init_print};

use cortex_m_rt::entry;

/////////////////////   ads
use ads1x1x::{Ads1x1x, ChannelSelection, DynamicOneShot, FullScaleRange, SlaveAddr};

/////////////////////   ssd
use ssd1306::{ prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x64 = DisplaySize128x64;
const VPIX:i32 = 16; // vertical pixels for a line, including space. 12 for 128x32

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

/////////////////////   hals
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;

use embedded_hal::{
   delay::DelayNs,
};

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

use hal::{
   pac::{Peripherals},
   block,
};


use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;


#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("ADS1015 example");

    let dp = Peripherals::take().unwrap();

    let (i2cset, _i2c2, mut led, mut delay, _clock) = i2c1_i2c2_led_delay::setup_from_dp(dp);

    let i2cset_ref_cell = RefCell::new(i2cset);
    let adc_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

    /////////////////////   ads
    let mut adc = Ads1x1x::new_ads1015(adc_rcd, SlaveAddr::default()); //addr = Gnd
    // need to be able to measure [0-5V]
    adc.set_full_scale_range(FullScaleRange::Within6_144V).unwrap();

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


    let mut lines: [heapless::String<32>; 4] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];

    /////////////////////   measure and display in loop
    loop {
        // Blink LED to check that everything is actually running.
        // If the LED is off, something went wrong.
        led.blink(50_u16, &mut delay);

        // Read voltage in all channels
        let values = [
            block!(DynamicOneShot::read(&mut adc,  ChannelSelection::SingleA0)).unwrap_or(8091),
            block!(DynamicOneShot::read(&mut adc,  ChannelSelection::SingleA1)).unwrap_or(8091),
            block!(DynamicOneShot::read(&mut adc,  ChannelSelection::SingleA2)).unwrap_or(8091),
            block!(DynamicOneShot::read(&mut adc,  ChannelSelection::SingleA3)).unwrap_or(8091),
        ];

        display.clear_buffer();

        for i in 0..values.len() {
            write!(lines[i], "Channel {}: {}", i, values[i]).unwrap();
            Text::new(&lines[i], Point::new(0, i as i32 * VPIX), text_style)
                .draw(&mut display)
                .unwrap();
        }
        display.flush().unwrap();

        delay.delay_ms(2000); // sleep for 2s
    }
}
