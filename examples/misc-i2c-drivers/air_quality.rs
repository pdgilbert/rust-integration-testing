//! Eventually this example should do much more. For now it displays on SSD1306 OLED values from
//!   -INA219 current monitor connected to check battery usage
//!   -DHT-11 sensor temperature and humidity
//!
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.)
//!  The DHT11 data pin is connectted to pin A8 on the MCU board and has
//!  a pull up resistor. (18K ohms used in some testing.)
//!

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

//use cortex_m_semihosting::{debug, hprintln};
//use cortex_m_semihosting::{hprintln};
//use rtt_target::{rprintln, rtt_init_print};

use core::fmt::Write;

use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

//  note that larger font size increases memory and may require building with --release
//  &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
//  &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8 =   4  characters high
//  &FONT_4X6  128 pixels/ 4 per font =  32  characters wide.  32/6 =  5.3 characters high

//common display sizes are 128x64 and 128x32
const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;

const VPIX:i32 = 12; // vertical pixels for a line, including space

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyle, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::Reading;
#[cfg(feature = "dht22")]
use dht_sensor::dht22::Reading;
use dht_sensor::*;

use  ina219::{INA219,};

use rust_integration_testing_of_examples::dht_i2c_led_delay::{ setup_dp, DelayMs, LED, };

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::pac::Peripherals;

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::pac::Peripherals;

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::pac::Peripherals;

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::pac::Peripherals;

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::pac::Peripherals;

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::pac::Peripherals;

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::pac::Peripherals;

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::stm32::Peripherals;

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::pac::Peripherals;



fn display<S>(
    dht_temp: i8,
    dht_humidity: u8,
    voltage: u16,
    voltage_shunt: i16,
    current: i16,
    power: i16,
    power_calc: i32,
    text_style: MonoTextStyle<BinaryColor>,
    display: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 3] = [
        heapless::String::new(),
        heapless::String::new(),
        heapless::String::new(),
    ];

        write!(lines[0], "V: {}mv Vs: {}mV", voltage, voltage_shunt).unwrap();
        write!(lines[1], "I: {}mA P:{}mW [{}mW]", current, power, power_calc).unwrap();
        write!(lines[2], "{:3} C   RH {:3}%", dht_temp, dht_humidity).unwrap();

    display.clear();
    for (i, line) in lines.iter().enumerate() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            line,
            Point::new(0, i as i32 * VPIX),
            text_style,
            Baseline::Top,
        )
        .draw(&mut *display)
        .unwrap();
    }
    display.flush().unwrap();
    ()
}


#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let (mut dht, i2c, mut led, mut delay) = setup_dp(dp);
    
    led.on();

 //   led.blink_ok(&mut delay); // blink OK to indicate setup complete and main started.

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    let mut disp = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    disp.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT) 
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline(
        "Display initialized ...",
        Point::zero(),
        text_style,
        Baseline::Top,
    )
    .draw(&mut disp)
    .unwrap();

    let mut ina = INA219::new(manager.acquire_i2c(), 0x40);
    //hprintln!("let mut ina addr {:?}", INA219_ADDR).unwrap();  // crate's  INA219_ADDR prints as 65

    ina.calibrate(0x0100).unwrap();

    delay.delay_ms(1000_u32);
    led.off();

 //   led.blink(3000_u16, &mut delay);

    loop {
        // Blink LED to check that everything is actually running.
        // Note that blink takes about a mA current and might make current measurement noisy.
        //led.blink(10_u16, &mut delay);
        
        let z = Reading::read(&mut delay, &mut dht);
        let (dht_temp, dht_humidity) = match z {
            Ok(Reading {temperature, relative_humidity,})
               =>  {//hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                    //show_display(temperature, relative_humidity, text_style, &mut display)
                    (temperature, relative_humidity)
                   },
            Err(_e) 
               =>  {//hprintln!("dht Error {:?}", e).unwrap(); 
                    //panic!("Error reading DHT")
                    (25, 40)  //supply default values
                   },
        };

       
        let voltage = match ina.voltage() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let voltage_shunt = match ina.shunt_voltage() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let current = match ina.current() {
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let power = match ina.power() {  // ina indicated power
            Ok(v) => v,
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };
        
        // power caclulated by P=IV.  (mA x mV /1000 = mW)
        //If the ina is wired with Vin- to battery and Vin+ to load sign then the
        // display will show "+" for battery charging and "-" for discharging.
        let power_calc = current as i32 * voltage as i32 / 1000_i32;



        display(
            dht_temp, dht_humidity, voltage, voltage_shunt, current, power, power_calc, text_style, &mut disp,
        );

        delay.delay_ms(2000_u32); // sleep for 2s
    }
}
