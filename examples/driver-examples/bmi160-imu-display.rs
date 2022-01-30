//! Continuously read the accelerometer and gyroscope and print
//! the data to an SSD1306 OLED display.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!
//!  On "BluePill" (stm32f1xx_hal) using I2C1.
//! ```
//! BP   <-> BMI160 <-> Display
//! GND  <-> GND    <-> GND
//! 3.3V <-> VCC    <-> VDD
//! PB8  <-> SCL    <-> SCL
//! PB9  <-> SDA    <-> SDA
//! ```

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use bmi160::{
    AccelerometerPowerMode, Bmi160, Data, GyroscopePowerMode, Sensor3DData, SensorSelector,
    SlaveAddr,
};

use cortex_m_rt::entry;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use embedded_graphics::{
    mono_font::{ascii::FONT_5X8 as FONT, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use rust_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("BMI160 example");
    //hprintln!("BMI160 example").unwrap();

    let (i2c, mut led, mut delay) = setup();

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0) //128x64 128x32
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    //  note that larger font size increases memory and may require building with --release
    //  &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //  &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8 =   4  characters high
    //  &FONT_4X6  128 pixels/ 4 per font =  32  characters wide.  32/6 =  5.3 characters high
    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT)
        .text_color(BinaryColor::On)
        .build();

    let mut imu = Bmi160::new_with_i2c(manager.acquire_i2c(), SlaveAddr::Alternative(true));
    imu.set_accel_power_mode(AccelerometerPowerMode::Normal)
        .unwrap();
    imu.set_gyro_power_mode(GyroscopePowerMode::Normal).unwrap();

    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];
    let default_3ddata = Sensor3DData {
        x: -1,
        y: -1,
        z: -1,
    };
    let default_data = Data {
        accel: Some(default_3ddata),
        gyro: Some(default_3ddata),
        magnet: None,
        time: None,
    };

    loop {
        //hprintln!("loop i").unwrap();
        // Blink LED 0 to check that everything is actually running.
        // If the LED is off, something went wrong.
        led.blink(20_u16, &mut delay);

        let data = imu
            .data(SensorSelector::new().accel().gyro())
            .unwrap_or(default_data);
        let accel = data.accel.unwrap();
        let gyro = data.gyro.unwrap();

        lines[0].clear();
        lines[1].clear();
        write!(lines[0], "acc: x {} y {} z {}", accel.x, accel.y, accel.z).unwrap();
        write!(lines[1], "gyr: x {} y {} z {}", gyro.x, gyro.y, gyro.z).unwrap();
        display.clear();
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
