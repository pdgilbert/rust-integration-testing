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

//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use cortex_m_rt::entry;

/////////////////////   bmi
use bmi160::{
    AccelerometerPowerMode, Bmi160, Data, GyroscopePowerMode, Sensor3DData, SensorSelector,
    SlaveAddr,
};


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
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
use hal::pac::{Peripherals};

use rust_integration_testing_of_examples::led::LED;
use rust_integration_testing_of_examples::setup;

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("BMI160 example");
    //hprintln!("BMI160 example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2cset, mut led, mut delay) = setup::i2c_led_delay_from_dp(dp);

    let i2cset_ref_cell = RefCell::new(i2cset);
    let bmi_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

    /////////////////////   bmi
    let mut imu = Bmi160::new_with_i2c(bmi_rcd, SlaveAddr::Alternative(true));
    imu.set_accel_power_mode(AccelerometerPowerMode::Normal)
        .unwrap();
    imu.set_gyro_power_mode(GyroscopePowerMode::Normal).unwrap();


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
    }
}
