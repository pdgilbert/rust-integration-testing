//!   NOT HARDWARE TESTED SINCE EMBEDDED-HAL V1.0.0 CHANGES
//! Continuously read temperature from htu21D sensor and display on SSD1306 OLED.
//!
//!  The setup() functions make the application code common. They are in src/.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.

//! Compare examples htu21D_rtic, aht10-display, aht10_rtic, dht_rtic, oled_dht, and blink_rtic.
//! Following https://github.com/samcrow/HTU2XD/blob/master/src/lib.rs


#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

/////////////////////   htu2xd
use htu21df_sensor::{Sensor}; 
//use htu2xd::{Htu2xd, Reading};   //, Resolution

/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
const VPIX:i32 = 12; // vertical pixels for a line, including space

use core::fmt::Write;
use embedded_graphics::{
    mono_font::{ascii::FONT_10X20 as FONT, MonoTextStyleBuilder},   //FONT_5X8   FONT_10X20
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

/////////////////////   hals
//use core::cell::RefCell;
//use embedded_hal_bus::i2c::RefCellDevice;

use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;
use rust_integration_testing_of_examples::led::{LED};
//use rust_integration_testing_of_examples::alt_delay::{AltDelay as OldDelayType};

use embedded_hal::{
   //i2c::I2c as I2cTrait,
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
    //timer::CountDownTimer,
    //delay::DelayFromCountDownTimer,
    //delay::Delay,
    //timer::SysDelay,
    delay::SYSTDelayExt,
    //pac::{TIM2, TIM3}, 
};

//#[cfg(feature = "stm32g4xx")]
//use embedded_hal_02::{
//   blocking::delay::DelayMs,    
//};


#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
   timer::Timer,
   //delay::Delay,
   delay::DelayFromCountDownTimer,
   pac::{TIM2, TIM5},
};


///////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("htu21D-display example");
    //hprintln!("htu21D-display example").unwrap();

    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let (i2c1, i2c2, mut led,  mut delay, clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);
    //let mut delayms: DelayMs<u32> = OldDelayType{};
    let mut delay2 = cp.SYST.delay(&clocks);

    /////////////// Note on Delay Type
    // let () = delay;    // type `impl embedded_hal::delay::DelayNs`
    // let () = delay2;   // type `SysDelay` with stm32f4xx;  type `Delay` with stm32g4xx
    // Either works for blink with stm32f4xx but only delay with stm32g4xx.
    // Only delay2 works for htu, with both  stm32f4xx and stm32g4xx.
    //    (trait `DelayMs<u16>` is not implemented)

    led.blink(1000_u16, &mut delay); // Blink LED to indicate setup finished.

    // both of these compile (using stm32f4xx_hal and stm32g4xx_hal, Feb 22, 2024)
    //let manager2 = shared_bus::BusManagerSimple::new(i2c2); 
    let manager2 = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>>::new(i2c2);

    // have not got this to compile yet
    //let i2c1_ref_cell = RefCell::new(i2c2);
    //let htu_rcd   = RefCellDevice::new(&i2c1_ref_cell); 
    //let ssd_rcd   = RefCellDevice::new(&i2c1_ref_cell); 

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

    Text::with_baseline(    "HTU2XD-display",Point::zero(), text_style, Baseline::Top )
        .draw(&mut display).unwrap();

    display.flush().unwrap();
    delay.delay_ms(2000);

    led.blink(500_u16, &mut delay); // Blink LED to indicate Ssd1306 initialized.

    /////////////////////   htu
    // Start the sensor.
    let mut htu = Sensor::new(manager2.acquire_i2c(), Some(&mut delay2)).expect("sensor init");
    //let mut htu = Sensor::new(htu_rcd, Some(&mut delay2)).expect("sensor init");
    //let mut htu = Sensor::new(i2c2, Some(&mut delay2)).expect("sensor init");


    //let mut htu    = Htu2xd::new();
    //let mut htu_ch = htu_rcd;

    //htu.soft_reset(i2c)?;
    //htu.soft_reset(&mut htu_ch).expect("sensor reset failed");

    // Wait for the reset to finish
    delay.delay_ms(15);

    //    .read_user_register() dos not return and changes something that requires sensot power off/on.
    //    let mut register = htu.read_user_register(&mut htu_ch).expect("htu.read_user_register failed");
    //    register.set_resolution(Resolution::Humidity10Temperature13);   //.expect("set_resolution failed");
    //    htu.write_user_register(&mut htu_ch, register).expect("write_user_register failed");

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        led.blink(10_u16, &mut delay);

        lines[0].clear();
        lines[1].clear();

        // using crate htu21df-sensor. commented out crate htu2xd.
        // Note that crate htu2xd consumes the delay. Crate htu21df-sensor does not, so
        //  calls to measure need the delay passed.

        let humidity: f32 = htu.measure_humidity(&mut delay2).expect("humidity").value();
        let temperature: f32 = htu.measure_temperature(&mut delay2).expect("temperature").value();

        write!(lines[0], "  {:.1} C", temperature).unwrap();
        write!(lines[1], "  {:.1} RH", humidity).unwrap();
        //let z = htu.measure_temperature(&mut delay2);
        //let () = z;

        //let z = htu.read_temperature_blocking(&mut htu_ch);
        //hprintln!("{:?}", z).unwrap();
        //  there is a double wrapping:  Ok(Ok(Temperature(24624)))

//        match z {
//            Ok(Reading::Ok(t))     => {//hprintln!("{} deg C", t.as_degrees_celsius()).unwrap();
//                                  write!(lines[0], "  {:.1} C", t.as_degrees_celsius()).unwrap(); },
//
//            Ok(Reading::ErrorLow)  => {//hprintln!("Error or off-scale low").unwrap();
//                                  write!(lines[0], "Error or off-scale low").unwrap(); },
//
//            Ok(Reading::ErrorHigh) => {//hprintln!("Error or off-scale high").unwrap();
//                                  write!(lines[0], "Error or off-scale high").unwrap(); },
//
//            Err(_)                 => {//hprintln!("Error reading temperature").unwrap();
//                                  write!(lines[0], "Error reading temperature").unwrap(); },
//        }
//
//        let z = htu.read_humidity_blocking(&mut htu_ch);
//
//        match z {
//            Ok(Reading::Ok(t))     => {//hprintln!("{}% RH", t.as_percent_relative()).unwrap();
//                                       write!(lines[1], "  {:.0}% RH", t.as_percent_relative()).unwrap();},
//
//            Ok(Reading::ErrorLow)  => {//hprintln!("Error or off-scale low").unwrap();
//                                  write!(lines[1], "humidity off-scale low").unwrap(); },
//
//            Ok(Reading::ErrorHigh) => {//hprintln!("Error or humidity off-scale high").unwrap();
//                                  write!(lines[1], "humidity off-scale high").unwrap(); },
//
//            Err(_)                 => {//hprintln!("Error reading humidity").unwrap();
//                                  write!(lines[1], "Error reading humidity").unwrap(); },
//        }

     
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

        delay.delay_ms(1000u32);
    }
}
