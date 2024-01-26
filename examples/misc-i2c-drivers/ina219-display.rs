//! Continuously read INA219 current monitor and display on SSD1306 OLED.
//!
//!  The setup() functions make the application code common. They are in src/i2c_led_delay.rs.
//!  The specific setup() function used will depend on the HAL setting (see README.md).
//!  See the section of setup() corresponding to the HAL setting for details on pin connections.
//!  (On "BluePill" (stm32f1xx_hal) scl is on PB8 and SDA is on PB9 using I2C1.)

#![deny(unsafe_code)]
#![no_std]
#![no_main]

//use rtt_target::{rprintln, rtt_init_print};

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;


/////////////////////   ina
use ina219::{address::{Address, Pin}, 
             SyncIna219,
             calibration::UnCalibrated
};

/////////////////////   ssd
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
// need more if there is a  function show_display()
//use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

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
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;

use embedded_hal::{
   //i2c::I2c as I2cTrait,
   delay::DelayNs,
};


use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

use hal::{
   pac::{Peripherals},
};


use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;
use rust_integration_testing_of_examples::led::{LED};

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("INA219 example");
    //hprintln!("INA219 example").unwrap();

    let dp = Peripherals::take().unwrap();

    let (i2cset, _i2c2, mut led, mut delay, _clocks) = i2c1_i2c2_led_delay::setup_from_dp(dp);

    led.off();
    delay.delay_ms(2000);
    led.blink(1000_u16, &mut delay);

    let i2cset_ref_cell = RefCell::new(i2cset);
    let ina_rcd   = RefCellDevice::new(&i2cset_ref_cell); 
    let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

    /////////////////////   ina
    let mut ina = SyncIna219::new( ina_rcd, Address::from_pins(Pin::Gnd, Pin::Gnd)).unwrap(); 
    ina.calibrate(UnCalibrated).unwrap();
    //ina.calibrate(0x0100).unwrap();

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
    //let interface = I2CDisplayInterface::new_custom_address(ssd_rcd,   0x3D);  //alt address

    let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();
    display.flush().unwrap();
    //hprintln!("display.flush").unwrap();


    let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    let mut lines: [heapless::String<32>; 2] = [heapless::String::new(), heapless::String::new()];

    ///////////////////// 
    led.blink(3000_u16, &mut delay);

    loop {
        //rprintln!("loop i");
        //hprintln!("loop i").unwrap();
        // Blink LED to indicate looping.
        led.blink(20_u16, &mut delay);

        lines[0].clear();
        lines[1].clear();
        
        let v = match ina.bus_voltage() {
            Ok(v) => v.voltage_mv(),
            Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let vs = match ina.shunt_voltage() {
            Ok(v) => v.shunt_voltage_uv(),
            Err(_e)   => 999i32  //write!(lines[0], "Err: {:?}", e).unwrap()
        };

        let i =  0;  //FAKE
       // let i = match ina.current() {
       //     Ok(v) => v,
       //     Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
       // };

        let p = 0;  //FAKE
       // let p = match ina.power() {  // ina indicated power
       //     Ok(v) => v,
       //     Err(_e)   => 999  //write!(lines[0], "Err: {:?}", e).unwrap()
       // };
        
        // power caclulated by P=IV.  (mA x mV /1000 = mW)
        //If the ina is wired with Vin- to battery and Vin+ to load sign then the
        // display will show "+" for battery charging and "-" for discharging.
        let pc = i as i32 * v as i32 / 1000_i32;

        write!(lines[0], "V: {}mv Vs: {}mV", v, vs).unwrap();
        write!(lines[1], "I: {}mA P:{}mW [{}mW]", i,  p, pc).unwrap();
        
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
        
        delay.delay_ms(2000);
    }
}

