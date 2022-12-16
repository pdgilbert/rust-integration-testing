// DS18B20 digital temperature sensor needs 4.7k pull-up resistor on data.
// The ds18b20 date wire is conncecte tp PA8 (specified in src/onewire.rs).
// The i2c for ssd1306 display is on pins as specifiied in function setup_i2c1
//  in src/i2c.rs (scl on PB8 and sda on PB9 for blackpill stm32f401 1nd stm32f411).
// The LED is on a pin as spcified in src/led.rs.
// 
// ERROR HANDLING NEEDS IMPROVEMENT
// 
// Following is using hints from README example at https://github.com/fuchsnj/ds18b20
//https://github.com/fuchsnj/one-wire-bus

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
//use cortex_m_semihosting::hprintln;

use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
use cortex_m_semihosting::hprintln;

use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::digital::v2::{InputPin, OutputPin};
use core::fmt::Debug;
use one_wire_bus::{OneWire, OneWireResult};


use ds18b20;
use ds18b20::{Ds18b20, Resolution};

//use rust_integration_testing_of_examples::delay::{DelayType};
//use rust_integration_testing_of_examples::onewire::{OneWireType};

use embedded_graphics::{
    //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};

use rust_integration_testing_of_examples::dp::{Peripherals};
use rust_integration_testing_of_examples::onewire_i2c_led_delay::{setup_onewire_i2c_led_delay_using_dp, LED, DelayMs};

// open_drain_output is really input and output

fn get_sensor<P, E>(
    delay: &mut (impl DelayMs<u16> + DelayUs<u16>),
    one_wire_bus: &mut one_wire_bus::OneWire<P>,
) -> OneWireResult<Ds18b20, E> //Option<Ds18b20>
    where
        P: OutputPin<Error=E> + InputPin<Error=E>,
        E: Debug
{
    // initiate a temperature measurement for all connected devices
    ds18b20::start_simultaneous_temp_measurement(one_wire_bus, delay)?;

    // wait for measurement depends on the resolution specified, which
    // can obtain it from reading the sensor data,
    // or just wait the longest time, which is the 12-bit resolution (750ms)
    Resolution::Bits12.delay_for_measurement_time(delay);
    // or
    delay.delay_ms(2000_u16); // Delay 2 seconds

    // iterate over all devices
    //according to  one_wire_bus  device_search:
    // Start the first search with a search_state of `None`, then use the returned state for subsequent searches
    // device_search() returns a Result(Option()) so an ok result may be None, 
    // meaning the search executed correctly but no device address was found on the bus.

    let mut search_state = None;
    let mut sensor = None;
    hprintln!("entering loop").unwrap();
    loop {
        let z = one_wire_bus.device_search(search_state.as_ref(), false, delay);
        hprintln!("Device at {:?}", z).unwrap();
       
        if let Some((device_address, state)) = one_wire_bus.device_search(search_state.as_ref(), false, delay)? {
            search_state = Some(state);
            hprintln!("first if").unwrap();
            if device_address.family_code() != ds18b20::FAMILY_CODE {
                // skip other devices
                hprintln!("second if").unwrap();
                continue;
            }
            sensor = Some(Ds18b20::new(device_address)?); // contains temperature and config info: resolution...

           hprintln!("Device at {:?} is °C", device_address).unwrap();
        } else {
            hprintln!("will now panic.").unwrap();
            break;
        }
    }
    Ok(sensor.unwrap())   //.unwrap() return `Result` or `Option` to accept `?`
}


fn show_display<S>(
    temperature: f32,
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 1] = [
        heapless::String::new(),
    ];

    // See example oled_dht for comments on oled and also relative humidity
    write!(lines[0], "{:3}°C ", temperature).unwrap();
 
    disp.clear();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
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
    let dp = Peripherals::take().unwrap();
    let (pin, i2c, mut led, mut delay) = setup_onewire_i2c_led_delay_using_dp(dp);

    let mut one_wire_bus = OneWire::new(pin).unwrap();

    led.blink(500_u16, &mut delay);  // to confirm startup

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    show_display(-300.0, text_style, &mut display);  //just to show display is working

    // get sensor address    
    let sensor = get_sensor(&mut delay, &mut one_wire_bus).unwrap();
      
    //hprintln!("endless loop. ^c to kill ...").unwrap();

    loop {
        // Blink LED to check that everything is actually running.
        led.blink(50_u16, &mut delay);

        let sensor_data = sensor.read_data(&mut one_wire_bus, &mut delay);
        
        //hprintln!("Device at {:?} is {}°C", device_address, sensor_data.temperature).unwrap();
       
        let temperature = sensor_data.unwrap().temperature;   //_or(-300)

        show_display(temperature, text_style, &mut display);

        //Delay before re-polling 
        delay.delay_ms(2000_u16); // Delay 2 seconds
    }
}
