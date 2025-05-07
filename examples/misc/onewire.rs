//read the temperature from a DS18B20. 
// example modified from crate onewire README (https://github.com/kellerkindt/onewire)
//  Compiling but untested and NEEDS WORK 

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

// use panic_halt as _;  // put a breakpoint on `rust_begin_unwind` to catch panics
// use panic_abort; // may still require nightly?
// use panic_itm;   // logs messages over ITM; requires ITM support
// use panic_semihosting as _; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::entry;

use embedded_hal::delay::DelayNs;
use onewire;
use onewire::OpenDrainOutput;

use rust_integration_testing_of_examples::setup;
use rust_integration_testing_of_examples::setup::{Peripherals};

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let (mut one, mut delay) = setup::pin_delay_from_dp(dp);

    delay.delay_ms(1000);
    
    // open_drain_output is really input and output

    //let mut one = gpioa
    //    .pa8
    //    .into_open_drain_output(&mut gpioa.crh)
    //    .downgrade();   //CHECK WHAT THIS WAS SUPPOSED TO DO
   
    // Pulling the pin high to avoid confusing the sensor when initializing. NOT SURE IF THIS IS NEEDED
    let _ = one.set_high();
    
    let mut wire = onewire::OneWire::new(&mut one, false);
    
    if wire.reset(&mut delay).is_err() {
        // missing pullup or error on line
        loop {}
    }
    
    // search for devices
    let mut search = onewire::DeviceSearch::new();
    while let Some(device) = wire.search_next(&mut search, &mut delay).unwrap() {
        match device.address[0] {
            onewire::ds18b20::FAMILY_CODE => {
                let ds18b20 = onewire::DS18B20::new(device).unwrap();
                
                // request sensor to measure temperature
                let resolution = ds18b20.measure_temperature(&mut wire, &mut delay).unwrap();
                
                // wait for compeltion, depends on resolution 
                delay.delay_ms(resolution.time_ms().into());
                //delay.delay(resolution.millis());
                
                // read temperature
                let _temperature = ds18b20.read_temperature(&mut wire, &mut delay).unwrap();
            },
            _ => {
                // unknown device type            
            }
        }
    }
    
    loop {}
}
