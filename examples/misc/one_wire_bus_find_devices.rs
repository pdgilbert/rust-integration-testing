//  With nothing on the bus this gives
//  found device at address 0000000000000000 with family code: 0x0


#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use core::fmt::Debug;
use one_wire_bus::{OneWire};   //, DeviceSearch

use rust_integration_testing_of_examples::onewire_i2c_led;

fn find_devices<P, E>(
    delay: &mut impl DelayNs,
    one_wire_pin: P,
) -> ()   //DeviceSearch< P, impl DelayNs>
    where
        P: OutputPin<Error=E> + InputPin<Error=E>,
        E: Debug
{
    let mut one_wire_bus = OneWire::new(one_wire_pin).unwrap();

    let z = one_wire_bus.devices(false, delay);
    //hprintln!("z = {:?}", z).unwrap();
  
// asm::bkpt();
  
    for device_address in z {
        // The search could fail at any time, so check each result. The iterator automatically
        // ends after an error.
 //        match device_address {
 //            Ok(address) 
 //               =>  {hprintln!("Found device at address {:?} with family code: {:#x?}", address, address.family_code());
 //                   //show_display(address, text_style, &mut display)
 //        hprintln!("Found device at address {:?} with family code: ",
 //                 device_address.unwrap()).unwrap();
 //                   },
 //            Err(e) 
 //               =>  {hprintln!("Error reading onewire bus address {:?}", e).unwrap(); 
 //                    //panic!("Error reading onewire bus address.")
 //                   },
 //        }

        // The family code can be used to identify the type of device
        // If supported, another crate can be used to interact with that device at the given address
        let device_address = device_address.unwrap();
        hprintln!("Found device at address {:?} with family code: {:#x?}",
                 device_address, device_address.family_code()).unwrap();
    }
    ()  //z
}
#[entry]
fn main() -> ! {
    let (pin, _i2c, _led, mut delay, _clocks) = onewire_i2c_led::setup();
    
    delay.delay_ms(1000);

// asm::bkpt();

    let _z = find_devices(&mut delay, pin);
    
// asm::bkpt();

    hprintln!("endless empty loop. ^c to kill ...").unwrap();
    loop {
    };
}
