//  NOT COMPILING

// example from crate onewire README
//read the temperature from a DS18B20. 
//The example currently requires the stm32f103xx-hal to be patched with this PR.

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

use embedded_hal::digital::{InputPin, OutputPin};
use onewire;

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    timer::SysDelay as Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (PC13<Output<PushPull>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioc = p.GPIOC.split();

//    impl LED for PC13<Output<PushPull>> {
//        fn on(&mut self) -> () {
//            self.set_low()
//        }
//        fn off(&mut self) -> () {
//            self.set_high()
//        }
//    }

    // see examples in https://github.com/stm32-rs/stm32f1xx-hal/examples/
    //  for other (better) ways to do delay

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(&mut gpioc.crh), // led on pc13 with on/off
        cp.SYST.delay(&clocks)
    )
}


fn main() -> ! {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut gpioa = p.GPIOA.split();
    
    let mut delay = cp.SYST.delay(&clocks);  //Delay::new(cp.SYST, clocks);
    
    // open_drain_output is really input and output

    let mut one = gpioa
        .pa8
        .into_open_drain_output(&mut gpioa.crh);
        //.downgrade();   CHECK WHAT THIS WAS SUPPOSED TO DO
    
    
     // Pulling the pin high to avoid confusing the sensor when initializing.
     //dht.set_high();
    
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
                let mut ds18b20 = onewire::DS18B20::new(device).unwrap();
                
                // request sensor to measure temperature
                let resolution = ds18b20.measure_temperature(&mut wire, &mut delay).unwrap();
                
                // wait for compeltion, depends on resolution 
                delay.delay_ms(resolution.time_ms());
                
                // read temperature
                let temperature = ds18b20.read_temperature(&mut wire, &mut delay).unwrap();
            },
            _ => {
                // unknown device type            
            }
        }
    }
    
    loop {}
}
