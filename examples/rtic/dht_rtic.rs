//! NOT YET WORKING. NEED RTIC VERSION OF SSD1306 FOR OLED. COMPARE display_stuff_rtic
//! 
//! Note that led and i2c pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.
//! 
//! Measure the temperature and humidity from a DHT11 or DHT22 on data pin (A8) and display on OLED with i2c.
//! (Specify feature "dht22"for DHT22).
//! Compare examples oled_dht, and blink_rtic.
//! Blink (onboard) LED with short pulse very read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main processe is scheduled. It reads the dht and spawns itself to run after aa delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [TIM3]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    //https://github.com/michaelbeaumont/dht-sensor
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::Reading;
    #[cfg(feature = "dht22")]
    use dht_sensor::dht22::Reading;
    use dht_sensor::*;

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    
    use systick_monotonic::*;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs
    use fugit::TimerDuration;


    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
//    use embedded_graphics::{
//        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
//        mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
//        pixelcolor::BinaryColor,
//        prelude::*,
//        text::{Baseline, Text},
//    };

//    use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};
    
    const MONOTICK:  u32 = 100;
    const READ_INTERVAL: u64 = 10;  // used as seconds

    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::dht_i2c_led_delay::{
        setup_dp, DhtPin, I2cType, LED, LedType, DelayType, DelayMs, MONOCLOCK};

    #[cfg(any(feature = "stm32f3xx", feature = "stm32l1xx", feature = "stm32f0xx"))]
    use embedded_hal::digital::v2::OutputPin;

//  THIS NEEDS TYPES FIGURED OUT FOR SHARE. SEE DISPLAY_STUFF_RTIC FOR SIMPLER EXAMPLE
//
//fn show_display<S>(
//    temperature: i8,
//    relative_humidity: u8,
//    text_style: MonoTextStyle<BinaryColor>,
//    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
//) -> ()
//where
//    S: DisplaySize,
//{
//   let mut lines: [heapless::String<32>; 1] = [
//       heapless::String::new(),
//   ];
//
//   // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
//   // It is possible to use \n in place of separate writes, with one line rather than vector.
//
//   // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
//   // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
//   // the line commented out!! The test_txt is taken from 
//   //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
//   
//   //let test_text  = "??????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????????";
//   //   degree symbol "??" is about                  ^^ here 
//   
//   write!(lines[0], "{:3}??C {:3}% RH", temperature, relative_humidity).unwrap();
//
//   disp.clear();
//   for i in 0..lines.len() {
//       // start from 0 requires that the top is used for font baseline
//       Text::with_baseline(
//           &lines[i],
//           Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
//           text_style,
//           Baseline::Top,
//       )
//       .draw(&mut *disp)
//       .unwrap();
//   }
//   disp.flush().unwrap();
//   ()
//}


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        //rtt_init_print!();
        //rprintln!("blink_rtic example");
        hprintln!("dht_rtic example").unwrap();

        //let mut led = setup(cx.device);
        let (mut dht, i2c, mut led, mut delay) = setup_dp(cx.device);

        led.on();
        delay.delay_ms(1000u32);  
        led.off();

        dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
        delay.delay_ms(2000_u32); //  2 second delay for dhtsensor initialization

        let _manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();
//
//        //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
//        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
//            .into_buffered_graphics_mode();
//        display.init().unwrap();
//
//        let text_style = MonoTextStyleBuilder::new()
//            .font(&FONT_10X20)
//            .text_color(BinaryColor::On)
//            .build();

        let mono = Systick::new(cx.core.SYST,  MONOCLOCK);

        // next turn LED for a period of time that can be used to calibrate the delay timer.
        // Ensure that nothing is spawned above. This relies on delay blocking.
        led.on();
        delay.delay_ms(10000u32);  
        led.off();
        delay.delay_ms(1000u32);  
        read_and_display::spawn().unwrap();

        hprintln!("exit init").unwrap();
        //(Shared {dht,  led, delay, text_style, display }, Local {}, init::Monotonics(mono))
        (Shared { led, }, Local {dht, delay, }, init::Monotonics(mono))
    }

    #[shared]
    struct Shared {
        led:   LedType,      //impl LED, would be nice
        //manager??? ,  uses i2c:   I2c2Type, or text_style, display ??
    }

    #[local]
    struct Local {
        dht:   DhtPin,
        delay: DelayType,
//        text_style: MonoTextStyle<BinaryColor>,
//        display: &mut Ssd1306<impl WriteOnlyDataCommand, DisplaySize, BufferedGraphicsMode<DisplaySize>>,
    }

    //#[task(shared = [led, delay, dht, text_style, display], capacity=2)]
    #[task(shared = [led, ], local = [dht, delay,], capacity=2)]
    fn read_and_display(cx: read_and_display::Context) {
        //hprintln!("read_and_display").unwrap();
        blink::spawn(BLINK_DURATION.millis()).ok();

        let delay = cx.local.delay;
        let dht = cx.local.dht;
        //let z = (delay, dht).lock(|delay, dht| { Reading::read(delay, dht) });  WHY DID THIS NOT NEED mut delay ?
        //let z = (delay).lock(|delay| { Reading::read(delay, dht) });             whereas this does need mut
        let z = Reading::read(delay, dht);
        let (_temperature, _humidity) = match z {
            Ok(Reading {temperature, relative_humidity,})
               =>  {hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                    //show_display(temperature, relative_humidity, text_style, &mut display)
                    (temperature, relative_humidity)
                   },
            Err(e) 
               =>  {hprintln!("dht Error {:?}", e).unwrap(); 
                    //panic!("Error reading DHT")
                    (25, 40)  //supply default values
                   },
        };

        read_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        crate::app::led_off::spawn_after(duration).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], capacity=2)]
    fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
