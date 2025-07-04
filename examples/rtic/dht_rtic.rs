//!  Examples dht, dht_rtic, and oled_dht are similar and might be consolidated sometime.
//!  
//! Jan, 2023 - This is working with USB probe and with battery. Run tested on bluepill. 
//!             It has a  workaround for SSD1306  text_style.
//! 
//! Note that led and i2c pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.
//! 
//! Measure the temperature and humidity from a DHT11 or DHT22 on data pin (A8) and display on OLED with i2c.
//! (Specify feature "dht22"for DHT22).
//! Compare examples aht10_rtic, oled_dht, and blink_rtic.
//! Blink (onboard) LED with short pulse evry read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main processe is scheduled. It reads the dht and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.
//!
//!  A good reference on performance of humidity sensors is
//!     https://www.kandrsmith.org/RJS/Misc/hygrometers.html

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;
use rtic_monotonics::systick_monotonic;
systick_monotonic!(Mono, 1000); 

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [TIM3]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

    //https://github.com/michaelbeaumont/dht-sensor
    #[cfg(not(feature = "dht22"))]
    use dht_sensor::dht11::{blocking::read, Reading};
    #[cfg(feature = "dht22")]
    use dht_sensor::dht22::{blocking::read, Reading};
    //use dht_sensor::*;

    // Note that hprintln is for debugging with usb probe and semihosting. It causes battery operation to stall.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;
   
    const READ_INTERVAL: u32 = 10;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20 as FONT, MonoTextStyleBuilder, MonoTextStyle}, 
        //mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306,
                  prelude::DisplaySize128x32 as DISPLAYSIZE };

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::
                         setup::{MONOCLOCK, OpenDrainType, I2cType, LED, LedType, Delay, prelude::*,};





    //#[cfg(any(feature = "stm32f3xx", feature = "stm32l0xx",  feature = "stm32l1xx", feature = "stm32f0xx"))]
    //use embedded_hal::digital::OutputPin;

    fn show_display<S>(
        temperature: i8,
        relative_humidity: u8,
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       let mut lines: [heapless::String<32>; 1] = [
           heapless::String::new(),
       ];
    
       // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
       // It is possible to use \n in place of separate writes, with one line rather than vector.
    
       // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
       // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
       // the line commented out!! The test_txt is taken from 
       //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
       
       //let test_text  = "¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
       //   degree symbol "°" is about                  ^^ here 
       
       write!(lines[0], "{:3}°C{:3}%RH", temperature, relative_humidity).unwrap();

       disp.clear_buffer();
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

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
        //rtt_init_print!();
        //rprintln!("blink_rtic example");
        //hprintln!("dht_rtic example").unwrap();

        let (dht, i2c, mut led, mut delay) = setup::pin_i2c_led_delay_from_dp(cx.device);

        led.on();
        delay.delay_ms(1000);  
        led.off();

        delay.delay_ms(2000); //  2 second delay for dhtsensor initialization

        let interface = I2CDisplayInterface::new(i2c);

        let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        Text::with_baseline("dht_rtic", Point::zero(), text_style, Baseline::Top, )
          .draw(&mut display).unwrap();
        display.flush().unwrap();
        
        delay.delay_ms(2000);    

        // next turn LED for a period of time that can be used to calibrate the delay timer.
        // Ensure that nothing is spawned above. This relies on delay blocking.
        led.on();
        delay.delay_ms(10000);  
        led.off();
        delay.delay_ms(1000);  
        read_and_display::spawn().unwrap();

        Mono::start(cx.core.SYST, MONOCLOCK);

        //hprintln!("exit init").unwrap();
        (Shared { led, }, Local {dht, display, delay })
    }

    #[shared]
    struct Shared {
        led:   LedType,      //impl LED, would be nice
    }

// see disply_stuff_rtic and  https://github.com/jamwaffles/ssd1306/issues/164 regarding
// problem Shared local text_style. Workaroound by building in the task (may not be very efficient).

    #[local]
    struct Local {
        dht:   OpenDrainType,
        display: Ssd1306<I2CInterface<I2cType>, 
                          DISPLAYSIZE, 
                          BufferedGraphicsMode<DISPLAYSIZE>>,
        delay:   Delay,
    }

    #[task(shared = [led, ], local = [dht, display, delay ] )]
    async fn read_and_display(cx: read_and_display::Context) {

        let delay = cx.local.delay;
        let dht = cx.local.dht;

        loop {
           //hprintln!("read_and_display").unwrap();
           blink::spawn(BLINK_DURATION).ok();

           let z = read(delay, dht);   // needs a delay other than systick
           let (_temperature, _humidity) = match z {
               Ok(Reading {temperature, relative_humidity,})
                  =>  {//hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                       //show_display(temperature, relative_humidity, text_style, &mut display)
                       show_display(temperature, relative_humidity, cx.local.display);
                       (temperature, relative_humidity)
                      },
               Err(_e) 
                  =>  {//hprintln!("dht Error {:?}", e).unwrap(); 
                       //panic!("Error reading DHT")
                       (25, 40)  //supply default values
                      },
           };

           Mono::delay(READ_INTERVAL.secs()).await;
       }
    }

    #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        Mono::delay(duration.millis()).await;
        crate::app::led_off::spawn().unwrap();
    }

    #[task(shared = [led] )]
    async fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led] )]
    async fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
