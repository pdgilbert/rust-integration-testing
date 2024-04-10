//! Feb 5, 2023 - This is too large to load on bluepill. It is working with USB probe and with battery
//!        on blackpill stm32f401  with SSD1306 and HTU2XD on shared bus i2c1,
//!                                with SSD1306 and HTU2XD on shared bus i2c2,
//! 
//!             with USB probe freezes after display of example name
//!           with SSD1306 on shared bus i2c2 and HTU2XD on shared bus i2c1,  
//!           with SSD1306 on shared bus i2c1 and HTU2XD on shared bus i2c2,  
//! 
//!           with SSD1306 on shared bus i2c1 and HTU2XD on i2c2,  
//!           with SSD1306 on i2c1 and HTU2XD on i2c2,  
//!           with SSD1306 on i2c2 and HTU2XD on i2c1,  
//! 
//!             (sometimes giving panicked at 'sensor reset failed: ArbitrationLoss')
//! 
//!          This example has a  workaround for SSD1306  text_style.
//! 
//! Note that led and i2c pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.
//! 
//! Measure the temperature and humidity from an htu2xd and display on OLED with shared bus i2c1.
//! Compare examples dht_rtic, htu2xd_displau.
//! Blink (onboard) LED with short pulse evry read.
//! On startup the LED is set on for a second in the init process.
//! One main processe is scheduled. It reads the sensor and spawns itself to run after a delay.
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

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [TIM3]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use htu21df_sensor::{Sensor}; 
    //use htu2xd::{Htu2xd, Reading};   //, Resolution

    // Note that hprintln is for debugging with usb probe and semihosting. It causes battery operation to stall.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;

    use rtic;
    use rtic_monotonics::systick::Systick;
    use rtic_monotonics::systick::fugit::{ExtU32};


    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},   // shows degree symbol
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const READ_INTERVAL: u32 = 2;  // used as seconds

    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::setup::{MONOCLOCK, I2c1Type, I2c2Type, LED, LedType, Delay};

    use embedded_hal::delay::DelayNs;

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
       pac::TIM5,
       timer::Delay,
    };

    ////////////////////////////////////////////////////////////////////////////////////
    #[shared]
    struct Shared {
        led:   LedType,      //impl LED, would be nice
    }

    #[local]
    struct Local {
        //display:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>,
        display:  Ssd1306<I2CInterface<I2c1Type>,                                   
                          ssd1306::prelude::DisplaySize128x32, 
                          BufferedGraphicsMode<DisplaySize128x32>>,

        sensor:  htu21df_sensor::Sensor<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c2Type>>>>, 
        delay:  Delay,   //Delay<TIM5, 1000000_u32>, for stm32f4xx
    }
    ////////////////////////////////////////////////////////////////////////////////////


    fn show_display<S>(
        temperature: f32,   // 10 * deg C to give one decimal place
        relative_humidity: f32,
        //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();
    
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
       
       write!(lines[0], "{:.1}°C {:.0}% RH", temperature, relative_humidity).unwrap();

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
        //rprintln!("htu2xd_rtic example");
        //hprintln!("htu2xd_rtic example").unwrap();

        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        let (i2c1, i2c2, mut led, delay) = setup::i2c1_i2c2_led_delay_from_dp(cx.device);

        led.on();
        Systick.delay_ms(1000u32);  
        led.off();

        let manager2 = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>>::new(i2c2);
        //let manager2: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap(); 
    
        /////////////////////   ssd
        let interface = I2CDisplayInterface::new(i2c1); //default address 0x3C

        let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();

        //common display sizes are 128x64 and 128x32
        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        Text::with_baseline(   "HTU2XD-rtic", Point::zero(), text_style, Baseline::Top )
          .draw(&mut display).unwrap();
        display.flush().unwrap();
        
        Systick.delay_ms(2000u32);    

        /////////////////////   htu
        // Start the sensor.
         // delay or Systick::delay ?  Neither WILL WORK. Need DelayMs<u16>
         //  and being able to .await would ne nice
         let mut sensor = Sensor::new(manager2.acquire_i2c(), Some(&mut delay)).expect("sensor init");
        //                                                  ^^^^^^^^^^^^^^^^ the trait `DelayMs<u16>` is not implemented for `impl embedded_hal::delay::DelayNs`
        Systick.delay_ms(15u32);     // Wait for the reset to finish // needed?

        read_and_display::spawn().unwrap();

        (Shared { led, },   Local {display, sensor, delay})
    }

    ////////////////////////////////////////////////////////////////////////////////////

    #[task(shared = [led, ], local = [display, sensor, delay] )]
    async fn read_and_display(cx: read_and_display::Context) {
       let sensor = cx.local.sensor;
       let delay = cx.local.delay;

       loop {
          blink::spawn(BLINK_DURATION).ok();

     //     let z = sensor.read_temperature_blocking(htu_ch);
          // delay2 or Systick::delay?   NOT SURE IF THIS WILLL WORK. .await?
          let t = sensor.measure_temperature(&mut delay).unwrap().value();

     //   See htu2xd-display for error handling
     //     let t = match z {
     //         Ok(Reading::Ok(t))     => t.as_degrees_celsius(),
     //         Ok(Reading::ErrorLow)  => 409.0,
     //         Ok(Reading::ErrorHigh) => 409.1,
     //         Err(_)                 => 409.2,
     //     };

     //     let z = sensor.read_humidity_blocking(htu_ch);
          let h = sensor.measure_humidity(&mut delay).unwrap().value();
     //     let h = match z {
     //         Ok(Reading::Ok(t))     => t.as_percent_relative(),
     //         Ok(Reading::ErrorLow)  => 409.0,
     //         Ok(Reading::ErrorHigh) => 409.1,
     //         Err(_)                 => 409.,
     //     };

          show_display(t, h, cx.local.display);
          
          Systick::delay(READ_INTERVAL.secs()).await;
       }
    }

    #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
        // note that if blink is called with ::spawn_after then the first agument is the after time
        // and the second is the duration.
        //hprintln!("blink {}", duration).unwrap();
        crate::app::led_on::spawn().unwrap();
        Systick::delay(duration.millis()).await;
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
