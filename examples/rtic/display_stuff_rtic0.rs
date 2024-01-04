//! Display "stuff" on OLED with i2c.
//! Compare examples display_stuff_rtic which adds shared i2c bus to display_stuff_rtic0.
//! (Crate shared_bus can cause additional problems.)
//! Compare also  examples dht_rtic.
//! Blink (onboard) LED with short pulse very read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main processe is scheduled. It writes to the display and spawns itself to run after an interval.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    use cortex_m_semihosting::{hprintln};

    //use core::fmt::Write;

    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},  //FONT_6X10  FONT_8X13
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    use rtic;
    use rtic_monotonics::systick::Systick;
    use rtic_monotonics::systick::fugit::{ExtU32};

    const READ_INTERVAL: u32 = 10;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::i2c1_i2c2_led;
    use rust_integration_testing_of_examples::monoclock::{MONOCLOCK};
    use rust_integration_testing_of_examples::led::{LED, LedType};

    use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

    use hal::{
        pac::{I2C1},
        i2c::I2c,
        //prelude::*,  // needed if 400.kHz() gives "you must specify a concrete type"
    };


    fn show_display<S>(
        //text: &[heapless::String<32>; 1],
        //text_style: TextStyle,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        //disp: DisplayType,
    ) -> ()
    where
        S: DisplaySize,
    {
       let lines: [heapless::String<32>; 1] = [
           heapless::String::new(),
       ];
    
       // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
       // It is possible to use \n in place of separate writes, with one line rather than vector.
      
       //write!(lines[0], "stuff").unwrap();
       
       // should not be necessary to do this every call but have not yet managed to pass it as an arg
       let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();
    
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

       let mono_token = rtic_monotonics::create_systick_token!();
       Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

       //rtt_init_print!();
       //rprintln!("isplay_stuff_rtic example");
       hprintln!("display_stuff_rtic example").unwrap();

       let (i2c, _i2c2, mut led, _clock) = i2c1_i2c2_led::setup(cx.device);

       led.on();

       let interface = I2CDisplayInterface::new(i2c);

       let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();

       let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
          .into_buffered_graphics_mode();

       display.init().unwrap();
       hprintln!("display.init",).unwrap();

       Text::with_baseline("Display initialized ...", Point::zero(), text_style, Baseline::Top, )
          .draw(&mut display).unwrap();
       hprintln!("Text::with_baseline",).unwrap();

       led.off();

       display_stuff::spawn().unwrap();

       //(Shared { led }, Local { display, text_style })
       (Shared { led }, Local { display })
    }

    #[shared]
    struct Shared {
        led: LedType,
    }
 

    #[local]
    struct Local {
        display: Ssd1306<I2CInterface<I2c<I2C1>>, 
                          ssd1306::prelude::DisplaySize128x64 , 
                          BufferedGraphicsMode<DisplaySize128x64>>
        //text_style: TextStyle,
        //text_style: MonoTextStyle<'static, BinaryColor>,
    }
// see https://github.com/jamwaffles/ssd1306/issues/164
//text_style cannot be in shared Local:
//(dyn GlyphMapping + 'static)` cannot be shared between threads safely
//    |
//    = help: within `MonoFont<'static>`, the trait `core::marker::Sync` is not implemented for `(dyn GlyphMapping + 'static)`
//    = note: required because it appears within the type `&'static (dyn GlyphMapping + 'static)`
//    = note: required because it appears within the type `MonoFont<'static>`
//    = note: required because of the requirements on the impl of `Send` for `&'static MonoFont<'static>`
//    = note: required because it appears within the type `embedded_graphics::mono_font::MonoTextStyle<'static, embedded_graphics::pixelcolor::BinaryColor>`
//note: required by a bound in `assert_send`

    #[task(shared = [ led, ], local = [ display, ] )]
    async fn display_stuff(cx: display_stuff::Context) {
       loop {
           // blink and re-spawn process to repeat
           blink::spawn(BLINK_DURATION).ok();

           // workaround. build here because text_style cannot be shared
           let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();

           let mut lines: [heapless::String<32>; 1] = [heapless::String::new(),];
           lines[0] = "s".into();
           lines[1] = "t".into();
           lines[2] = "u".into();
           lines[3] = "f".into();
           lines[4] = "f".into();

           //show_display(*cx.local.text_style, &mut cx.local.display);
           //show_display(&lines, cx.local.display);
           show_display(cx.local.display);

           // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
           // It is possible to use \n in place of separate writes, with one line rather than vector.
          
          // write!(&lines[0], "stuff").unwrap();
       
          cx.local.display.clear_buffer();
          for i in 0..lines.len() {
              // start from 0 requires that the top is used for font baseline
              Text::with_baseline(
                  &lines[i],
                  Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
                  //cx.local.text_style,
                  text_style,
                  Baseline::Top,
              ).draw(cx.local.display).unwrap();
          }
          cx.local.display.flush().unwrap();
          Systick::delay(READ_INTERVAL.secs()).await;
       }        
    }

    #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
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
