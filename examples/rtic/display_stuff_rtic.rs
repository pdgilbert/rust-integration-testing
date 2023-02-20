//! Display "stuff" on OLED with i2c.
//! Compare examples dht_rtic.
//! Blink (onboard) LED with short pulse very read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main processe is scheduled. It writes to the display and spawns itself to run after an interval.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

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
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};

    //use core::fmt::Write;

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high

    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
        //mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle},  //FONT_6X10  FONT_8X13
        mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder},  //FONT_6X10  FONT_8X13
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    //use ssd1306::{prelude::*, Ssd1306, I2CDisplayInterface,mode::BufferedGraphicsMode};
//    use ssd1306::{prelude::{I2CInterface, DisplaySize, DisplaySize128x32, DisplayRotation},
//                  Ssd1306, I2CDisplayInterface,
//                  mode::BufferedGraphicsMode};
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    use systick_monotonic::*;
    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs
    
    use fugit::TimerDuration;

    const MONOTICK: u32 = 100;
    const READ_INTERVAL: u64 = 10;  // used as seconds

    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    //use rust_integration_testing_of_examples::led::{setup_led, LED, LedType};
    use rust_integration_testing_of_examples::i2c1_i2c2_led_delay::{
        setup_i2c1_i2c2_led_delay_using_dp, I2c1Type as I2cType, LED, LedType, MONOCLOCK};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;


    //type TextStyle = MonoTextStyle<'static, BinaryColor>;
    // = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();
    //const TEXTSTYLE: TextStyle = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();
    
    //type DisplayType = Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>;

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


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
       let mono = Systick::new(cx.core.SYST, MONOCLOCK);
       //rtt_init_print!();
       //rprintln!("isplay_stuff_rtic example");
       hprintln!("display_stuff_rtic example").unwrap();

       //let (i2c, mut led) = setup(cx.device);
       let (i2c, _i2c2, mut led, _delay) = setup_i2c1_i2c2_led_delay_using_dp(cx.device);

       led.on();

       let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

       let interface = I2CDisplayInterface::new(manager.acquire_i2c());

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

       //(Shared { led }, Local { display, text_style }, init::Monotonics(mono))
       (Shared { led }, Local { display }, init::Monotonics(mono))
    }

    #[shared]
    struct Shared {
        led: LedType,
    }
 

    #[local]
    struct Local {
        display:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, 
                          ssd1306::prelude::DisplaySize128x64, 
                          BufferedGraphicsMode<DisplaySize128x64>>,
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

    //#[task(shared = [led, ], local = [display, text_style,], capacity=2)]
    #[task(shared = [ led, ], local = [ display, ], capacity=2)]
    fn display_stuff(cx: display_stuff::Context) {
        // blink and re-spawn process to repeat
        blink::spawn(BLINK_DURATION.millis()).ok();

        // workaround. build here because text_style cannot be shared
        let text_style = MonoTextStyleBuilder::new().font(&FONT_10X20).text_color(BinaryColor::On).build();

        let lines: [heapless::String<32>; 1] = [heapless::String::new(),];
        //show_display(*cx.local.text_style, &mut cx.local.display);
        //show_display(&lines, cx.local.display);
        show_display(cx.local.display);

        // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
        // It is possible to use \n in place of separate writes, with one line rather than vector.
       
       // write!(&lines[0], "stuff").unwrap();
    
       cx.local.display.clear();
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
          
        display_stuff::spawn_after(READ_INTERVAL.secs()).unwrap();
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
