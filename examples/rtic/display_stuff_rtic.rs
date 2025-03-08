//! Display "stuff" on OLED with i2c.
//! Compare 
//!   display_stuff_rtic0 has only the display on the i2c bus and does not share the bus.
//!   display_stuff_rtic  has only the display on the i2c bus but is set up to share the bus.
//!   displayX2_rtic  has two displays on two i2c buses and is set up shares the buses.
//! 
//! Compare also example dht_rtic and several others that read sensors and display results.
//! 
//! Blink (onboard) LED with short pulse very read.
//! On startup the LED is set on for about (at least) 5 seconds in the init process.
//! One main process is scheduled. It writes to the display and pauses for an interval before re-writing.
//! The main process spawns a `blink` process that turns the led on and  another process to turn it off.

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

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
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
    use cortex_m_semihosting::{hprintln};

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

    use core::fmt::Write;

    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_6X10 as FONT, MonoTextStyleBuilder},  //FONT_6X10  FONT_8X13  FONT_10X20
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    use core::mem::MaybeUninit;

    const READ_INTERVAL: u32 = 10;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::
                         setup::{MONOCLOCK, I2c1Type, LED, LedType,}; // prelude::*, I2c};


    use core::cell::RefCell;
    //use embedded_hal_bus::i2c::RefCellDevice;
    //use shared_bus::{I2cProxy};
    use cortex_m::interrupt::Mutex;


    fn show_display<S>(
        //text: &[heapless::String<32>; 1],
        //text_style: TextStyle,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
        //disp: &mut DisplayType,
    ) -> ()
    where
        S: DisplaySize,
    {
       let lines: [heapless::String<32>; 1] = [heapless::String::new(),];
    
       // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
       // It is possible to use \n in place of separate writes, with one line rather than vector.
      
       //write!(lines[0], "stuff").unwrap();
       
       // should not be necessary to do this every call but have not yet managed to pass it as an arg
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       disp.clear_buffer();
       for i in 0..lines.len() {
           // start from 0 requires that the top is used for font baseline
           Text::with_baseline(
               &lines[i],
               Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space  8X13  14 = 13 + 1
               text_style,
               Baseline::Top,
           )
           .draw(&mut *disp)
           .unwrap();
       }
       disp.flush().unwrap();
       ()
    }


    ////////////////////////////////////////////////////////////////////////////////////

 NOT USING SHARED_BUS?? where does this come from?
    pub type DisplayType = Ssd1306<I2CInterface<shared_bus::I2cProxy<'static,  Mutex<RefCell<I2c1Type>>>>, 
                          ssd1306::prelude::DisplaySize128x64, 
                          BufferedGraphicsMode<DisplaySize128x64>>;
    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {
        //display:  Ssd1306<I2CInterface<RefCellDevice<I2c1Type>>, 
        display: DisplayType,
        //i2c: &'static mut I2CBus,
        //text_style: TextStyle,
        //text_style: MonoTextStyle<'static, BinaryColor>,
        // see https://github.com/jamwaffles/ssd1306/issues/164
    }


    ////////////////////////////////////////////////////////////////////////////////////

//    //#[init]
//    #[init(local=[
//        // Task local initialized resources are static
//        // Here we use MaybeUninit to allow for initialization in init()
//        // This enables its usage in driver initialization
//        i2c1_rcd: MaybeUninit<RefCellDevice<I2c1Type>> = MaybeUninit::uninit()
//    ])]
    #[init()]
    fn init(cx: init::Context) -> (Shared, Local ) {
       Mono::start(cx.core.SYST, MONOCLOCK);

       hprintln!("display_stuff_rtic example").unwrap();

       let (i2c1, mut led) = setup::i2c_led_from_dp(cx.device);

       led.on();

       //let i2c1_rc: RefCell<'static, I2c1Type>   = RefCell::new(i2c1);
  //     let i2c1_rc   = RefCell::new(i2c1);
  //     let i2c1_rcd  = RefCellDevice::new(&i2c1_rc); 
       
       let manager: &'static _ = shared_bus::new_cortexm!(I2c1Type = i2c1).unwrap(); 
       let interface = I2CDisplayInterface::new(manager.acquire_i2c()); //default address 0x3C  alt address 0x3D

       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

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


    #[task(shared = [ led, ], local = [ display, ] )]
    async fn display_stuff(cx: display_stuff::Context) {
       loop {
           // blink and re-spawn process to repeat
           blink::spawn(BLINK_DURATION).ok();

           // workaround. build here because text_style cannot be shared
           let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

           let mut lines: [heapless::String<32>; 5] = [heapless::String::new(), heapless::String::new(),
                              heapless::String::new(), heapless::String::new(), heapless::String::new(), ];
           write!(lines[0], "stuff").unwrap();
           write!(lines[1], "t").unwrap();
           write!(lines[2], "u").unwrap();
           write!(lines[3], "f").unwrap();
           write!(lines[4], "f").unwrap();

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
                  Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space  8X13  14 = 13 + 1
                  //cx.local.text_style,
                  text_style,
                  Baseline::Top,
              ).draw(cx.local.display).unwrap();
          }
          cx.local.display.flush().unwrap();
          Mono::delay(READ_INTERVAL.secs()).await;
       }        
    }

    #[task(shared = [led] )]
    async fn blink(_cx: blink::Context, duration: u32) {
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
