//! Compare 
//!   display_stuff_rtic0 has only the display on the i2c bus and does not share the bus.
//!   display_stuff_rtic  has only the display on the i2c bus but is set up to share the bus.
//!   displayX2_rtic  has two displays on two i2c buses and is set up shares the buses.
//! 
//! Feb 6, 2023 - Testing with USB probe and with battery. Pre embedded-hal 1.0.0.
//! 
//!   Run tested on bluepill (scl,sda) i2c1 on (PB8,PB9) and i2c2 on (PB10,PB11). 
//!             Must be compiled --release to fit flash.
//!        Working with display_a on i2c1 and display_b on i2c2.
//!        Working with display_a on i2c1 and display_b on shared bus i2c2.
//!        Working with display_a on shared bus i2c1 and display_b on shared bus i2c2. 
//!         
//!   Run tested on blackpill (scl,sda) stm32f401 i2c1 on (PB8,PB9) and i2c2 on (PB10,PB3).
//!        Working with display_a on i2c1 and display_b on i2c2.
//!        Working with display_a on i2c1 and display_b on shared bus i2c2.
//!        Working with display_a on shared bus i2c1 and display_b on shared bus i2c2.
//! 
//! 
//! The maim purpose of this is to test the use of two i2c busses. 
//! Two ssd1306 displays minimizes other differences.
//! 
//! Measure the temperature and humidity from an AHT10 on data pin i2c2 and display on OLED with i2c1.
//! Blink (onboard) LED with short pulse evry read.
//! On startup the LED is set on for a second in the init process.
//! One main processe is scheduled. It writes to two displays and spawns itself to run after a delay.
//! It also spawns a `blink` process that turns the led on and schedules another process to turn it off.
//!

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

    // Note that hprintln is for debugging with usb probe and semihosting. It causes battery operation to stall.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;

    use rtic;
    use rtic_monotonics::systick::Systick;
    use rtic_monotonics::systick::fugit::{ExtU32};
   
    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const READ_INTERVAL: u32 = 2;  // used as seconds

    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::monoclock::MONOCLOCK;
    use rust_integration_testing_of_examples::led::{LED, LedType};
    use rust_integration_testing_of_examples::i2c::{I2c1Type, I2c2Type};
    use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;
    
    use embedded_hal::delay::DelayNs;

    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

use embedded_hal::i2c::I2c as I2cTrait; 
use embedded_hal_bus::i2c::RefCellDevice;

    fn show_display<S>(
       text: &str,
       i : &mut u16, 
       disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {         
        let mut line: heapless::String<32> = heapless::String::new();
        write!(line, "{} {}",  text, i).unwrap();
        show_message(&line, disp);
       ()
    }

    fn show_message<S>(
       text: &str, 
       disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {         
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       disp.clear_buffer();
       Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
               .draw(&mut *disp)
               .unwrap();

       disp.flush().unwrap();
       ()
    }


    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        let (i2c1, i2c2, mut led, _delay, _clock) = i2c1_i2c2_led_delay::setup_from_dp(cx.device);

        led.on();
        Systick.delay_ms(1000u32);  
        led.off();

    let i2c1_rc = RefCell::new(i2c1);
    let i2c1_rcd   = RefCellDevice::new(&i2c1_rc); 
    let interfaceA = I2CDisplayInterface::new(i2c1_rcd); //default address 0x3C
    //let interfaceA = I2CDisplayInterface::new_custom_address(i2c1_rcd,   0x3D);  //alt address


        let mut display_a = Ssd1306::new(interfaceA, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display_a.init().unwrap();
        
        show_message("displayX2_rtic", &mut display_a);   // Example name
        Systick.delay_ms(2000u32);  

    let i2c2_rc = RefCell::new(i2c2);
    let i2c2_rcd   = RefCellDevice::new(&i2c2_rc); 
    let interfaceB = I2CDisplayInterface::new(i2c2_rcd); //default address 0x3C
    //let interfaceB = I2CDisplayInterface::new_custom_address(i2c2_rcd,   0x3D);  //alt address

        let mut display_b = Ssd1306::new(interfaceB, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display_b.init().unwrap();
        
        show_message("displayX2_rtic", &mut display_b);   // Example name
        Systick.delay_ms(2000u32);  

        let count: u16 = 0;
        display_and_display::spawn().unwrap();

        (Shared { led, },   Local {display_a, display_b, count })
    }

    #[shared]
    struct Shared { led: LedType }


    #[local]
    struct Local {
        //delay: DelayType,

//        display_a:  Ssd1306<I2CInterface<I2c1Type>, 
//        display_a:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, 
        display_a:  Ssd1306<I2CInterface<RefCellDevice<'static, I2c1Type>>, 
                          ssd1306::prelude::DisplaySize128x32, 
                          BufferedGraphicsMode<DisplaySize128x32>>,
//        display_b:  Ssd1306<I2CInterface<I2c2Type>, 
//         display_b:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>, 
        display_b:  Ssd1306<I2CInterface<RefCellDevice<'static, I2c2Type>>, 
                          ssd1306::prelude::DisplaySize128x32, 
                          BufferedGraphicsMode<DisplaySize128x32>>,
        count: u16,
    }

    #[task(shared = [led, ], local = [display_a, display_b, count ])]
    async fn display_and_display(cx: display_and_display::Context) {
        loop {
           blink::spawn(BLINK_DURATION).ok();

           *cx.local.count = *cx.local.count + 1u16;
           show_display("display A", cx.local.count, cx.local.display_a);
           show_display("display B", cx.local.count, cx.local.display_b);

           Systick::delay(READ_INTERVAL.secs()).await;
       }
    }

    #[task(shared = [led])]
    async fn blink(_cx: blink::Context, duration: u32) {
        crate::app::led_on::spawn().unwrap();
        Systick::delay(duration.millis()).await;
        crate::app::led_off::spawn().unwrap();
    }

    #[task(shared = [led])]
    async fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led])]
    async fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
