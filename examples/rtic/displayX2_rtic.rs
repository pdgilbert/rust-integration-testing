//! Feb 6, 2023 - Testing with USB probe and with battery. 
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
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::stm32,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::stm32, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {

    // Note that hprintln is for debugging with usb probe and semihosting. It causes battery operation to stall.
    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
    
    use core::fmt::Write;
    use systick_monotonic::*;
    use fugit::TimerDuration;
    
    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const MONOTICK:  u32 = 100;
    const READ_INTERVAL: u64 = 2;  // used as seconds

    const BLINK_DURATION: u64 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::i2c1_i2c2_led_delay::{
        setup_i2c1_i2c2_led_delay_using_dp, I2c1Type, I2c2Type, LED, LedType, DelayMs, MONOCLOCK}; //DelayType, 

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

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
    
       disp.clear();
       Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
               .draw(&mut *disp)
               .unwrap();

       disp.flush().unwrap();
       ()
    }


    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<MONOTICK>;

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        let (i2c1, i2c2, mut led, mut delay) = setup_i2c1_i2c2_led_delay_using_dp(cx.device);

        led.on();
        delay.delay_ms(1000u32);  
        led.off();

        //let managerA: &'static _ = shared_bus::new_cortexm!(I2c1Type = i2c1).unwrap();
        //let interfaceA = I2CDisplayInterface::new(managerA.acquire_i2c());
        let interfaceA = I2CDisplayInterface::new(i2c1);                //without shared bus

        let mut display_a = Ssd1306::new(interfaceA, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display_a.init().unwrap();
        
        show_message("displayX2_rtic", &mut display_a);   // Example name
        delay.delay_ms(2000u32);  

        let managerB: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();
        let interfaceB = I2CDisplayInterface::new(managerB.acquire_i2c());
        //let interfaceB = I2CDisplayInterface::new(i2c2);                //without shared bus

        let mut display_b = Ssd1306::new(interfaceB, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display_b.init().unwrap();
        
        show_message("displayX2_rtic", &mut display_b);   // Example name
        delay.delay_ms(2000u32);  

        let mono = Systick::new(cx.core.SYST,  MONOCLOCK);

        let count: u16 = 0;
        display_and_display::spawn().unwrap();

        (Shared { led, },   Local {display_a, display_b, count }, init::Monotonics(mono))
    }

    #[shared]
    struct Shared { led: LedType }


    #[local]
    struct Local {
        //delay: DelayType,

        display_a:  Ssd1306<I2CInterface<I2c1Type>, 
//        display_a:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, 
                          ssd1306::prelude::DisplaySize128x32, 
                          BufferedGraphicsMode<DisplaySize128x32>>,
//        display_b:  Ssd1306<I2CInterface<I2c2Type>, 
         display_b:  Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>, 
                          ssd1306::prelude::DisplaySize128x32, 
                          BufferedGraphicsMode<DisplaySize128x32>>,
        count: u16,
    }

    #[task(shared = [led, ], local = [display_a, display_b, count ], capacity=2)]
    fn display_and_display(cx: display_and_display::Context) {
        blink::spawn(BLINK_DURATION.millis()).ok();

        *cx.local.count = *cx.local.count + 1u16;
        show_display("display A", cx.local.count, cx.local.display_a);
        show_display("display B", cx.local.count, cx.local.display_b);

        display_and_display::spawn_after(READ_INTERVAL.secs()).unwrap();
    }

    #[task(shared = [led], capacity=2)]
    fn blink(_cx: blink::Context, duration: TimerDuration<u64, MONOTICK>) {
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
