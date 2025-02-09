//!   NOT HARDWARE TESTED SINCE EMBEDDED-HAL V1.0.0 CHANGES
//!
//! This uses analogue to digital chips ads1x1x on I2C2 to measure the battery.
//! Code FOR NOW has only one ads1x1x. Two are possible but require sharing the i2c bus.
//! (See ina219 examples for a better way to measure the battery.)
//! It also has an oled ssd display on  I2C1.

//! See examples/misc/battery_monitor_ads1015_rtic_dma.rs for version using dma.

#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;
//use panic_rtt_target as _;

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

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

    /////////////////////   ads
    //    use ads1x1x::{Ads1x1x, DynamicOneShot, FullScaleRange, TargetAddr, 
    //                  ChannelSelection,
    //                  ic::{Ads1015, Resolution12Bit},
    //                  interface::I2cInterface};
    
    use ads1x1x::{Ads1x1x, channel, FullScaleRange, TargetAddr,
                  ic::{Ads1015, Resolution12Bit},
    };

    /////////////////////   ssd
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    type DisplaySizeType = ssd1306::prelude::DisplaySize128x64;
    const DISPLAYSIZE: DisplaySizeType = DisplaySize128x64;

    const VPIX:i32 = 12; // vertical pixels for a line, including space. 12 for 128x32

    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    use core::fmt::Write;
    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20 as FONT, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };


    const READ_INTERVAL:  u32 =  2;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    /////////////////////  setup
    use rust_integration_testing_of_examples::setup;
    use rust_integration_testing_of_examples::
                         setup::{MONOCLOCK, I2c1Type, I2c2Type, LED, LedType, DelayNs};


//    use core::cell::RefCell;
//    use shared_bus::{I2cProxy};
//    use cortex_m::interrupt::Mutex;

//    use embedded_hal_bus::i2c::RefCellDevice;

    use nb::block;

    ////////////////////////////////////////////////////////////////////////

    // Note SCALE_CUR divides,  SCALE_B multiplies
    const  SCALE_CUR: i16 = 10; // calibrated to get mA/mV depends on FullScaleRange above and values of shunt resistors
    const  SCALE_A: i16 = 2; // calibrated to get mV    depends on FullScaleRange
//    const  SCALE_B: i16 = 2; // calibrated to get mV    depends on FullScaleRange

    //TMP35 scale is 100 deg C per 1.0v (slope 10mV/deg C) and goes through  <50C, 1.0v>,  so 0.0v is  -50C.
//    const  SCALE_TEMP:  i16  = 5; //divides
//    const  OFFSET_TEMP: i16 = 50;

    ////////////////////////////////////////////////////////////////////////

    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {    // NB I2CInterface  renamed I2cInterface ??
       adc_a:   Ads1x1x<I2c2Type, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
//       adc_a:   Ads1x1x<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
//       adc_b:   Ads1x1x<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,

       //display: Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, 
       display: Ssd1306<I2CInterface<I2c1Type>, DisplaySizeType, BufferedGraphicsMode<DisplaySizeType>>,
       //text_style: MonoTextStyle<BinaryColor>,
    }

    ////////////////////////////////////////////////////////////////////////

    fn show_display<S>(
        bat_mv: i16, 
        bat_ma: i16, 
        load_ma: i16, 
        temp_c: i16, 
        values_b: [i16; 3],
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
       
       write!(lines[0], "bat {:3}mv bat {:3}ma load {:3}ma temp{:3}°C v1{:3} v2{:3}  v3{:3} FIX ME", 
                         bat_mv, bat_ma, load_ma, temp_c, values_b[0], values_b[1], values_b[2],).unwrap();

       disp.clear_buffer();
       for i in 0..lines.len() {
           // start from 0 requires that the top is used for font baseline
           Text::with_baseline(
               &lines[i],
               Point::new(0, i as i32 * VPIX), //with font 6x10, 12 = 10 high + 2 space
               text_style,
               Baseline::Top,
               )
               .draw(&mut *disp)
               .unwrap();
       }
       disp.flush().unwrap();
       ()
    }

    ////////////////////////////////////////////////////////////////////////

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
  
        Mono::start(cx.core.SYST, MONOCLOCK);

        //rtt_init_print!();
        //rprintln!("battery_monitor_ads1015_rtic example");
        hprintln!("battery_monitor_ads1015_rtic example").unwrap();

        let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(cx.device);

        led.on(); 
        delay.delay_ms(1000);
        led.off();

        //////////////////////////////////  ssd
        
        let interface = I2CDisplayInterface::new(i2c1);

        let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();

        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        Text::with_baseline("battery_monitor_ads1015_rtic", Point::zero(), text_style, Baseline::Top, )
           .draw(&mut display).unwrap();
        display.flush().unwrap();
        
        delay.delay_ms(2000);    

        //////////////////////////////////  ads
        
//        let manager: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();

        let mut adc_a = Ads1x1x::new_ads1015(i2c2,  TargetAddr::Gnd);
//        let mut adc_b = Ads1x1x::new_ads1015(i2c2,  TargetAddr::Vdd);
//        let mut adc_a = Ads1x1x::new_ads1015(manager.acquire_i2c(),  TargetAddr::Gnd);
//        let mut adc_b = Ads1x1x::new_ads1015(manager.acquire_i2c(),  TargetAddr::Vdd);

        // FullScaleRange is very small for diff across low value shunt resistors for current
        //   but up to 5v when measuring usb power.
        // Set FullScaleRange to measure expected max voltage.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

        let z = adc_a.set_full_scale_range(FullScaleRange::Within0_256V);
        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        match z {
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_a.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

//        match adc_b.set_full_scale_range(FullScaleRange::Within4_096V) {
//            Ok(())   =>  (),
//            Err(e) =>  {hprintln!("Error {:?} in adc_2.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
//                        panic!("panic")
//                       },
//        };

        read_and_display::spawn().unwrap();

        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();

//        (Shared {led}, Local {adc_a, adc_b, display} )
        (Shared {led}, Local {adc_a, display} )
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). It may affect debugging.
           rtic::export::wfi()
        }
    }

//    #[task(shared = [led], local = [adc_a, adc_b, display], priority=1 )]
    #[task(shared = [led], local = [adc_a, display], priority=1 )]
    async fn read_and_display(mut cx: read_and_display::Context) {
       //let adc_a = *cx.local.adc_a.borrow_mut(); // extract from proxy
       let adc_a = cx.local.adc_a;
       //type `&mut Ads1x1x<I2cProxy<'static, cortex_m::interrupt::Mutex<RefCell<I2c<I2C2, PA8<AlternateOD<4>>, PA9<AlternateOD<4>>>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>`
       loop {
          Mono::delay(READ_INTERVAL.secs()).await;
          //hprintln!("measure").unwrap();
          blink::spawn(BLINK_DURATION).ok();

          adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();  // reading voltage which is higher 
          let bat_mv = block!(adc_a.read(channel::SingleA0)).unwrap_or(8091)* SCALE_A;
          adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

          //first adc  Note that readings will be zero using USB power (ie while programming) 
          // but not when using battery.

          let bat_ma =
              block!(adc_a.read(channel::DifferentialA2A3)).unwrap_or(8091) / SCALE_CUR;

          let load_ma =
              block!(adc_a.read(channel::DifferentialA2A3)).unwrap_or(8091) / SCALE_CUR;

//          // second adc
//          let values_b = [
//              block!(adc_b.read(channel::SingleA0)).unwrap_or(8091) * SCALE_B,
//              block!(adc_b.read(channel::SingleA1)).unwrap_or(8091) * SCALE_B,
//              block!(adc_b.read(channel::SingleA2)).unwrap_or(8091) * SCALE_B,
//          ];
//
//          let temp_c =
//              block!(adc_b.read(channel::SingleA3)).unwrap_or(8091) / SCALE_TEMP - OFFSET_TEMP;

           let temp_c = 0;   // FAKE
           let values_b = [0, 0, 0 ];   // FAKE

           show_display(bat_mv, bat_ma, load_ma, temp_c, values_b, &mut cx.local.display);
           
           hprintln!("bat_mv {:4}mV bat_ma {:4}mA  load_ma {:5}mA temp_c {}   values_b {:?}", bat_mv, bat_ma, load_ma, temp_c, values_b).unwrap();
       }
    }

    #[task(shared = [led], priority=1  )]
    async fn blink(_cx: blink::Context, duration: u32) {
        crate::app::led_on::spawn().unwrap();
        Mono::delay(duration.millis()).await;
        crate::app::led_off::spawn().unwrap();
    }

    #[task(shared = [led], priority=1  )]
    async fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], priority=1  )]
    async fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
