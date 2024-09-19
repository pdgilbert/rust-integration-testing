//! THIS NEEDS TO SHARE MULTIPLE ADCs ON I2C1 AND IS NOT WORKING.
//! Compare 
//!    temperature-display_4jst which does not need share but reads only 4 ADCs. And has more documentation.
//!    battery_monitor_ads1015_rtic which is currently not sharing
//!    therm10k_display
//!    htu2xd_rtic which has sensor on shared bus and 'static,         let manager2: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap(); 
//!    ads1015-adc-display uses  RefCellDevice but is not rtic
//!    ccs811-co2-voc  uses  RefCell  for sensor and rtic  AND SEE NOTES  Jan 2024
//!    temp-humidity-display

//! TARGET Measure temperature with multiple 10k thermistor sensors (NTC 3950 10k thermistors probes) 
//! using multiple channel adc and crate ads1x1x. Display using SSD1306.


//!  WORK IN PROGRESS.

//! https://www.ametherm.com/thermistor/ntc-thermistor-beta


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
    
    use ads1x1x::{Ads1x1x, ic::Ads1115, ic::Resolution16Bit, channel, FullScaleRange, TargetAddr};

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use cortex_m::asm;

    use core::fmt::Write;

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;


    /////////////////////   ssd
    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};


    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high

    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    //I2CInterface is from ssd1306::prelude 
    type  DisplayType =  Ssd1306<I2CInterface<I2c2Type>,                            
                          ssd1306::prelude::DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;

    const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x64 = DisplaySize128x64;

    //const DISPLAY_LINES: usize = 3; 
    //const VPIX:i32 = 12; // vertical pixels for a line, including space



    ///////////////////// 

    const READ_INTERVAL:  u32 = 5;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    
    // The units of measurement are  [32767..-32768] for 16-bit devices (and  [2047..-2048] for 12-bit devices)
    // but only half the range is used for single-ended measurements. (Precision could be improved by using
    // one input set at GND as one side of a differential measurement.)
    // set FullScaleRange to measure expected max voltage.

    // SCALE is measured units per mV, so the adc measurement is divided by SCALE to get mV.
    // Calibrated to get mV depends on FullScaleRange.
    // adc.set_full_scale_range(FullScaleRange::Within4_096V) sets adc max to 4.096v.
    // Check by connecting J2 +pin to 3.3v on blackpill and J2 -pin NC.
    // (Also J1 +pin to GND on blackpill to check 0, but this will cause power draw through 10k).
    // (blackpill pin measures  3200mv by meter, so measurement/SCALE should give 3200mv.)

    const  SCALE: i64 = 8 ;  // = 32767 / 4096

        // The FullScaleRange needs to be small if measuring diff across low value shunt resistors for current
        //   but would be up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

    use rust_integration_testing_of_examples::setup::{MONOCLOCK, LED, LedType, I2c1Type, I2c2Type};
    use rust_integration_testing_of_examples::setup;

    use embedded_hal::delay::DelayNs;

    use nb::block;

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    //use embedded_hal_bus::i2c::RefCellDevice;
    //use shared_bus::{I2cProxy};
    use cortex_m::interrupt::Mutex;


    fn show_display<S>(
        v_a: [i16; 4],
        v_b: [i16; 4],
        v_c: [i16; 4],
        v_d: [i16; 4],
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {    
       let mut line: heapless::String<128> = heapless::String::new(); // \n to separate lines
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line, "A1-4 {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C ", 
            v_a[0]/10,v_a[0]%10,  v_a[1]/10,v_a[1]%10,  v_a[2]/10,v_a[2]%10,  v_a[2]/10,v_a[2]%10,).unwrap();

       write!(line, "\nB1-4 {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C ", 
            v_b[0]/10,v_b[0]%10,  v_b[1]/10,v_b[1]%10,  v_b[2]/10,v_b[2]%10,  v_b[2]/10,v_b[2]%10,).unwrap();

       write!(line, "\nC1-4 {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C ", 
            v_c[0]/10,v_c[0]%10,  v_c[1]/10,v_c[1]%10,  v_c[2]/10,v_c[2]%10,  v_c[2]/10,v_c[2]%10,).unwrap();

       write!(line, "\nD1-4 {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C ", 
            v_d[0]/10,v_d[0]%10,  v_d[1]/10,v_d[1]%10,  v_d[2]/10,v_d[2]%10,  v_d[2]/10,v_d[2]%10,).unwrap();

       show_message(&line, disp);
       ()
    }

    fn show_message<S>(
        text: &str,   //text_style: MonoTextStyle<BinaryColor>,
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: ssd1306::size::DisplaySize,  //trait
    {
       
       // workaround. build here because text_style cannot be shared
       let text_style = MonoTextStyleBuilder::new().font(&FONT).text_color(BinaryColor::On).build();
    
       disp.clear_buffer();
       Text::with_baseline( &text, Point::new(0,0), text_style, Baseline::Top)
               .draw(&mut *disp)
               .unwrap();

       disp.flush().unwrap();
       ()
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        Mono::start(cx.core.SYST, MONOCLOCK);

        //hprintln!("temperature-display example").unwrap();

        let (i2c1, i2c2, mut led, _delay) = setup::i2c1_i2c2_led_delay_from_dp(cx.device);
        //hprintln!("setup done.").unwrap();

        led.on(); 
        Mono.delay_ms(1000u32);
        led.off();

        // As of Feb 2024 I2CDisplayInterface::new is not working with shared bus.
        // (No luck Using embedded-bus instead.
        // Try ssd on i2c2 and shared-bus ads's on i2c1
        //let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c2).unwrap();
        let manager1: &'static _ = shared_bus::new_cortexm!(I2c1Type = i2c1).unwrap();


       //let i2c1_ref_cell = RefCell::new(i2c1);
       //let adc_a_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let adc_b_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let adc_c_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let adc_d_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let ina_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let ssd_rcd   = RefCellDevice::new(&i2c1_ref_cell); 
       //let interface = I2CDisplayInterface::new(ssd_rcd);
        let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C
        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        show_message(" temperature", &mut display);
        Mono.delay_ms(2000u32);    
        //hprintln!("display initialized.").unwrap();

         
        // ADS11x5 chips allows four different I2C addresses using one address pin ADDR. 
        // Connect ADDR pin to GND for 0x48(1001000) , to VCC for 0x49. to SDA for 0x4A, and to SCL for 0x4B.

        let mut adc_a = Ads1x1x::new_ads1115(manager1.acquire_i2c(),  TargetAddr::Gnd);
        let mut adc_b = Ads1x1x::new_ads1115(manager1.acquire_i2c(),  TargetAddr::Vdd);
        let mut adc_c = Ads1x1x::new_ads1115(manager1.acquire_i2c(),  TargetAddr::Sda);
        let mut adc_d = Ads1x1x::new_ads1115(manager1.acquire_i2c(),  TargetAddr::Scl);

       // let mut adc_a = Ads1x1x::new_ads1115(adc_a_rcd, TargetAddr::Gnd);
       // let mut adc_b = Ads1x1x::new_ads1115(adc_b_rcd, TargetAddr::Vdd);
       // let mut adc_c = Ads1x1x::new_ads1115(adc_c_rcd, TargetAddr::Sda);
       // let mut adc_d = Ads1x1x::new_ads1115(adc_d_rcd, TargetAddr::Scl);

        //hprintln!("adc initialized.").unwrap();
        show_message("adc initialized.", &mut display);

        // set FullScaleRange to measure expected max voltage.
        // This is very small if measuring diff across low value shunt resistors for current
        //   but would be up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        match adc_a.set_full_scale_range(FullScaleRange::Within4_096V) {  
            Ok(())  =>  (),
            Err(e)  =>  {show_message("range error.", &mut display);
                         Mono.delay_ms(2000u32);    
                         //hprintln!("Error {:?} in adc_a.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        match adc_b.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())  =>  (),
            Err(e)  =>  {show_message("range error.", &mut display);
                         Mono.delay_ms(2000u32);    
                         //hprintln!("Error {:?} in adc_b.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        match adc_c.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())  =>  (),
            Err(e)  =>  {show_message("range error.", &mut display);
                         Mono.delay_ms(2000u32);    
                         //hprintln!("Error {:?} in adc_c.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        match adc_d.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())  =>  (),
            Err(e)  =>  {show_message("range error.", &mut display);
                         Mono.delay_ms(2000u32);    
                         //hprintln!("Error {:?} in adc_d.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        read_and_display::spawn().unwrap();

        //hprintln!("start, interval {}s", READ_INTERVAL).unwrap();

        (Shared {led}, Local {adc_a, adc_b, adc_c, adc_d, display})
    }

    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {
       adc_a:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       adc_b:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       adc_c:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       adc_d:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,

       //adc_a:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       //adc_b:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       //adc_c:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       //adc_d:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,

       // next I2CInterface is type of I2CDisplayInterface may not be the same as  above I2cInterface !!! ???
       //display: Ssd1306<I2CInterface<RefCellDevice<'static, I2c1Type>>,  ssd1306::prelude::DisplaySize128x64, 
       //                   BufferedGraphicsMode<DisplaySize128x64>>,
       //display: Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>,  ssd1306::prelude::DisplaySize128x64, 
       //                   BufferedGraphicsMode<DisplaySize128x64>>,
       display:  DisplayType,
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        //hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). Sleep may affect debugging.
           rtic::export::wfi()
        }
    }

    #[task(shared = [led], local = [adc_a, adc_b, adc_c, adc_d, display], priority=1 )]
    async fn read_and_display(mut cx: read_and_display::Context) {
       //hprintln!("read_and_display started").unwrap();
       loop {
          Mono::delay(READ_INTERVAL.secs()).await;
          //hprintln!("read_and_display").unwrap();
          blink::spawn(BLINK_DURATION).ok();

          cx.local.adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          cx.local.adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          cx.local.adc_c.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          cx.local.adc_d.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          
          // note the range can be switched if needed, eg.
          //cx.local.adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

          let values_a = [
              block!(cx.local.adc_a.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_a.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_a.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_a.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          let values_b = [
              block!(cx.local.adc_b.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_b.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_b.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_b.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          let values_c = [
              block!(cx.local.adc_c.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_c.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_c.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_c.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          let values_d = [
              block!(cx.local.adc_d.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_d.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_d.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(cx.local.adc_d.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          // SEE temperature-display_4jst REGARDING CALCUATION HERE
          
          show_display(values_a, values_b, values_c, values_d, &mut cx.local.display);
       }
    }

    #[task(shared = [led], priority=1 )]
    async fn blink(_cx: blink::Context, duration: u32) {
        crate::app::led_on::spawn().unwrap();
        Mono::delay(duration.millis()).await;
        crate::app::led_off::spawn().unwrap();
    }

    #[task(shared = [led], priority=1 )]
    async fn led_on(mut cx: led_on::Context) {
        cx.shared.led.lock(|led| led.on());
    }

    #[task(shared = [led], priority=1 )]
    async fn led_off(mut cx: led_off::Context) {
        cx.shared.led.lock(|led| led.off());
    }
}
