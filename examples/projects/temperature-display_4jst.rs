//! Measure temperature on four 10k thermistor sensors (NTC 3950 10k thermistors probes) 
//! using a 4-channel adc on I2C1 and crate ads1x1x. Display using SSD1306 on I2C2.
//! This does not require bus sharing.  Compare example temperature-display which uses
//! four 4-channel adc's to read 16 sensor, but requires bus sharing.
//! Compare also examples/rtic/battery_monitor_ads1015_rtic.rs, examples/misc/therm10k_display.rs,
//! and examples/projects/temp-humidity-display.rs.

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
//    use ads1x1x::{Ads1x1x, DynamicOneShot, FullScaleRange, SlaveAddr, 
//                  ChannelSelection,
//                  ic::{Ads1015, Resolution12Bit},
//                  interface::I2cInterface};
    
    use ads1x1x::{Ads1x1x, ic::Ads1015, ic::Resolution12Bit, channel,  
                  FullScaleRange, SlaveAddr};


    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

    use core::fmt::Write;

    use rtic;
    //use rtic_monotonics::systick::Systick;
    //use rtic_monotonics::systick::fugit::{ExtU32};
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

    // secs() and millis() methods from https://docs.rs/fugit/latest/fugit/trait.ExtU32.html#tymethod.secs


    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // DisplaySize128x32:
    //    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
    //    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
    //    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
    //    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
    //    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
    //    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high
    
    use embedded_graphics::{
        //mono_font::{ascii::FONT_10X20 as FONT, MonoTextStyleBuilder, MonoTextStyle}, 
        mono_font::{iso_8859_1::FONT_10X20 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    const READ_INTERVAL:  u32 =  2;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds

    use rust_integration_testing_of_examples::setup::{MONOCLOCK, LED, LedType, I2c1Type, I2c2Type};
    use rust_integration_testing_of_examples::setup;

    use embedded_hal::delay::DelayNs;


    //use shared_bus::{I2cProxy};
    //use core::cell::RefCell;
    //use embedded_hal_bus::i2c::RefCellDevice;
    //use shared_bus::{I2cProxy};
    //use cortex_m::interrupt::Mutex;

    use nb::block;

    // SCALE multiplies
    const  SCALE: i16 = 2; // calibrated to get mV    depends on FullScaleRange


    fn show_display<S>(
        v: [i16; 4],
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {    
       let mut line: heapless::String<64> = heapless::String::new(); // \n to separate lines
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line, "A1-4 {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C  {:3}.{:1}°C ", 
            v[0]/10,v[0]%10,  v[1]/10,v[1]%10,  v[2]/10,v[2]%10,  v[2]/10,v[2]%10,).unwrap();

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
       Text::with_baseline( &text, Point::new(0, 0), text_style, Baseline::Top)
               .draw(&mut *disp)
               .unwrap();

       disp.flush().unwrap();
       ()
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        //let mono_token = rtic_monotonics::create_systick_token!();
        //Systick::start(cx.core.SYST, MONOCLOCK, mono_token);
        Mono::start(cx.core.SYST, MONOCLOCK);

        //rtt_init_print!();
        //rprintln!("battery_monitor_ads1015_rtic example");
        //hprintln!("temperature-display example").unwrap();

        let (i2c1, i2c2, mut led, _delay) = setup::i2c1_i2c2_led_delay_from_dp(cx.device);

        led.on(); 
        Mono.delay_ms(1000u32);
        led.off();

        // As of Feb 2024 I2CDisplayInterface::new is not working with shared bus.
        // (No luck Using embedded-bus instead.
        // Try ssd on i2c2 and shared-bus ads's on i2c1
        //let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c2).unwrap();
        //let manager1: &'static _ = shared_bus::new_cortexm!(I2c1Type = i2c1).unwrap();


       //let i2c1_ref_cell = RefCell::new(i2c1);
       //let adc_a_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let adc_b_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let adc_c_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let adc_d_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let ina_rcd = RefCellDevice::new(&i2c1_ref_cell); 
       //let ssd_rcd   = RefCellDevice::new(&i2c1_ref_cell); 
       //let interface = I2CDisplayInterface::new(ssd_rcd);
        let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C
        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        show_message("temp-humidity", &mut display);
        Mono.delay_ms(2000u32);    

         
        // ADS11x5 chips allows four different I2C addresses using one address pin ADDR. 
        // Connect ADDR pin to GND for 0x48(1001000) , to VCC for 0x49. to SDA for 0x4A, and to SCL for 0x4B.

        let mut adc = Ads1x1x::new_ads1015(i2c1,  SlaveAddr::Gnd);
        //let mut adc = Ads1x1x::new_ads1015(i2c1,  SlaveAddr::Vdd);
        //let mut adc = Ads1x1x::new_ads1015(i2c1,  SlaveAddr::Sda);
        //let mut adc = Ads1x1x::new_ads1015(i2c1,  SlaveAddr::Scl);

       // let mut adc = Ads1x1x::new_ads1015(adc_a_rcd, SlaveAddr::Gnd);
       // let mut adc = Ads1x1x::new_ads1015(adc_b_rcd, SlaveAddr::Vdd);
       // let mut adc = Ads1x1x::new_ads1015(adc_c_rcd, SlaveAddr::Sda);
       // let mut adc = Ads1x1x::new_ads1015(adc_d_rcd, SlaveAddr::Scl);

        // set FullScaleRange to measure expected max voltage.
        // This is very small if measuring diff across low value shunt resistors for current
        //   but would be up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        match adc.set_full_scale_range(FullScaleRange::Within4_096V) {  
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

        read_and_display::spawn().unwrap();

        //hprintln!("start, interval {}s", READ_INTERVAL).unwrap();

        (Shared {led}, Local {adc, display})
    }

    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {
       adc:   Ads1x1x<I2c1Type, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       //adc:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       //adc:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2c1Type>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       // next I2CInterface is type of I2CDisplayInterface may not be the same as  above I2cInterface !!! ???
       //display: Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2c2Type>>>>, 
       //                   ssd1306::prelude::DisplaySize128x64, 
       //                   BufferedGraphicsMode<DisplaySize128x64>>,
       display:  Ssd1306<I2CInterface<I2c2Type>,       //I2CInterface is from ssd1306::prelude                      
                          ssd1306::prelude::DisplaySize128x64, 
                          BufferedGraphicsMode<DisplaySize128x64>>,

       //adc_a:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       //adc_b:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       //adc_c:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       //adc_d:   Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       //display: Ssd1306<I2CInterface<RefCellDevice<'static, I2c1Type>>, 
       //                   ssd1306::prelude::DisplaySize128x64, 
       //                   BufferedGraphicsMode<DisplaySize128x64>>,
    }

    #[idle()]
    fn idle(_cx: idle::Context) -> ! {
        //hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). Sleep may affect debugging.
           rtic::export::wfi()
        }
    }

    #[task(shared = [led], local = [adc, display], priority=1 )]
    async fn read_and_display(mut cx: read_and_display::Context) {
       loop {
          Mono::delay(READ_INTERVAL.secs()).await;
          //hprintln!("read_and_display").unwrap();
          blink::spawn(BLINK_DURATION).ok();

          cx.local.adc.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          
          // note the range can be switched if needed, eg.
          //cx.local.adc.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

          let values = [
              block!(cx.local.adc.read(channel::SingleA0)).unwrap_or(8091) * SCALE,
              block!(cx.local.adc.read(channel::SingleA1)).unwrap_or(8091) * SCALE,
              block!(cx.local.adc.read(channel::SingleA2)).unwrap_or(8091) * SCALE,
              block!(cx.local.adc.read(channel::SingleA3)).unwrap_or(8091) * SCALE,
          ];

          show_display(values, &mut cx.local.display);
           
          //hprintln!(" values_a {:?}  values_b {:?}", values_a, values_b).unwrap();
          //hprintln!(" values_c {:?}  values_d {:?}", values_a, values_b).unwrap();
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
