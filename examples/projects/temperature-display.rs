//! TARGET Measure temperature with multiple 10k thermistor sensors (NTC 3950 10k thermistors probes) 
//! using multiple channel adc and crate ads1x1x. Display using SSD1306.

//!  i2c pin numbers for the MCU are set in src/i2c.rs  by links below through 
//!     use rust_integration_testing_of_examples::dht_i2c_led_usart_delay

//!  WORK IN PROGRESS. MOST CODE STILL FROM ANOTHER EXAMPLE.
//! https://www.ametherm.com/thermistor/ntc-thermistor-beta

//! Compare examples/misc/battery_monitor_ads1015_rtic.rs, examples/misc/therm10k_display.rs,
//! and examples/projects/temp-humidity-display.rs.

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
    use ads1x1x::{Ads1x1x, DynamicOneShot, FullScaleRange, SlaveAddr, 
                  ChannelSelection,
                  ic::{Ads1015, Resolution12Bit},
                  interface::I2cInterface};
    

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use rtt_target::{rprintln, rtt_init_print};

    use core::fmt::Write;

    use rtic;
    use rtic_monotonics::systick::Systick;
    use rtic_monotonics::systick::fugit::{ExtU32};

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

    use rust_integration_testing_of_examples::dht_i2c_led_usart_delay::{
        setup_dht_i2c_led_usart_delay_using_dp, I2cType, LED, LedType, DelayNs, MONOCLOCK};

    use shared_bus::{I2cProxy};
    use core::cell::RefCell;
    use cortex_m::interrupt::Mutex;

    use nb::block;

    // SCALE multiplies
    const  SCALE: i16 = 2; // calibrated to get mV    depends on FullScaleRange


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
       let mut line: heapless::String<64> = heapless::String::new(); // \n to separate lines
           
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

        //rtt_init_print!();
        //rprintln!("battery_monitor_ads1015_rtic example");
        //hprintln!("temperature-display example").unwrap();

        let (_dht, i2c, mut led, _usart, mut delay) = setup_dht_i2c_led_usart_delay_using_dp(cx.device);

        led.on(); 
        delay.delay_ms(1000u32);
        led.off();

        let manager: &'static _ = shared_bus::new_cortexm!(I2cType = i2c).unwrap();

        let interface = I2CDisplayInterface::new(manager.acquire_i2c());

        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        show_message("temp-humidity", &mut display);
        delay.delay_ms(2000u32);    

         
        // ADS11x5 chips allows four different I2C addresses using one address pin ADDR. 
        // Connect ADDR pin to GND for 0x48(1001000) , to VCC for 0x49. to SDA for 0x4A, and to SCL for 0x4B.

        let mut adc_a = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(false, false)); //addr = GND
        let mut adc_b = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(false, true )); //addr =  V
        let mut adc_c = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(true,  false)); //addr =  SDA
        let mut adc_d = Ads1x1x::new_ads1015(manager.acquire_i2c(), SlaveAddr::Alternative(true,  true )); //addr =  SCL

        // set FullScaleRange to measure expected max voltage.
        // This is very small if measuring diff across low value shunt resistors for current
        //   but would be up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        match adc_a.set_full_scale_range(FullScaleRange::Within4_096V) {  
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_a.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

        match adc_b.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_b.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

        match adc_c.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_c.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
                       },
        };

        match adc_d.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())   =>  (),
            Err(e) =>  {hprintln!("Error {:?} in adc_d.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                        panic!("panic")
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
       adc_a:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       adc_b:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       adc_c:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       adc_d:   Ads1x1x<I2cInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       display: Ssd1306<I2CInterface<I2cProxy<'static, Mutex<RefCell<I2cType>>>>, 
                          ssd1306::prelude::DisplaySize128x64, 
                          BufferedGraphicsMode<DisplaySize128x64>>,
    }

    #[idle(local = [])]
    fn idle(_cx: idle::Context) -> ! {
        //hprintln!("idle with wfi started").unwrap();
        loop { // Wait For Interrupt allows sleep (vs default nop which does not). Sleep may affect debugging.
           rtic::export::wfi()
        }
    }

    #[task(shared = [led], local = [adc_a, adc_b, adc_c, adc_d, display], priority=1 )]
    async fn read_and_display(mut cx: read_and_display::Context) {
       loop {
          Systick::delay(READ_INTERVAL.secs()).await;
          //hprintln!("read_and_display").unwrap();
          blink::spawn(BLINK_DURATION).ok();

          cx.local.adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          cx.local.adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          cx.local.adc_c.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          cx.local.adc_d.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          
          // note the range can be switched if needed, eg.
          //cx.local.adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

          let values_a = [
              block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::SingleA0)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::SingleA1)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::SingleA2)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_a, ChannelSelection::SingleA3)).unwrap_or(8091) * SCALE,
          ];

          let values_b = [
              block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA0)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA1)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA2)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_b, ChannelSelection::SingleA3)).unwrap_or(8091) * SCALE,
          ];

          let values_c = [
              block!(DynamicOneShot::read(cx.local.adc_c, ChannelSelection::SingleA0)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_c, ChannelSelection::SingleA1)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_c, ChannelSelection::SingleA2)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_c, ChannelSelection::SingleA3)).unwrap_or(8091) * SCALE,
          ];

          let values_d = [
              block!(DynamicOneShot::read(cx.local.adc_d, ChannelSelection::SingleA0)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_d, ChannelSelection::SingleA1)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_d, ChannelSelection::SingleA2)).unwrap_or(8091) * SCALE,
              block!(DynamicOneShot::read(cx.local.adc_d, ChannelSelection::SingleA3)).unwrap_or(8091) * SCALE,
          ];

          show_display(values_a, values_b, values_c, values_d, &mut cx.local.display);
           
          //hprintln!(" values_a {:?}  values_b {:?}", values_a, values_b).unwrap();
          //hprintln!(" values_c {:?}  values_d {:?}", values_a, values_b).unwrap();
       }
    }

    #[task(shared = [led], priority=1 )]
    async fn blink(_cx: blink::Context, duration: u32) {
        crate::app::led_on::spawn().unwrap();
        Systick::delay(duration.millis()).await;
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
