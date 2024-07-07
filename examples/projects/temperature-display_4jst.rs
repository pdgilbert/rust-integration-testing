//! Measure temperature on four 10k thermistor sensors (NTC 3950 10k thermistors probes) 
//! using a 4-channel adc on I2C1 and crate ads1x1x. Display using SSD1306 on I2C2.
//! (The (scl, sda) pair on blackpill is (b8, b9) for I2C1 and (b10, b3) for I2C2,
//! This does not require bus sharing.  Compare example temperature-display which uses
//! four 4-channel adc's to read 16 sensor, but requires bus sharing.
//! Compare also examples/rtic/battery_monitor_Ads1015_rtic.rs, examples/misc/therm10k_display.rs,
//! and examples/projects/temp-humidity-display.rs.

//! One side of the thermistor is connected to GND and other side to adc pin and also through
//! a 10k sresistor to VCC. That makes the max voltage about VCC/2, so about 2.5v when VCC is 5v.
//! and 1.6v when vcc is 3.2v. This is convenient for adc pins that are not 5v tolerant.
//! This means the voltage varies inversely compared to connecting throught the resistor to GND 
//! as is sometimes done. (Since NTC resistance goes down as temperature goes up, this means
//! higher temperature gives higher voltage measurement.) 
//! It also has the advantage that unused adc inputs (resistor in place but no NTC) are pulled high
//! rather than to ground. This is preferred to reduce power leakage (datasheet section 10.1.4).
//! Regarding crate ads1x1x see
//! -   https://github.com/eldruin/ads1x1x-rs?tab=readme-ov-file
//! -   https://blog.eldruin.com/ads1x1x-analog-to-digital-converter-driver-in-rust/
//! Regarding NTC thermistors see,  for example,
//! -   https://eepower.com/resistor-guide/resistor-types/ntc-thermistor/#
//! -   https://www.ametherm.com/thermistor/ntc-thermistor-beta
//! -   https://www.electronics-tutorials.ws/io/thermistors.html
//! -   https://www.mathscinotes.com/2014/05/yet-another-thermistor-discussion/
//! -   https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html
//! -   https://learn.adafruit.com/thermistor/using-a-thermistor
//! Regarding  ADS1x1x Datasheets:
//! - [ADS101x](http://www.ti.com/lit/ds/symlink/ads1015.pdf)
//! - [ADS111x](http://www.ti.com/lit/ds/symlink/ads1115.pdf)
//! The output value will be within `[2047..-2048]` for 12-bit devices (`ADS101x`)
//! and within `[32767..-32768]` for 16-bit devices (`ADS111x`).
//! The full range is only used for differential measurement between two pins. These can be
//! positive or negative.  Direct (single) measurements are always positive (the devices do not
//! allow inputs lower than GND) so only half the range is used.
//! The voltage that values correspond to depend on the full-scale range setting.

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
    
    use ads1x1x::{Ads1x1x, ic::Ads1115, ic::Resolution16Bit, channel, FullScaleRange, SlaveAddr};

    //use cortex_m_semihosting::{debug, hprintln};
    //use cortex_m_semihosting::{hprintln};
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

    //const DISPLAY_LINES: usize = 3; 
    //const VPIX:i32 = 12; // vertical pixels for a line, including space

    const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;

    //I2CInterface is from ssd1306::prelude 
    type  DisplayType =  Ssd1306<I2CInterface<I2c2Type>,                            
                          ssd1306::prelude::DisplaySize128x32, BufferedGraphicsMode<DisplaySize128x32>>;



    ///////////////////// 

    const READ_INTERVAL:  u32 =  3;  // used as seconds
    const BLINK_DURATION: u32 = 20;  // used as milliseconds
    
    // The units of measurement are  [32767..-32768] for 16-bit devices (and  [2047..-2048] for 12-bit devices)
    // but only half the range is used for single-ended measurements. (Percision could be improved by using
    // one input set at GND as one side of a differential measurement.)
    // set FullScaleRange to measure expected max voltage.
    // SCALE multiplies
    const  FS: i64 = 4096;   // max 4.096v should correspond to setting  adc.set_full_scale_range(FullScaleRange::Within4_096V)
    // calibrated to get mV    depends on FullScaleRange
    //  SCALE is measured units per mV, so the  measurement is divided by SCALE to get mV
    const  SCALE: i64 = 32767 / FS ; 
        // This is very small if measuring diff across low value shunt resistors for current
        //   but would be up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

    use rust_integration_testing_of_examples::setup::{MONOCLOCK, LED, LedType, I2c1Type, I2c2Type};
    use rust_integration_testing_of_examples::setup;

    use embedded_hal::delay::DelayNs;

    use nb::block;


    fn show_display<S>(
        v: [i64; 4],
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {    
       let mut line: heapless::String<64> = heapless::String::new(); // \n to separate lines

       let j0: u8 = 0;   //set to  0 / 4 / 8 / 12 depending on adc

       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line, "J{:1}{:3}.{:1} J{:1}{:3}.{:1}\nJ{:1}{:3}.{:1} J{:1}{:3} °C", 
           j0+1, v[0]/10,v[0]%10,  j0+2, v[1]/10,v[1]%10,    j0+3, v[2]/10,v[2]%10, j0+4,  v[3]/10,).unwrap();  //v[3]%10,

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

        let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C
        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        show_message("temp-humidity", &mut display);
        Mono.delay_ms(2000u32);    
        //hprintln!("display initialized.").unwrap();

         
        // ADS11x5 chips allows four different I2C addresses using one address pin ADDR. 
        // Connect ADDR pin to GND for 0x48(1001000) , to VCC for 0x49. to SDA for 0x4A, and to SCL for 0x4B.

        //  also set j0 to  0 / 4 / 8 / 12 in show_display()
        let mut adc = Ads1x1x::new_ads1115(i2c1,  SlaveAddr::Gnd);
        //let mut adc = Ads1x1x::new_ads1115(i2c1,  SlaveAddr::Vdd);
        //let mut adc = Ads1x1x::new_ads1115(i2c1,  SlaveAddr::Sda);
        //let mut adc = Ads1x1x::new_ads1115(i2c1,  SlaveAddr::Scl);

        //hprintln!("adc initialized.").unwrap();
        show_message("adc initialized.", &mut display);

        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        //asm::bkpt();

        //let z = adc.set_full_scale_range(FullScaleRange::Within4_096V);
       // match z {  
       //     Ok(())   =>  (),
       //     Err(e) =>  {hprintln!("Error {:?} in adc.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
       //                 //panic!("panic")
       //                },
       // };
//        hprintln!("set_full_scale_range skipped.").unwrap();

        read_and_display::spawn().unwrap();

//        hprintln!("start, interval {}s", READ_INTERVAL).unwrap();

        (Shared {led}, Local {adc, display})
    }

    #[shared]
    struct Shared {
        led: LedType,
    }

    #[local]
    struct Local {
       adc:   Ads1x1x<I2c1Type, Ads1115, Resolution16Bit, ads1x1x::mode::OneShot>,
       display:  DisplayType,
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
       //hprintln!("read_and_display started").unwrap();
       loop {
          Mono::delay(READ_INTERVAL.secs()).await;
          //hprintln!("read_and_display").unwrap();
          blink::spawn(BLINK_DURATION).ok();

          //  NEED TO HANDLE ERROR
          //cx.local.adc.set_full_scale_range(FullScaleRange::Within4_096V).unwrap(); 
          //hprintln!("done set_full_scale_range").unwrap();
          
          // note the range can be switched if needed, eg.
          //cx.local.adc.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

          let v = [
              block!(cx.local.adc.read(channel::SingleA0)).unwrap_or(-40),
              block!(cx.local.adc.read(channel::SingleA1)).unwrap_or(-40),
              block!(cx.local.adc.read(channel::SingleA2)).unwrap_or(-40),
              block!(cx.local.adc.read(channel::SingleA3)).unwrap_or(-40),
          ];
           
          //hprintln!("val {:?}", v).unwrap();
 
          let mut mv:[i64; 4] = [-100; 4] ;
          for i in 0..mv.len() { mv[i] = v[i] as i64 / SCALE  }; 
          //hprintln!(" mv {:?}", mv).unwrap();

          // very crude linear aproximation mv to degrees C
          // t in tenths of a degree C, so it is an int  but t[0]/10, t[0]%10 give a degree with one decimal.
          // t = a + v/b , v in mV, b inverse slope, a includes degrees K to C
          let a = 968i64;  //  tenth degree
          let b = -207i64;     //  tenth degrees / V. 
          // hprintln!("a {:?}  b {:?}   SCALE {:?}", a,b, SCALE).unwrap();

          
          let mut t:[i64; 4] = [-100; 4] ;
          for i in 0..t.len() { t[i] = a + (mv[i] as i64 * b)/1000  };  //  REALLY DO BETTER APROX.
 
          //hprintln!(" t {:?} 10 * degrees", t).unwrap();

          show_display(t, &mut cx.local.display);
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
