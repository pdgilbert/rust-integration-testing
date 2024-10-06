//! Measure temperature on sixteen 10k thermistor sensors (NTC 3950 10k thermistors probes) 
//! using four 4-channel adc's sharing I2C1 and crate ads1x1x. Display using SSD1306 on I2C2.
//! See additional documentation in temperature-display_4jst.
//! Eventually, transmit with LoRa.

//! Compare 
//!    temperature-display_4jst which uses rtic but reads only 4 ADCs and does not need share the bus. (It has more documentation.)
//!    temperature-display which uses rtic and shares the bus, but does not work yet.
//!    battery_monitor_ads1015_rtic which is currently not sharing
//!    therm10k_display
//!    htu2xd_rtic which has sensor on shared bus and 'static,  but the htu2xd crate is not using e-h 1.0.
//!    ccs811-co2-voc  uses  RefCell  for sensor and rtic  AND SEE NOTES  Jan 2024,  but the ccs811 crate is not using e-h 1.0.
//!    temp-humidity-display


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

use cortex_m_rt::entry;

    //use ads1x1x::{Ads1x1x, ic::Ads1115, ic::Resolution16Bit, channel, FullScaleRange, TargetAddr};
    use ads1x1x::{Ads1x1x, channel, FullScaleRange, TargetAddr};

    //use cortex_m_semihosting::{debug, hprintln};
    use cortex_m_semihosting::{hprintln};
    //use cortex_m::asm;

    use core::fmt::Write;


    /////////////////////   ssd
    // See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
    // regarding font sizes

    use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

    use embedded_graphics::{
        mono_font::{iso_8859_1::FONT_8X13 as FONT, MonoTextStyleBuilder}, 
        pixelcolor::BinaryColor,
        prelude::*,
        text::{Baseline, Text},
    };

    const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x64 = DisplaySize128x64;

    //const DISPLAY_LINES: usize = 3; 
    //const VPIX:i32 = 12; // vertical pixels for a line, including space


    /////////////////////  


    //use rust_integration_testing_of_examples::setup::{Peripherals, MONOCLOCK, LED, LedType, I2c1Type, I2c2Type};
    use rust_integration_testing_of_examples::setup::{Peripherals, LED};
    use rust_integration_testing_of_examples::setup;

    use embedded_hal::delay::DelayNs;

    use nb::block;

    /////////////////////  bus sharing

    use core::cell::RefCell;
    use embedded_hal_bus::i2c;  // has RefCellDevice;


    ///////////////////// 

    const READ_INTERVAL:  u32 = 5000;  // used as milliseconds
    const BLINK_DURATION: u32 = 20;    // used as milliseconds

    
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

//   //////////////////////////////////////////////////////////////////

    fn show_display<S>(
        t: [i64; 16],
        disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
    ) -> ()
    where
        S: DisplaySize,
    {    
       let mut line: heapless::String<128> = heapless::String::new(); // \n to separate lines
           
       // Consider handling error in next. If line is too short then attempt to write it crashes
       write!(line,   "1:{:3}.{:1}  {:3}.{:1}°C",  t[0]/10,t[0].abs() %10,   t[1]/10,t[1].abs() %10).unwrap();
       write!(line, "\n3:{:3}.{:1}  {:3}.{:1}°C",  t[2]/10,t[2].abs() %10,   t[3]/10,t[3].abs() %10,).unwrap();
       write!(line, "\n5:{:3}.{:1}  {:3}.{:1}°C",  t[4]/10,t[4].abs() %10,   t[5]/10,t[5].abs() %10,).unwrap();
       write!(line, "\n7:{:3}.{:1}  {:3}.{:1}°C",  t[6]/10,t[6].abs() %10,   t[7]/10,t[7].abs() %10,).unwrap();
       write!(line, "\n9:{:3}.{:1}  {:3}.{:1}°C",  t[8]/10,t[8].abs() %10,   t[9]/10,t[9].abs() %10,).unwrap();
 //      delay.delay_ms(3000u32);
       line.clear();
       write!(line, "\n7:{:3}.{:1}  {:3}.{:1}°C",  t[6]/10,t[6].abs() %10,   t[7]/10,t[7].abs() %10,).unwrap();
       write!(line, "\n9:{:3}.{:1}  {:3}.{:1}°C",  t[8]/10,t[8].abs() %10,   t[9]/10,t[9].abs() %10,).unwrap();
       write!(line, "\n11:{:3}.{:1} {:3}.{:1}°C",  t[10]/10,t[10].abs() %10, t[11]/10,t[11].abs() %10,).unwrap();
       write!(line, "\n13:{:3}.{:1} {:3}.{:1}°C",  t[12]/10,t[12].abs() %10, t[13]/10,t[13].abs() %10,).unwrap();
       write!(line, "\n15:{:3}.{:1} {:3}.{:1}°C",  t[14]/10,t[14].abs() %10, t[15]/10,t[15].abs() %10,).unwrap();

   // CHECK SIGN IS CORRECT FOR -0.3 C
   // NEED TO DISPLAY THE REST
      // hprintln!(" t {:?} = 10 * degrees", t).unwrap();

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

//   //////////////////////////////////////////////////////////////////

#[entry]
fn main() -> ! {

        //hprintln!("temperature-display_no-rtic example").unwrap();

        let dp = Peripherals::take().unwrap();
        let (i2c1, i2c2, mut led, mut delay) = setup::i2c1_i2c2_led_delay_from_dp(dp);
        //hprintln!("setup done.").unwrap();

        led.on(); 
        delay.delay_ms(1000u32);
        led.off();

        let interface = I2CDisplayInterface::new(i2c2); //default address 0x3C
        let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();

        display.init().unwrap();

        show_message(" temp.._no-rtic", &mut display);
        delay.delay_ms(2000u32);
        //hprintln!("display initialized.").unwrap();

         
        // ADS11x5 chips allows four different I2C addresses using one address pin ADDR. 
        // Connect ADDR pin to GND for 0x48(1001000) , to VCC for 0x49. to SDA for 0x4A, and to SCL for 0x4B.

        let i2c_ref_cell = RefCell::new(i2c1);

        let mut adc_a = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Gnd);
        let mut adc_b = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Vdd);
        let mut adc_c = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Sda);
        let mut adc_d = Ads1x1x::new_ads1115(i2c::RefCellDevice::new(&i2c_ref_cell),  TargetAddr::Scl);

        //hprintln!("adc initialized.").unwrap();
        show_message("adc initialized.", &mut display);

        // set FullScaleRange to measure expected max voltage.
        // This is very small if measuring diff across low value shunt resistors for current
        //   but would be up to 5v when measuring usb power.
        // +- 6.144v , 4.096v, 2.048v, 1.024v, 0.512v, 0.256v

        // wiring errors such as I2C1 on PB8-9 vs I2C2 on PB10-3 show up here as Err(I2C(ARBITRATION)) in Result
        match adc_a.set_full_scale_range(FullScaleRange::Within4_096V) {  
            Ok(())  =>  (),
            Err(_e)  =>  {show_message("range error.", &mut display);
                         delay.delay_ms(2000u32);
                         //hprintln!("Error {:?} in adc_a.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        match adc_b.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())  =>  (),
            Err(_e)  =>  {show_message("range error.", &mut display);
                         delay.delay_ms(2000u32);
                         //hprintln!("Error {:?} in adc_b.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        match adc_c.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())  =>  (),
            Err(_e)  =>  {show_message("range error.", &mut display);
                         delay.delay_ms(2000u32);
                         //hprintln!("Error {:?} in adc_c.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };

        match adc_d.set_full_scale_range(FullScaleRange::Within4_096V) {
            Ok(())  =>  (),
            Err(_e)  =>  {show_message("range error.", &mut display);
                         delay.delay_ms(2000u32);
                         //hprintln!("Error {:?} in adc_d.set_full_scale_range(). Check i2c is on proper pins.", e).unwrap(); 
                         //panic!("panic")
                        },
        };


       //COMPARE temperature-display_4jst REGARDING CALCULATION HERE
       //  REALLY DO BETTER APROX.
       // very crude linear aproximation mv to degrees C using 
       // based on 
       // t = a + v/b , v in mV, b inverse slope
       let a = 72i64;    //  72 deg
       let b = -34i64;   //  -34 mv/degree   
       // hprintln!("a {:?}  b {:?}   SCALE {:?}", a,b, SCALE).unwrap();

       loop {
          delay.delay_ms(READ_INTERVAL);
          //hprintln!("read_and_display").unwrap();
 
          // blink
          led.on(); 
          delay.delay_ms(BLINK_DURATION);
          led.off();
          
          // note the range can be switched if needed, (and switched back) eg.
          //adc_a.set_full_scale_range(FullScaleRange::Within0_256V).unwrap();

          let values_a = [
              block!(adc_a.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_a.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_a.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_a.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          let values_b = [
              block!(adc_b.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_b.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_b.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_b.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          let values_c = [
              block!(adc_c.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_c.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_c.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_c.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          let values_d = [
              block!(adc_d.read(channel::SingleA0)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_d.read(channel::SingleA1)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_d.read(channel::SingleA2)).unwrap_or(-40) as i64 / SCALE,
              block!(adc_d.read(channel::SingleA3)).unwrap_or(-40) as i64 / SCALE,
          ];

          // this is ugly
          let mut mv:[i64; 16]= [-100; 16];
          for i in 0..4  {mv[   i] = values_a[i]};
          for i in 0..4  {mv[ 4+i] = values_b[i]};
          for i in 0..4  {mv[ 8+i] = values_c[i]};
          for i in 0..4  {mv[12+i]=  values_d[i]};

          //If the mv value is over 3000 (temperature < about -19.0 C ) then the thermistor is probably missing.
          
          //hprintln!(" mv{:?} = values_a mv ", values_a).unwrap();
          hprintln!(" mv{:?} =      mv     ", mv).unwrap();

          //show_display(mv, &mut display);

 //         for i in 0..mv.len() { mv[i] = v[i] as i64 / SCALE};  
 //         //hprintln!(" mv {:?}", mv).unwrap();

          // t in tenths of a degrees C, so it is an int  but t[0]/10, t[0].abs() %10 give a degree with one decimal.
          let mut t:[i64; 16] = [-100; 16] ;

          //for i in 0..t.len() { t[i] = 10 * (a + mv[i] / b) }; // loses the decimal rounding division
          for i in 0..t.len() { t[i] =  10 * a  + (10 * mv[i]) / b };
 
          hprintln!(" t {:?} = 10 * degrees", t).unwrap();

          show_display(t, &mut display);

       }
}
