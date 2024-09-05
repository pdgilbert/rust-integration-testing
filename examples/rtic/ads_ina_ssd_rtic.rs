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

    use rtic;
    use crate::Mono;
    use rtic_monotonics::systick::prelude::*;

   use core::mem::MaybeUninit;
   use rtic_sync::arbiter::{i2c::ArbiterDevice, Arbiter};

    // Instantiate an Arbiter with a static lifetime.
    static ARBITER: Arbiter<u32> = Arbiter::new(32);
// THIS HAS THE SAME PROBLEM AS   https://github.com/rtic-rs/rtic/issues/886

   const MONOCLOCK: u32 = 8_000_000; 

   const READ_INTERVAL: u32 = 10;  // used as seconds
   const BLINK_DURATION: u32 = 20;  // used as milliseconds

   /////////////////////   ads
   use ads1x1x::{Ads1x1x, channel, 
                 mode::OneShot, 
                 //Ads1015, Resolution12Bit,    PRIVATE?
                 FullScaleRange, TargetAddr};
   
   /////////////////////   ina
   use ina219::{address::{Address, Pin}, 
                measurements::BusVoltage, 
                SyncIna219, 
                calibration::UnCalibrated,
   };
   
   /////////////////////   ssd
   use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};
   
   type DisplaySizeType = ssd1306::prelude::DisplaySize128x32;
   const DISPLAYSIZE: DisplaySizeType = DisplaySize128x32;
   const VPIX:i32 = 12; // vertical pixels for a line, including space
   
   use core::fmt::Write;
   use embedded_graphics::{
       mono_font::{ascii::FONT_6X10 as FONT, MonoTextStyle, MonoTextStyleBuilder},
       pixelcolor::BinaryColor,
       prelude::*,
       text::{Baseline, Text},
   };

   /////////////////////   hals
   use core::cell::RefCell;
   use embedded_hal_bus::i2c::RefCellDevice;
   
  

   #[cfg(feature = "stm32h7xx")]
   use stm32h7xx_hal::{
      delay::DelayFromCountDownTimer,
      //pwr::PwrExt,
   };
   
   use rust_integration_testing_of_examples::setup;
   use rust_integration_testing_of_examples::
                  setup::{Peripherals, I2C1, I2cType, I2c1Type, I2c2Type, RccExt, DelayNs,};

  
///////////////////////////////////////////////////////////////////////////////////////////

       #[shared]
       struct Shared {
           led: u8,  //LedType,
       }
    

       #[local]
       struct Local {
//           display:  Ssd1306<I2CInterface<RefCellDevice<'static, I2c2Type>>,   //EVERYTHING ON 2
           display:  Ssd1306<I2CInterface<ArbiterDevice<'static, I2c2Type>>,   //EVERYTHING ON 2
                             DisplaySizeType, 
                             BufferedGraphicsMode<DisplaySizeType>>,
           //text_style: TextStyle,
           text_style: MonoTextStyle<'static, BinaryColor>,

           ina:    SyncIna219<ArbiterDevice<'static, I2c2Type>, UnCalibrated>,
//           ina:    SyncIna219<RefCellDevice<'static, I2c2Type>, UnCalibrated>,
//          adc_a:  Ads1x1x<RefCellDevice<'static, I2c2Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
//          adc_b:  Ads1x1x<RefCellDevice<'static, I2c2Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
       }

////////////////////////////////////////////////////////////////////////////////

   fn show_display<S>(
       voltage: BusVoltage,
       a_mv: i16, 
       b_mv: [i16; 2], 
       text_style: MonoTextStyle<BinaryColor>,
       display: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
   ) -> ()
   where
       S: DisplaySize,
   {
       let mut lines: [heapless::String<32>; 3] = [
           heapless::String::new(),
           heapless::String::new(),
           heapless::String::new(),
       ];
   
       write!(lines[0], "ina V: {:?} mv? ", voltage).unwrap();
       write!(lines[1], "ads_a V: {} mv  ", a_mv).unwrap();
       write!(lines[2], "ads_b A0: {} mv.  ads_b A1: {} mv.", b_mv[0], b_mv[1]).unwrap();
   
       display.clear_buffer();
       for (i, line) in lines.iter().enumerate() {
           // start from 0 requires that the top is used for font baseline
           Text::with_baseline(
               line,
               Point::new(0, i as i32 * VPIX),
               text_style,
               Baseline::Top,
           )
           .draw(&mut *display)
           .unwrap();
       }
       display.flush().unwrap();
       ()
   }

////////////////////////////////////////////////////////////////////////////////

    #[init(local = [
        i2c_arbiter: MaybeUninit<Arbiter<I2c2Type>> = MaybeUninit::uninit(),
    ])]
   fn init(cx: init::Context) -> (Shared, Local ) {

       Mono::start(cx.core.SYST, MONOCLOCK);

       //let (i2cset, _delay) = setup_from_dp(cx.device);
       let (i2c1, i2c2) = setup::i2c1_i2c2_from_dp(cx.device);

       // let i2c = I2c::new(cx.device.I2C1);
       // let i2c_arbiter = cx.local.i2c_arbiter.write(Arbiter::new(i2c));
       // let ens160 = Ens160::new(ArbiterDevice::new(i2c_arbiter), 0x52);

       let i2c_arbiter = cx.local.i2c_arbiter.write(Arbiter::new(i2c2)); // with MaybeUninit above

//       let i2cset_ref_cell: &'static _ = &RefCell::new(i2c2);
//    //let manager: &'static _ = shared_bus::new_cortexm!(I2c2Type = i2c2).unwrap();
//       //let i2cset_ref_cell: RefCell<I2c1Type> = RefCell::new(i2c1);
//       let adc_a_rcd = RefCellDevice::new(&i2cset_ref_cell); 
//       let adc_b_rcd = RefCellDevice::new(&i2cset_ref_cell); 
//       let ina_rcd   = RefCellDevice::new(&i2cset_ref_cell); 
//       let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

       /////////////////////   ads
//       let mut adc_a = Ads1x1x::new_ads1015(adc_a_rcd,  TargetAddr::Gnd);
//       let mut adc_b = Ads1x1x::new_ads1015(adc_b_rcd,  TargetAddr::Vdd);
//       // set FullScaleRange to measure expected max voltage.
//       adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
//       adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

       let mut adc_a = Ads1x1x::new_ads1015(ArbiterDevice::new(i2c_arbiter),  TargetAddr::Gnd);
       let mut adc_b = Ads1x1x::new_ads1015(ArbiterDevice::new(i2c_arbiter),  TargetAddr::Vdd);
       // set FullScaleRange to measure expected max voltage.
       adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
       adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

       /////////////////////   ina
       //let ina = SyncIna219::new( ina_rcd, Address::from_pins(Pin::Gnd, Pin::Gnd)).unwrap(); 
       let ina = SyncIna219::new(ArbiterDevice::new(i2c_arbiter), Address::from_pins(Pin::Gnd, Pin::Gnd));

       /////////////////////   ssd
       let interface = I2CDisplayInterface::new(ArbiterDevice::new(i2c_arbiter)); //default address 0x3C
       //let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
       //let interface = I2CDisplayInterface::new_custom_address(ssd_rcd,   0x3D);  //alt address

       let mut display = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
           .into_buffered_graphics_mode();

       display.init().unwrap();

       let text_style = MonoTextStyleBuilder::new()
           .font(&FONT) 
           .text_color(BinaryColor::On)
           .build();

       Text::with_baseline(
           "Display initialized ...",
           Point::zero(),
           text_style,
           Baseline::Top,
       )
       .draw(&mut display)
       .unwrap();

       let led:u8 = 8;
          (Shared {led}, Local { ina, display, text_style })  //, adc_a, adc_b
       }

       /////////////////////   measure and display 
     
       #[task(local = [ ina, display, text_style])]  // adc_a, adc_b 
       async fn read_and_display(cx: read_and_display::Context) {
          
          let ina = cx.local.ina;
//          let adc_a = cx.local.adc_a;
//          let adc_b = cx.local.adc_b;
          let mut display = cx.local.display;
          let text_style = cx.local.text_style;
     
          loop {
             let voltage = ina.bus_voltage().unwrap();  
     
//             let a_mv = block!(adc_a.read(channel::SingleA0)).unwrap_or(8091);

//             let values_b = [
//                 block!(adc_b.read(channel::SingleA0)).unwrap_or(8091),
//                 block!(adc_b.read(channel::SingleA1)).unwrap_or(8091),
//             ];
     
             let a_mv: i16 = 0;                 //FAKE
             let values_b: [i16; 2] = [0, 0];   //FAKE
             show_display(voltage, a_mv, values_b, *text_style, &mut display);
             Mono::delay(READ_INTERVAL.secs()).await;
          }        
       }
}    
     

     
     
     
     
     
     
     
     

     
     
     

     
     
     
     
   

     
     
     
     
     
     
     

     
