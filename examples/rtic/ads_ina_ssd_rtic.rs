#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use rtic::app;

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
   use rtic_monotonics::systick::Systick;
   use rtic_monotonics::systick::fugit::{ExtU32};

   const MONOCLOCK: u32 = 8_000_000; 

   const READ_INTERVAL: u32 = 10;  // used as seconds
   const BLINK_DURATION: u32 = 20;  // used as milliseconds

   /////////////////////   ads
   use ads1x1x::{Ads1x1x, channel, ChannelSelection, 
                 DynamicOneShot, mode::OneShot, 
                 //Ads1015, Resolution12Bit,    PRIVATE?
                 FullScaleRange, SlaveAddr};
   
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
   
   use embedded_hal::{
      //i2c::I2c as I2cTrait,
      delay::DelayNs,
   };
   
   // "hal" is used for items that are the same in all hal crates
   use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;
   
   use hal::{
      pac::{Peripherals, I2C1}, 
      i2c::I2c as I2cType,
      //  i2c::Error as i2cError,
      rcc::{RccExt},
      prelude::*,
      //block,
   };
   
   // may NOT need this
   //#[cfg(feature = "stm32f4xx")]
   //pub use stm32f4xx_hal::{
   //   rcc::Clocks,
   //   pac::{TIM5},
   //   timer::Delay,
   //   timer::TimerExt,
   //   gpio::GpioExt,
   //};
   

   // may need this
   //#[cfg(feature = "stm32h7xx")]
   //use stm32h7xx_hal::{
   //   delay::DelayFromCountDownTimer,
   //   //pwr::PwrExt,
   //};
   
   use rust_integration_testing_of_examples::i2c::{I2c1Type, I2c2Type};
   use rust_integration_testing_of_examples::i2c1_i2c2_led_delay;

// THESE ARE NO LONGER USED
   #[cfg(feature = "stm32f4xx")]            
   pub fn setup_from_dp(dp: Peripherals) ->  ( I2cType<I2C1>, impl DelayNs) { // NEEDS u8 NOT I2C1 Why?
      let rcc = dp.RCC.constrain();
      let clocks = rcc.cfgr.freeze();
   
      let gpiob = dp.GPIOB.split();
      let scl = gpiob.pb8.into_alternate_open_drain(); 
      let sda = gpiob.pb9.into_alternate_open_drain(); 
   
      let i2c = dp.I2C1.i2c( (scl, sda), 400.kHz(), &clocks);
   
      // need  ::<1000000_u32>  for `FREQ` of the method `delay   WHY?
      let delay = dp.TIM5.delay::<1000000_u32>(&clocks);

      (i2c, delay)
   }

   #[cfg(feature = "stm32h7xx")]
   pub fn setup_from_dp(dp: Peripherals) ->  ( I2cType<I2C1>, impl DelayNs) { // NEEDS u8 NOT I2C1 Why?
   //pub fn setup_from_dp(dp: Peripherals) ->  ( impl I2cTrait<u8>, impl DelayNs) { // NEEDS u8 NOT I2C1 Why?
      let rcc = dp.RCC.constrain();
      let vos = dp.PWR.constrain().freeze();
      let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); 
      let clocks = ccdr.clocks;
   
      let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
      let scl = gpiob.pb8.into_alternate().set_open_drain();
      let sda = gpiob.pb9.into_alternate().set_open_drain();
   
      let i2c = dp.I2C1.i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &clocks);
   
      // CountDownTimer not supported by embedded-hal 1.0.0
      let timer = dp.TIM5.timer(1.Hz(), ccdr.peripheral.TIM5, &clocks);
      let delay = DelayFromCountDownTimer::new(timer);
   
      (i2c, delay)
   }
   
///////////////////////////////////////////////////////////////////////////////////////////

       #[shared]
       struct Shared {
           led: u8,  //LedType,
       }
    

       #[local]
       struct Local {
           display:  Ssd1306<I2CInterface<RefCellDevice<'static, I2c1Type>>, 
                             DisplaySizeType, 
                             BufferedGraphicsMode<DisplaySizeType>>,
           //text_style: TextStyle,
           text_style: MonoTextStyle<'static, BinaryColor>,

           ina:    SyncIna219<RefCellDevice<'static, I2c1Type>, UnCalibrated>,
//          adc_a:  Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
//          adc_b:  Ads1x1x<RefCellDevice<'static, I2c1Type>, Ads1015, Resolution12Bit, ads1x1x::mode::OneShot>,
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

   #[init]
   fn init(cx: init::Context) -> (Shared, Local ) {

       let mono_token = rtic_monotonics::create_systick_token!();
       Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

       //let (i2cset, _delay) = setup_from_dp(cx.device);
       let (i21, i2c2, _led, _delay, _clock) = i2c1_i2c2_led_delay::setup_from_dp(cx.device);

       //let i2cset_ref_cell: 'static + RefCell<I2c1Type> = RefCell::new(i21);
       let i2cset_ref_cell: RefCell<I2c1Type> = RefCell::new(i21);
       let adc_a_rcd = RefCellDevice::new(&i2cset_ref_cell); 
       let adc_b_rcd = RefCellDevice::new(&i2cset_ref_cell); 
       let ina_rcd   = RefCellDevice::new(&i2cset_ref_cell); 
       let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

       /////////////////////   ads
       let mut adc_a = Ads1x1x::new_ads1015(adc_a_rcd,  SlaveAddr::Gnd);
       let mut adc_b = Ads1x1x::new_ads1015(adc_b_rcd,  SlaveAddr::Vdd);
       // set FullScaleRange to measure expected max voltage.
       adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
       adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

       /////////////////////   ina
       let ina = SyncIna219::new( ina_rcd, Address::from_pins(Pin::Gnd, Pin::Gnd)).unwrap(); 

       /////////////////////   ssd
       let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
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
     
//             let a_mv = block!(DynamicOneShot::read(&mut adc_a, ChannelSelection::SingleA0)).unwrap_or(8091);

//             let values_b = [
//                 block!(adc_b.read(channel::SingleA0)).unwrap_or(8091),
//                 block!(adc_b.read(channel::SingleA1)).unwrap_or(8091),
//             ];
     
             let a_mv: i16 = 0;                 //FAKE
             let values_b: [i16; 2] = [0, 0];   //FAKE
             show_display(voltage, a_mv, values_b, *text_style, &mut display);
             Systick::delay(READ_INTERVAL.secs()).await;
          }        
       }
}    
     

     
     
     
     
     
     
     
     

     
     
     

     
     
     
     
   

     
     
     
     
     
     
     

     
