#![deny(unsafe_code)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

/////////////////////   ads
use ads1x1x::{Ads1x1x, channel, FullScaleRange, TargetAddr};

/////////////////////   ina
use ina219::{address::{Address, Pin}, measurements::BusVoltage, SyncIna219};

/////////////////////   ssd
use ssd1306::{mode::BufferedGraphicsMode, prelude::*, I2CDisplayInterface, Ssd1306};

const DISPLAYSIZE:ssd1306::prelude::DisplaySize128x32 = DisplaySize128x32;
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
   i2c::I2c as I2cTrait,
};

use rust_integration_testing_of_examples::
       setup::{Peripherals, DelayNs, prelude::*, block, RccExt,};


#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
      pac::I2C1,
      i2c::{DutyCycle, Mode as i2cMode, BlockingI2c,}, 
};


#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    timer::Timer,
    time::{ExtU32, RateExtU32},
    i2c::Config,
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
   delay::DelayFromCountDownTimer,
   //pwr::PwrExt,
};



#[cfg(feature = "stm32f1xx")]            
pub fn setup_from_dp(dp: Peripherals) ->  ( impl I2cTrait<u8>, impl DelayNs) { // NEEDS u8 NOT I2C1 Why?

   let mut flash = dp.FLASH.constrain();
   let rcc = dp.RCC.constrain();
   let clocks = rcc.cfgr.freeze(&mut flash.acr);

   let mut gpiob = dp.GPIOB.split();
   let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh); 
   let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh); 

   let mut afio = dp.AFIO.constrain();

   // still only on branch = "rmp-new"
   let i2c = BlockingI2c::<I2C1>::new(
                  dp.I2C1
                  .remap(&mut afio.mapr),  // add this for PB8, PB9
                  (scl, sda),
                  i2cMode::Fast {frequency: 400.kHz(), duty_cycle: DutyCycle::Ratio16to9,},
                  &clocks, 1000, 10, 1000, 1000,);

   // need  ::<1000000_u32>  for `FREQ` of the method `delay   WHY?
   let delay = dp.TIM2.delay::<1000000_u32>(&clocks);

   (i2c, delay)
}


#[cfg(feature = "stm32f4xx")]            
pub fn setup_from_dp(dp: Peripherals) ->  ( impl I2cTrait<u8>, impl DelayNs) { // NEEDS u8 NOT I2C1 Why?
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


#[cfg(feature = "stm32g4xx")]
pub fn setup_from_dp(dp: Peripherals) ->  (impl I2cTrait<u8>, impl DelayNs,) {
   let mut rcc = dp.RCC.constrain();

   let gpiob = dp.GPIOB.split(&mut rcc);

   let scl = gpiob.pb8.into_alternate_open_drain(); 
   let sda = gpiob.pb9.into_alternate_open_drain(); 

   let i2c = dp.I2C1.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

   let clocks = rcc.clocks;  // not sure if this is right

   let timer1 = Timer::new(dp.TIM3, &clocks);
   let delay = DelayFromCountDownTimer::new(timer1.start_count_down(100.millis()));

   (i2c, delay)
}


#[cfg(feature = "stm32h7xx")]
pub fn setup_from_dp(dp: Peripherals) ->  ( impl I2cTrait<u8>, impl DelayNs) { // NEEDS u8 NOT I2C1 Why?
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

///////////////////////////////////////////////////////////////////////////////////////////


fn display<S>(
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

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let (i2cset, mut delay) = setup_from_dp(dp);

    let i2cset_ref_cell = RefCell::new(i2cset);
    let adc_a_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let adc_b_rcd = RefCellDevice::new(&i2cset_ref_cell); 
    let ina_rcd   = RefCellDevice::new(&i2cset_ref_cell); 
    let ssd_rcd   = RefCellDevice::new(&i2cset_ref_cell); 

    /////////////////////   ads
    let mut adc_a = Ads1x1x::new_ads1015(adc_a_rcd, TargetAddr::Gnd);
    let mut adc_b = Ads1x1x::new_ads1015(adc_b_rcd, TargetAddr::Vdd);

    // set FullScaleRange to measure expected max voltage.
    adc_a.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();
    adc_b.set_full_scale_range(FullScaleRange::Within4_096V).unwrap();

    /////////////////////   ina
    let mut ina = SyncIna219::new( ina_rcd, Address::from_pins(Pin::Gnd, Pin::Gnd)).unwrap(); 

    /////////////////////   ssd
    let interface = I2CDisplayInterface::new(ssd_rcd); //default address 0x3C
    //let interface = I2CDisplayInterface::new_custom_address(ssd_rcd,   0x3D);  //alt address

    let mut disp = Ssd1306::new(interface, DISPLAYSIZE, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    disp.init().unwrap();

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
    .draw(&mut disp)
    .unwrap();

    /////////////////////   measure and display in loop
    loop {
        let voltage = ina.bus_voltage().unwrap();  

        //let a_mv = block!(DynamicOneShot::read(&mut adc_a, ChannelSelection::SingleA0)).unwrap_or(8091);
        let a_mv = block!(adc_a.read(channel::SingleA0)).unwrap_or(8091);

        let values_b = [
            block!(adc_b.read(channel::SingleA0)).unwrap_or(8091),
            block!(adc_b.read(channel::SingleA1)).unwrap_or(8091),
        ];

        display(voltage, a_mv, values_b, text_style, &mut disp);
        
        delay.delay_ms(2000); // sleep for 2s
    }
}
