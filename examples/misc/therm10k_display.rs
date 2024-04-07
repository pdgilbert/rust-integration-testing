// THIS NEEDS WORK. compare projects/temperature-display.rs
// NOTE THAT impl DelayNs does not work dht11::read as of Feb 2024.
//    It still needs DelayUs which is called DelayUsType below.

//! Measure temperature with 10k thermistor sensor (NTC 3950 10k thermistors probe) and temperature and
//! humidity from a DHT-11 (or DHT-22) sensor. Display on SSD1306 OLED display.
//! 
//! One side of the thermistor is connected to GND and other side to adc pin and also through
//! a 10k sresistor to VCC. That makes the max voltage about VCC/2, so about 2.5v when VCC is 5v.
//! and 1.6v when vcc is 3.2v. This is convenient for adc pins that are not 5v tolerant.
//! This means the voltage varies inversely compared to connecting throught the resistor to GND 
//! as is sometimes done. (Since NTC resistance goes down as temperature goes up, this means
//! higher temperature gives higher voltage measurement.) Regarding NTC thermistors see, 
//! for example, https://eepower.com/resistor-guide/resistor-types/ntc-thermistor/#
//!
//! If 3.3v is supplied through BluePill regulator from 5v USB probe BEWARE of regulator current limit.
//! Some places it is claimed that when the limit is exceeded then 5v is supplied but mine failed 
//! by dropping voltage to 2.8v when the SSD1306, 10k thermistor and DHT-11 were on the 3.3v using 
//! USB power rather than battery. It ran but 10K temperature accuracy is questionable.
//!
//! See setup() functions for thepin settings on various boards.
//! 
//! See misc/temperature.rs and misc/temperature_display.rs for other sensors and 
//! the internal mcu temperature measurement.
//! 
//! Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display.
//! 
//! The SSD1306 OLED display connects to the I2C bus: VCC  (or VDD) to 3.3v, and also to GND, SDA, and SCL. 
//! 
//! Voltage at the thermistor to fixed resistor connection is measured on the ADC pin.
//! The choice of 10k series resistor in the voltage divider ...
//! Values in the temperature calculation below are very rough based on my extremely crude calibration
//! effort, but see for example 
//!     https://www.mathscinotes.com/2014/05/yet-another-thermistor-discussion/
//!  or https://www.jameco.com/Jameco/workshop/TechTip/temperature-measurement-ntc-thermistors.html
//!  or  https://learn.adafruit.com/thermistor/using-a-thermistor


// regarding adc use see
// https://www.st.com/resource/en/application_note/cd00258017-stm32s-adc-modes-and-their-applications-stmicroelectronics.pdf

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use cortex_m_rt::entry;

//use embedded_hal::adc::OneShot;
//use nb::block;

use core::fmt::Write;

// Note hprintln will not run without an ST-link probe (eg. not on battery power)
// The use statement does not need to be removed, but it will be unused and causes warnings.
//use cortex_m_semihosting::hprintln;
//use rtt_target::{rprintln, rtt_init_print};

const DISPLAY_LINES: usize = 2; 

use embedded_graphics::{
    mono_font::{ascii::FONT_8X13, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::{read, Reading};
#[cfg(feature = "dht22")]
use dht_sensor::dht22::{read, Reading};

use rust_integration_testing_of_examples::led::LED;

//use dht_sensor::Delay;  // trait, whereas timer::Delay is a type does not yet use DelayNs

use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

use hal::{
   pac::{Peripherals, CorePeripherals},
   //i2c::I2c as I2cType,
   gpio::{gpioa::PA8, Output, OpenDrain, GpioExt},
   gpio::{Analog, gpioa::{PA1}},
   prelude::*,  
   adc::Adc,
   pac::ADC1,
};


use embedded_hal::{
   i2c::I2c as I2cTrait,
   delay::DelayNs,
};

///////////////////////////////

type DhtType = PA8<Output<OpenDrain>>;

pub struct Sensor<U, A> { ch: U, adc: A }

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

///////////////////////////////

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
type SensorType = Sensor<PA1<Analog>, Adc>;

#[cfg(feature = "stm32f1xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f4xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f7xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32g0xx")]
type SensorType = Sensor<PA1<Analog>, Adc>;

#[cfg(feature = "stm32g4xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1, Disabled>>; // possibly needs to be Active

#[cfg(feature = "stm32h7xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1, Enabled>>;

#[cfg(feature = "stm32l0xx")]
type SensorType = Sensor<PA1<Analog>, Adc<Ready>>;

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
type SensorType = Sensor<PA1<Analog>, Adc>;

#[cfg(feature = "stm32l4xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

///////////////////////////////


#[cfg(feature = "stm32f0xx")]
pub fn setup(mut dp: Peripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let rcc = dp.RCC.configure();
    let mut rcc = rcc.freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let mut dht = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high().ok();

    let sens: SensorType = Sensor {
        ch: cortex_m::interrupt::free(move |cs| {gpioa.pa1.into_analog(cs)}),
        adc: Adc::new(dp.ADC, &mut rcc),
    }; 


    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
     }

    let (i2c, _i2c2) = i2c::setup_i2c1_i2c2(dp.I2C1, dp.GPIOB.split(&mut rcc), &mut rcc);

    let led = cortex_m::interrupt::free(move |cs| gpioc.pc13.into_push_pull_output(cs));

    (sens, dht, i2c, led, delay)
}


#[cfg(feature = "stm32f1xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    //            (..., BlockingI2c<I2C1, impl Pins<I2C1>>, ...) 
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    let mut gpioa = dp.GPIOA.split();
    let mut gpioc = dp.GPIOC.split();

    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(&mut gpioa.crl), //channel
        adc: Adc::adc1(dp.ADC1, clocks),
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
     }

    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high();

    let (i2c, _i2c2) = i2c::setup_i2c1_i2c2(dp.I2C1, dp.GPIOB.split(), &mut dp.AFIO.constrain(), &clocks);

    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    adc::{CommonAdc, config::Config},
};

#[cfg(feature = "stm32f3xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let adc_common = CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);

    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr), //channel
        adc: Adc::new(dp.ADC1, Config::default(), &clocks, &adc_common, ),
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
     }

    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high().ok();

    let (i2c, _i2c2) = i2c::setup_i2c1_i2c2(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb), clocks, rcc.apb1);

    let led = gpioe.pe15.into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    timer::SysDelay as DelayUsType,
    timer::SysTimerExt,
    adc::{config::AdcConfig}, //SampleTime
    //gpio::{Alternate, OpenDrain, gpiob::{PB8, PB9,  PB10, PB3, Parts as PartsB}},
    i2c::{I2c},
};

#[cfg(feature = "stm32f4xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, DelayUsType) {
                //(SensorType, I2c<I2C1, impl Pins<I2C1>>, LedType, DelayType) {
    let rcc = dp.RCC.constrain();
    let mut clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split(); 

    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(), //channel
        adc: Adc::adc1(dp.ADC1, true, AdcConfig::default()),
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() as u32}
    }

    let mut dht = gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high();

    let gpiob = dp.GPIOB.split();
    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    let sda = gpiob.pb9.into_alternate_open_drain(); 

    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.kHz(), &clocks);

    let led = gpioc.pc13.into_push_pull_output();

    // let mut delay = Delay::new(cp.SYST, clocks); 
    // Delay::new() works with DelayNs but needs older delay for dht sensor crate

    //let delay = cp.SYST.delay(&clocks);  // needs timer::SysTimerExt needs DelayNs 
    //let delay = dp.TIM2.delay(&clocks);  // needs timer::TimerExt and type SysDelay

    let delay = cp.SYST.delay(&mut clocks); 

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    rcc::CoreClocks as Clocks,
};

#[cfg(feature = "stm32f7xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(), //channel
        adc: Adc::adc1(dp.ADC1, &mut rcc.apb2, &clocks, 4, true),
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap()}
    }

    let mut dht =  gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high();

    let mut sda = gpiob.pb9.into_push_pull_output();
    let mut scl = gpiob.pb8.into_alternate_open_drain();

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        //400.khz(),
        Mode::Fast {
            frequency: 400_000.Hz(),
        },
        &clocks,
        &mut rcc.apb1,
        1000,
    );

    let led = gpioc.pc13.into_push_pull_output();

    let mut delay = dp.TIM2.delay_us(&clocks);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32g0xx")] 
use stm32g0xx_hal::{
    analog::adc::{Adc,},  // OversamplingRatio, Precision, SampleTime, VBat},
};

#[cfg(feature = "stm32g0xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let mut rcc = dp.RCC.constrain();
// 
//  see https://github.com/stm32-rs/stm32g0xx-hal/blob/main/examples/adc.rs  
// dp.ADC.constrain(&mut rcc);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(), //channel
        adc: Adc::new(dp.ADC, &mut rcc ),           //  NEEDS PROPER CONFIGURATION
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read_voltage(&mut self.ch).unwrap() as u32}
    }

    let mut dht = gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high().unwrap();
    
    let scl = gpiob.pb8.into_open_drain_output();
    let mut sda = gpiob.pb9.into_push_pull_output();
    let i2c = I2c::i2c1(dp.I2C1,  sda, scl,  i2cConfig::with_timing(0x2020_151b), &mut rcc);

    let led = gpioc.pc13.into_push_pull_output();

    //let mut delay = DelayType{};
    let mut delay = dp.TIM2.delay(&mut rcc);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32g4xx")] 
use stm32g4xx_hal::{
    pac::{TIM2},
    timer::{Timer, CountDownTimer},
    time::{ExtU32, RateExtU32},
    delay::DelayFromCountDownTimer,
    adc::{config::{SampleTime}, Disabled, AdcClaim, ClockSource},
    i2c::{Config},
};

#[cfg(feature = "stm32g4xx")]
type DelayUsType = DelayFromCountDownTimer<CountDownTimer<TIM2>>;
// impl DelayNs does not work dht11::read which needs DelayUs

#[cfg(feature = "stm32g4xx")]
pub fn setup(dp: Peripherals, _cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, DelayUsType) {
    let mut rcc = dp.RCC.constrain();

    //let cp = CorePeripherals::take().unwrap();
    //let mut delay = cp.SYST.delay(&rcc.clocks);
   
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let pin = gpioa.pa1.into_analog();
    let adc1 = dp.ADC1.claim(ClockSource::SystemClock, &rcc, &mut delay, true);  // Adc::new(...  would be nice

    let sens: SensorType = Sensor {
        ch:  pin,
        adc: adc1,
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { 
           let sample = self.adc.convert(&self.ch, SampleTime::Cycles_640_5);
           self.adc.sample_to_millivolts(sample) as u32
        } 
    }

    let mut dht = gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high().unwrap();

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    let gpiob = dp.GPIOB.split(&mut rcc);
    let sda = gpiob.pb9.into_alternate_open_drain(); 
    let scl = gpiob.pb8.into_alternate_open_drain(); 
    //let i2c = i2c1.i2c(sda, scl, Config::new(400.kHz()), rcc);
    //let i2c = I2c::new(i2c1, (scl, sda), 400.kHz(), &clocks);
    let i2c = dp.I2C1.i2c(sda, scl, Config::new(400.kHz()), &mut rcc);

    let led = gpioc.pc13.into_push_pull_output();

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    adc,
    adc::{Enabled, },
    //rcc::CoreClocks as Clocks,
    pac::TIM2,
    timer::Timer,
    delay::DelayFromCountDownTimer,
};

#[cfg(feature = "stm32h7xx")]
type DelayUsType = DelayFromCountDownTimer<Timer<TIM2>>;

#[cfg(feature = "stm32h7xx")]
pub fn setup(dp: Peripherals, _cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, DelayUsType) {
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    //let mut delay = DelayType{};
    //let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);
    //let mut delay = DelayUsType::new(cp.SYST, clocks); 
     let timer = dp.TIM2.timer(1.Hz(), ccdr.peripheral.TIM2, &clocks);
     let mut delay = DelayFromCountDownTimer::new(timer);

    let mut adc1 = Adc::adc1(dp.ADC1, 4.MHz(), &mut delay, ccdr.peripheral.ADC12, &ccdr.clocks);
    adc1.set_resolution(adc::Resolution::SixteenBit);
    let adc1 = adc1.enable();

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(), //channel
        adc: adc1,
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
     }

    let mut dht = gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high();

    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    //let i2cx = ccdr.peripheral.I2C1;

    //let (i2c, _i2c2) = i2c::setup_i2c1_i2c2(dp.I2C1, gpiob, i2cx, &clocks);

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();
    let i2c = dp.I2C1.i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C1, &clocks);

    let led = gpioc.pc13.into_push_pull_output();

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    adc::{Ready},
    rcc::Config, 
};

#[cfg(feature = "stm32l0xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let mut rcc = dp.RCC.freeze(Config::hsi16());
    //let clocks = rcc.clocks;

    let gpioa = dp.GPIOA.split(&mut rcc);
    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(), //channel
        adc: dp.ADC.constrain(&mut rcc),
        //adc: Adc::adc(dp.ADC, clocks),
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
        //fn read_mv(&mut self)    -> u32 { self.adc.read_available().unwrap() }
     }

    let mut dht = gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high().ok();


    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let led = gpioc.pc13.into_push_pull_output(); 

    let (i2c, _i2c2) = i2c::setup_i2c1_i2c2(dp.I2C1, gpiob, rcc);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    adc::{Precision},
    rcc, // for ::Config but avoid name conflict with serial
};

#[cfg(feature = "stm32l1xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    //let clocks = rcc.clocks;

    let mut adc = dp.ADC.adc(&mut rcc);
    adc.set_precision(Precision::B_12);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);
    
    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(), //channel
        adc: adc,
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() }
     }

    let mut dht = gpioa.pa8.into_open_drain_output();
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high().ok();

    let led = gpioc.pc9.into_push_pull_output(); 

    let (i2c, _i2c2) = i2c::setup_i2c1_i2c2(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    adc::{AdcCommon},
};

#[cfg(feature = "stm32l4xx")]
pub fn setup(dp: Peripherals, cp: CorePeripherals) -> (SensorType, DhtType, impl I2cTrait, impl LED, impl DelayNs) {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80.MHz()).pclk1(80.MHz()).pclk2(80.MHz()).freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);

    //let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);
    let mut delay = DelayType{};

    let adc_common = AdcCommon::new(dp.ADC_COMMON, &mut rcc.ahb2);
    let adc = Adc::adc1(dp.ADC1, adc_common, &mut rcc.ccipr, &mut delay );

    let sens: SensorType = Sensor {
        ch:  gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr), //channel
        adc: adc,
    }; 
    impl ReadAdc for SensorType {
        fn read_mv(&mut self)    -> u32 { self.adc.read(&mut self.ch).unwrap() as u32 }
     }

    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    // Pulling the pin high to avoid confusing the sensor when initializing.
    dht.set_high();

    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c1 = i2c1.i2c((scl, sda), 400.khz(), &mut rcc);

    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper); 

    (sens, dht, i2c, led, delay)
}

// End of HAL/MCU specific setup.


fn show_display<S>(
    mv: u32,
    temp: i64,
    dht_temp: i8, 
    dht_humidity: u8, 
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; DISPLAY_LINES] = [
        heapless::String::new(),
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // it is now possible to use \n in place of separate writes, with one line rather than vector.
    //write!(lines[0], "10k {:4}mV{:3}°C", mv, temp).unwrap();
    write!(lines[0], "10k {:4}mV{:3}C", mv, temp).unwrap();
    write!(lines[1], "dht{:4}%RH{:3}C", dht_humidity, dht_temp).unwrap();

    disp.clear_buffer();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            //using font 6x10, 10 high + 2 space = 12
            //using font 8X13, 13 high + 2 space = 15       
            Point::new(0, i as i32 * 15),
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("therm10k_display example");
    //hprintln!("therm10k_display example").unwrap();

    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    // setup needs to return delay because adc for some hals needs delay.
    // setup needs cp for delay from syst.
    let (mut sens, mut dht, i2c, mut led, mut delay) = setup(dp, cp);
    
    led.blink(500_u16, &mut delay);  // to confirm startup

    let interface = I2CDisplayInterface::new(i2c);

    //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_8X13)
        .text_color(BinaryColor::On)
        .build();

    // crude linear aproximation
    // t = a - v/b , t degrees C, v in mV, b negative of inverse slope, a includes degrees K to C
    let a = 75i64;
    let b = 40u32; 

    loop {
        // Blink LED  to check that loop is actually running.
        led.blink(50_u16, &mut delay);

        // Read 10k probe on adc
        let mv = sens.read_mv();
        let temp:i64  = a - (mv / b) as i64 ;
        //hprintln!("probe  {}mV  {}C ", mv, temp).unwrap();

        let z = read(&mut delay, &mut dht);
        let (dht_temp, dht_humidity) = match z {
            Ok(Reading {temperature, relative_humidity,})
               =>  {//hprintln!("temperature:{}, humidity:{}, ", temperature, relative_humidity).unwrap();
                    (temperature, relative_humidity)
                   },
            Err(_e) 
               =>  {//hprintln!("dht Error {:?}. Using default temperature:{}, humidity:{}", e, 25, 40).unwrap(); 
                    //panic!("Error reading DHT"),
                    (25, 40)  //supply default values
                   },
        };
 
        show_display(mv, temp, dht_temp, dht_humidity, text_style, &mut display);

        delay.delay_ms(5000);
    }
}
