/*!  BEWARE THIS IS NOT WORKING PROPERLY! Even on some MCUs where it builds some things faked.
     Measure the internal mcu temperature sensor and an analog external TMP36 sensor.
     see https://github.com/stm32-rs/stm32f1xx-hal/blob/master/examples/adc.rs
     for stm32f4xx see examples in
      https://docs.rs/stm32f4xx-hal/0.8.3/stm32f4xx_hal/adc/struct.Adc.html
     http://ctms.engin.umich.edu/CTMS/Content/Activities/TMP35_36_37.pdf
     TMP36   analog temperature sensor
     Setup() functions return (mcutemp, tmp36, adcs) and set methods for ...
     CHECK METHODS ARE ALL ON RIGHT CHANNELS

      Notes of Interest:
      -I don't understand the details of setting  ADC or ADC clocks. If you know what you are
       doing you can probably do better than what is done below. Please let me know of important
       improvements by entering an issue at https://github.com/pdgilbert/eg_stm_hal .

      -If the MCU has enough adc's for each sensor (2 here) then a structure tying the channel (pin) to
       an adc can be returned from setup(). That works for bluepill and many others. This approach
       is commented out in the code below. However, stm32f401 and stm32f411 have only one adc.
       The setup() function cannot tie one adc into two structures so, to accomodate the possibility of
       fewer adc's than sensors, the adc(s)  have to be passed separately to the application code,
       which must deal with switching the adc between channels. (Possibly a closure could do this,
       but I don't think so.)
       Thus, to make the application code common, all adc's need to be passed back.
       The Sensor struct has an indication the optional channel (internal temp sensor has no channel)
       and the trait implementations encode the adc to be used.

     For digital temperature sensor exanples see  ds1820.rs, dht.rs and dht11.rs.
*/

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use panic_semihosting as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;

// Traits to be supported by methods on sensors. Self refers to a Sensor which is an optional pin.
// It would be possible to have just one trait, eg ReadSensor, that included both read_tempC and read_mv
//  but then both need to be implemented, which is not done below.

// possibly trait cfg's could be eliminated by using <T> or <T: Adcs> or Type: item =  x; ??

pub trait ReadTempC {
    // for reading channel temperature in degrees C on channel (self.ch)
   #![allow(non_snake_case)]
    #[cfg(feature = "stm32f0xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc>) -> i32;

    #[cfg(feature = "stm32f103")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> i32;
    #[cfg(any(feature = "stm32f100", feature = "stm32f101"))]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<ADC1>>) -> i32;

    #[cfg(feature = "stm32f3xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<ADC1>, Adc<ADC3>>) -> i32;
    #[cfg(feature = "stm32f4xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<ADC1>>) -> i32;
    #[cfg(feature = "stm32f7xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> i32;
    #[cfg(feature = "stm32h7xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>>) -> i32;
    #[cfg(feature = "stm32l0xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc<Ready>>) -> i32;
    #[cfg(feature = "stm32l1xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<Adc>) -> i32;
    #[cfg(feature = "stm32l4xx")]
    fn read_tempC(&mut self, adcs: &mut Adcs<ADC>) -> i32;
}

pub trait ReadMV {
    // for reading channel millivolts on channel (self.ch)
    #[cfg(feature = "stm32f0xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc>) -> u32;

    #[cfg(feature = "stm32f103")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> u32;
    #[cfg(any(feature = "stm32f100", feature = "stm32f101"))]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<ADC1>>) -> u32;

    #[cfg(feature = "stm32f3xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<ADC1>, Adc<ADC3>>) -> u32;
    #[cfg(feature = "stm32f4xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<ADC1>>) -> u32;
    #[cfg(feature = "stm32f7xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> u32;
    #[cfg(feature = "stm32h7xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>>) -> u32;
    #[cfg(feature = "stm32l0xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc<Ready>>) -> u32;
    #[cfg(feature = "stm32l1xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<Adc>) -> u32;
    #[cfg(feature = "stm32l4xx")]
    fn read_mv(&mut self, adcs: &mut Adcs<ADC>) -> u32;
}

pub struct Sensor<U> {
    // when no channel used (internal temp), ch may be None
    ch: U,
    //ch  : Option<U>,
}

// Possibly this should all be by MCU rather than some by HAL?

#[cfg(any(
    feature = "stm32f103",
    feature = "stm32f3xx",
    feature = "stm32f7xx",
    feature = "stm32h7xx"
))]
pub struct Adcs<T, U> {
    ad_1st: T,
    ad_2nd: U,
}

#[cfg(any(
    feature = "stm32f0xx",
    feature = "stm32f100",
    feature = "stm32f101",
    feature = "stm32f4xx",
    feature = "stm32l0xx",
    feature = "stm32l1xx",
    feature = "stm32l4xx"
))]
pub struct Adcs<T> {
    ad_1st: T,
}

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    adc::{Adc, VTemp},
    gpio::{gpiob::PB1, Analog},
    pac::Peripherals,
    prelude::*,
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (impl ReadTempC, impl ReadTempC + ReadMV, Adcs<Adc>) {
    // On stm32f030xc a temperature sensor is internally connected to the single adc.
    // No channel is specified for the mcutemp because it uses an internal channel.
    // see  https://github.com/stm32-rs/stm32f0xx-hal/blob/master/examples/adc_values.rs

    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpiob = p.GPIOB.split(&mut rcc);

    let adcs: Adcs<Adc> = Adcs {
        ad_1st: Adc::new(p.ADC, &mut rcc),
    };

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(cortex_m::interrupt::free(move |cs| {
            gpiob.pb1.into_analog(cs)
        })), //channel pb1
    };

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_1st.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32 //CHECK THIS
                }
                None => {
                    let t: f32 = a.ad_1st.read(&mut VTemp).unwrap();
                    (t / 100.0) as i32 //CHECK THIS
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC2
        fn read_mv(&mut self, a: &mut Adcs<Adc>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_1st.read(ch).unwrap(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}

#[cfg(feature = "stm32f103")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpiob::PB1, Analog},
    pac::{Peripherals, ADC1, ADC2},
    prelude::*,
};

//  stm32f100 and stm32f101 only have one adc
#[cfg(any(feature = "stm32f101", feature = "stm32f100"))]
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{gpiob::PB1, Analog},
    pac::{Peripherals, ADC1},
    prelude::*,
};

#[cfg(feature = "stm32f103")]
fn setup() -> (
    impl ReadTempC,
    impl ReadTempC + ReadMV,
    Adcs<Adc<ADC1>, Adc<ADC2>>,
) {
    //fn setup() ->  (Sensor<Option<PB1<Analog>>>,  Sensor<PB1<Analog>>,   Adcs<Adc<ADC1>, Adc<ADC2>>) {

    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.adcclk(2.MHz()).freeze(&mut flash.acr);

    // with above on bluepill  clocks.sysclk() is  8 Mhz and  clocks.adcclk() is  2 Mhz
    // with below on bluepill  clocks.sysclk() is 56 Mhz and  clocks.adcclk() is 14 Mhz
    // The mcu temp does not seem to be affected by this difference
    // but the external analog temperature (tmp36) is high by 6-7deg C with clock below.
    //let clocks = rcc.cfgr.use_hse(8.mhz()).sysclk(56.mhz())
    //    .pclk1(28.mhz()).adcclk(14.mhz()).freeze(&mut flash.acr);

    let mut gpiob = p.GPIOB.split();

    let adcs: Adcs<Adc<ADC1>, Adc<ADC2>> = Adcs {
        ad_1st: Adc::adc1(p.ADC1, clocks),
        ad_2nd: Adc::adc2(p.ADC2, clocks),
    };

    //The MCU temperature sensor is internally connected to the ADC12_IN16 input channel
    // so no channel needs to be specified here.

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog(&mut gpiob.crl)),
    }; //channel pb1

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_2nd.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let z = &mut a.ad_1st;
                    z.read_temp() as i32
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC2
        fn read_mv(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_2nd.read(ch).unwrap(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}

#[cfg(any(feature = "stm32f101", feature = "stm32f100"))]
fn setup() -> (impl ReadTempC, impl ReadTempC + ReadMV, Adcs<Adc<ADC1>>) {
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();

    let clocks = rcc.cfgr.adcclk(2.MHz()).freeze(&mut flash.acr);

    let mut gpiob = p.GPIOB.split();

    let adcs: Adcs<Adc<ADC1>> = Adcs {
        ad_1st: Adc::adc1(p.ADC1, clocks),
    };

    //The MCU temperature sensor is internally connected to the ADC12_IN16 input channel
    // so no channel needs to be specified here.

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog(&mut gpiob.crl)),
    }; //channel pb1

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1>>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_1st.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let z = &mut a.ad_1st;
                    z.read_temp() as i32
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ad_1st
        fn read_mv(&mut self, a: &mut Adcs<Adc<ADC1>>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_1st.read(ch).unwrap(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    adc::{Adc, CommonAdc, config::Config},
    gpio::{gpiob::PB1, Analog},
    pac::Peripherals,
    pac::{ADC1, ADC3},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    impl ReadTempC,
    impl ReadTempC + ReadMV,
    Adcs<Adc<ADC1>, Adc<ADC3>>,
) {
    // On stm32f303xc a temperature sensor is internally connected to ADC3_IN18.
    // Two adc's are used but no channel is specified for the mcutemp on adc3 because it uses an internal channel.

    // THIS BUILDS WITH CRITICAL SECTIONS COMMENTED OUT,  BUT DOES NOT YET BUILDING PROPERLY
    // see https://github.com/stm32-rs/stm32f3xx-hal/issues/163 for some suggestions

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();

    // Configure ADC clocks. See Notes of Interest above.
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

    let adc_common1_2 = CommonAdc::new(p.ADC1_2, &clocks, &mut rcc.ahb);
    let adc_common3_4 = CommonAdc::new(p.ADC3_4, &clocks, &mut rcc.ahb);

    let adcs: Adcs<Adc<ADC1>, Adc<ADC3>> = Adcs {
        ad_1st: Adc::new(p.ADC1, Config::default(), &clocks, &adc_common1_2,),
        ad_2nd: Adc::new(p.ADC3, Config::default(), &clocks, &adc_common3_4,),
    };
    // adc3 can use pb1, but not adc2

    //The MCU temperature sensor is internally connected to the ADC1 on channel 16
    // so no channel needs to be specified here. Also
    /*  https://www.st.com/resource/en/datasheet/stm32f303vc.pdf
    temperature sensor connected to ADC1 channel 16,
    VBAT/2 connected to ADC1 channel 17,
    Voltage reference VREFINT connected to the 4 ADCs channel 18,
    VOPAMP1 connected to ADC1 channel 15,
    VOPAMP2 connected to ADC2 channel 17,
    VREFOPAMP3 connected to ADC3 channel 17
    VREFOPAMP4 connected to ADC4 channel 17
    */

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog(&mut gpiob.moder, &mut gpiob.pupdr)),
    }; //channel pb1

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC3>>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_2nd.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let z = &mut a.ad_1st;
                    //z.read_temp() as i32;  //NEEDS TO CONNECT USING INTERNAL CHANNEL 16
                    // see https://github.com/stm32-rs/stm32f3xx-hal/issues/163
                    //let t = read_mcu_temp(&mut p.ADC1_2, &mut p.ADC1);
                    //t as i32
                    32 as i32
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC2
        fn read_mv(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC3>>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_2nd.read(ch).unwrap(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc, Temperature}, //SampleTime
    gpio::{gpiob::PB1, Analog},
    pac::{Peripherals, ADC1}, //ADC2},          // 405 has ADC2 but 401 and 411 have only one adc
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (impl ReadTempC, impl ReadTempC + ReadMV, Adcs<Adc<ADC1>>) {
    //fn setup() ->  (AdcCh<&'static Adc<ADC1>, Temperature>, AdcCh<&'static Adc<ADC1>, PB1<Analog>>) {

    // On stm32f401 and 411 a temperature sensor is internally connected to the single adc.
    // No channel is specified for the mcutemp because it uses an internal channel.
    // (stm32f405 has other adc's so the model as in stm32f1xx could be used for that.)

    // see https://docs.rs/stm32f4xx-hal/0.8.3/stm32f4xx_hal/adc/struct.Adc.html
    // and https://docs.rs/stm32f4xx-hal/0.8.3/stm32f4xx_hal/adc/struct.Adc.html#method.adc2

    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();

    //from datasheet:To synchronize A/D conversion and timers, the ADCs could be triggered by
    //any of TIM1,TIM2, TIM3, TIM4 or TIM5 timer.

    let _clocks = rcc
        .cfgr
        .hclk(48.MHz())
        .sysclk(48.MHz())
        .pclk1(24.MHz())
        .pclk2(24.MHz())
        .freeze();

    let gpiob = p.GPIOB.split();

    let adcs: Adcs<Adc<ADC1>> = Adcs {
        ad_1st: Adc::adc1(p.ADC1, true, AdcConfig::default()),
    };

    // no channel  one-shot conversion
    //The MCU temperature sensor is internally connected to input channel 18
    // THERE IS NO CHANNEL FOR mcutemp. REALLY SHOULD NOT NEED TO SPECIFY <PB1<Analog>>???

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // internal, no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog()),
    }; // TMP36 on pb1 using ADC1

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1>>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_1st.read(ch).unwrap().into(); //into converts u16 to f32
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let z = &mut a.ad_1st;
                    z.read(&mut Temperature).unwrap() as i32
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ad_1st
        fn read_mv(&mut self, a: &mut Adcs<Adc<ADC1>>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_1st.read(ch).unwrap().into(), //into converts u16 to u32
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    adc::Adc,
    gpio::{gpiob::PB1, Analog},
    pac::{Peripherals, ADC1, ADC2},
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    impl ReadTempC,
    impl ReadTempC + ReadMV,
    Adcs<Adc<ADC1>, Adc<ADC2>>,
) {
    // adc module in stm32f7xx_hal is only supported for stm32f765, stm32f767, and stm32f769. See
    //    https://github.com/stm32-rs/stm32f7xx-hal/blob/master/src/lib.rs
    // On stm32f722 a temperature sensor is internally connected to the same input channel as VBAT, ADC1_IN18.
    // When the temperature sensor and VBAT conversion are enabled at the same time, only VBAT conversion is performed.
    // The stm32f722 datasheet says it has 3 12bit ADCs.
    // Switched from stm32f722 to stm32f769 for adc support, but internal mcu temperature is still FAKED!
    // see   https://github.com/stm32-rs/stm32f7xx-hal/issues/116

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpiob = p.GPIOB.split();

    let adcs: Adcs<Adc<ADC1>, Adc<ADC2>> = Adcs {
        ad_1st: Adc::adc1(p.ADC1, &mut rcc.apb2, clocks, 4, true), // NO IDEA WHAT ARGS 4 AND 5 ARE! FIND EXAMPLE OR DOCUMENTATION
        ad_2nd: Adc::adc2(p.ADC2, &mut rcc.apb2, clocks, 4, true),
    };

    //The MCU temperature sensor is internally connected to the ADC12_IN16 input channel
    // so no channel needs to be specified here.

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<PB1<Analog>> = Sensor {
        ch: gpiob.pb1.into_analog(),
    };

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> i32 {
            match &mut self.ch {
                // it should be possible to call next method here  read_tempC(ch) on Sensor<PB1<Analog>>
                // but doesn't seem to get to this impl when there is Some(ch)?
                Some(ch) => {
                    //hprintln!("panic at Some(ch)").unwrap();
                    //panic!()
                    let v: f32 = a.ad_2nd.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let z = &mut a.ad_1st;
                    // Need internal hal setting ch 16.  See https://github.com/stm32-rs/stm32f7xx-hal/issues/116
                    //z.read(&mut Temperature) as i32
                    0.0 as i32 // internal temp returned as zero   FAKE READ
                }
            }
        }
    }

    impl ReadTempC for Sensor<PB1<Analog>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> i32 {
            match &mut self.ch {
                ch => {
                    let v: f32 = a.ad_2nd.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32
                }
            }
        }
    }

    impl ReadMV for Sensor<PB1<Analog>> {
        // TMP36 on PB1 using ADC2
        fn read_mv(&mut self, a: &mut Adcs<Adc<ADC1>, Adc<ADC2>>) -> u32 {
            match &mut self.ch {
                ch => a.ad_2nd.read(ch).unwrap(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    //rcc::rec::AdcClkSel,
    adc, // for adc::Resolution::SIXTEENBIT
    adc::{Adc, Enabled, Temperature},
    delay::Delay,
    gpio::{gpiob::PB1, Analog},
    pac::Peripherals,
    prelude::*,
    stm32::{ADC1, ADC3},
};

//#[cfg(feature = "stm32h7xx")]
//use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (
    impl ReadTempC,
    impl ReadTempC + ReadMV,
    Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>>,
) {
    // On stm32h742 a temperature sensor is internally connected to ADC3_IN18.
    // Two adc's are used but no channel is specified for the mcutemp on adc3 because it uses an internal channel.
    // BUILDS BUT CHECK https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/temperature.rs
    // REGARDING CALIBRATION

    let cp = cortex_m::Peripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;
    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);

    let mut delay = Delay::new(cp.SYST, clocks);

    // To use adc1 and adc2 they have to be done together because ADC12 is consumed.
    //let (adc1, adc2) = adc12(p.ADC1, p.ADC2, &mut delay, ccdr.peripheral.ADC12, &ccdr.clocks);
    // but this example uses adc1 and adc3.

    let mut adc1 = Adc::adc1(p.ADC1, &mut delay, ccdr.peripheral.ADC12, &ccdr.clocks);
    adc1.set_resolution(adc::Resolution::SIXTEENBIT);
    let adc1 = adc1.enable();

    let ch_tmp36 = gpiob.pb1.into_analog();

    //   https://www.st.com/resource/en/datasheet/stm32h743bi.pdf
    // See also https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/temperature.rs
    //     and  https://github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/adc.rs
    //   also  regarding  voltage = reading * (vref/resolution)

    let mut adc3 = Adc::adc3(p.ADC3, &mut delay, ccdr.peripheral.ADC3, &ccdr.clocks);
    adc3.set_resolution(adc::Resolution::SIXTEENBIT);

    let mut ch_mcu = Temperature::new();
    ch_mcu.enable(&adc3);

    let adc3 = adc3.enable();

    let adcs: Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>> = Adcs {
        ad_1st: adc1, // tmp36 temp and mv
        ad_2nd: adc3, // mcu temp
    };

    let mcutemp = Sensor { ch: Some(ch_mcu) }; // internal temperature channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor { ch: Some(ch_tmp36) }; // TMP36 on pb1 using ADC3

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>>) -> i32 {
            match &mut self.ch {
                Some(ch_tmp36) => {
                    let z = &mut a.ad_1st;
                    let v: u32 = z.read(ch_tmp36).expect("TMP36 temperature read failed.");
                    (v as f32 / 12.412122) as i32 - 50 as i32
                }

                None => {
                    hprintln!("panic at ch None)").unwrap();
                    panic!()
                }
            }
        }
    }

    impl ReadTempC for Sensor<Option<adc::Temperature>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>>) -> i32 {
            match &mut self.ch {
                Some(ch_mcu) => {
                    let z = &mut a.ad_2nd;
                    let v: u32 = z.read(ch_mcu).expect("MCU temperature read failed.");
                    //  CHECK above link regarding  voltage = reading * (vref/resolution)
                    (v as f32 / 12.412122) as i32 - 50 as i32
                }

                None => {
                    hprintln!("panic at ch None)").unwrap();
                    panic!()
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC1
        fn read_mv(&mut self, a: &mut Adcs<Adc<ADC1, Enabled>, Adc<ADC3, Enabled>>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_1st.read(ch).unwrap(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    adc::{Adc, Ready, VTemp},
    gpio::{gpiob::PB1, Analog},
    pac::Peripherals,
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
#[cfg(feature = "stm32l0xx")]
fn setup() -> (impl ReadTempC, impl ReadTempC + ReadMV, Adcs<Adc<Ready>>) {
    // On stm32L0X2 a temperature sensor is internally connected to the single adc.
    // No channel is specified for the mcutemp because it uses an internal channel ADC_IN18.
    // Needs to be built with  --release  to fit memory.
    //ALMOST WORKING BUT CANNOT ACCESS INTERNAL TEMP. ISSUE https://github.com/stm32-rs/stm32f7xx-hal/issues #116

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let adc = p.ADC.constrain(&mut rcc);

    let gpiob = p.GPIOB.split(&mut rcc);

    // only one  adc

    let adcs: Adcs<Adc<Ready>> = Adcs { ad_1st: adc };

    //The MCU temperature sensor is internally connected to the ADC12_IN16 input channel
    // so no channel needs to be specified here.

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog()),
    }; //channel pb1

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc<Ready>>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_1st.read(ch).unwrap();
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let v: f32 = a.ad_1st.read(&mut VTemp).unwrap();
                    (v / 12.412122) as i32 - 50 as i32 //CHECK THIS
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC2
        fn read_mv(&mut self, a: &mut Adcs<Adc<Ready>>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_1st.read(ch).unwrap(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    adc::{Adc, Precision},
    gpio::{gpiob::PB1, Analog},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    stm32::Peripherals,
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (impl ReadTempC, impl ReadTempC + ReadMV, Adcs<Adc>) {
    // On stm32L100 there seems to be no internal temperature sensor. \
    // MCU temperature set as zero.

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());
    //let clocks  = rcc.cfgr.freeze();

    let gpiob = p.GPIOB.split(&mut rcc);
    //  see https://github.com/stm32-rs/stm32l1xx-hal/blob/master/examples/adc.rs

    let mut adc = p.ADC.adc(&mut rcc);
    adc.set_precision(Precision::B_12);

    let adcs: Adcs<Adc> = Adcs { ad_1st: adc };

    // no channel  one-shot conversion

    //There is no MCU temperature sensor (I think).
    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // internal, no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog()),
    }; // TMP36 on pb1 using ADC1

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<Adc>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let z = &mut a.ad_1st;
                    let v: u32 = z.read(ch).expect("TMP36 temperature read failed.");
                    //let v: u32 = z.read(ch).unwrap().into(); //into converts u16 to f32
                    (v as f32 / 12.412122) as i32 - 50 as i32
                }

                None => {
                    0.0 as i32 // internal temp returned as zero
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC
        fn read_mv(&mut self, a: &mut Adcs<Adc>) -> u32 {
            match &mut self.ch {
                Some(ch) => {
                    let z = &mut a.ad_1st;
                    let v: u32 = z.read(ch).expect("TMP36 temperature read failed.");
                    //let v: u32 = z.read(ch).unwrap().into();  //into converts u16 to u32
                    v as u32
                }

                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    adc::{Temperature, ADC},
    delay::Delay,
    gpio::{gpiob::PB1, Analog},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (impl ReadTempC, impl ReadTempC + ReadMV, Adcs<ADC>) {
    // On stm32L4X2 a temperature sensor is internally connected to the single adc.
    // No channel is specified for the mcutemp because it uses an internal channel ADC1_IN17.

    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    //    only one adc

    // unclear why Delay is needed in this hal. (Thus needs cp.)
    let mut delay = Delay::new(cp.SYST, clocks);
    let adcs: Adcs<ADC> = Adcs {
        ad_1st: ADC::new(
            p.ADC1,
            p.ADC_COMMON,
            &mut rcc.ahb2,
            &mut rcc.ccipr,
            &mut delay,
        ),
    };

    //The MCU temperature sensor is internally connected to the ADC12_IN16 input channel
    // so no channel needs to be specified here.

    let mcutemp: Sensor<Option<PB1<Analog>>> = Sensor { ch: None }; // no channel

    let tmp36: Sensor<Option<PB1<Analog>>> = Sensor {
        ch: Some(gpiob.pb1.into_analog(&mut gpiob.moder, &mut gpiob.pupdr)), //channel pb1
    };

    impl ReadTempC for Sensor<Option<PB1<Analog>>> {
        fn read_tempC(&mut self, a: &mut Adcs<ADC>) -> i32 {
            match &mut self.ch {
                Some(ch) => {
                    let v: f32 = a.ad_1st.read(ch).unwrap().into();
                    (v / 12.412122) as i32 - 50 as i32
                }

                None => {
                    let z = &mut a.ad_1st;
                    let v = z
                        .read(&mut Temperature)
                        .expect("MCU temperature read failed.");
                    (v as f32 / 12.412122) as i32 - 50 as i32 // CHECK SCALE
                }
            }
        }
    }

    impl ReadMV for Sensor<Option<PB1<Analog>>> {
        // TMP36 on PB1 using ADC
        fn read_mv(&mut self, a: &mut Adcs<ADC>) -> u32 {
            match &mut self.ch {
                Some(ch) => a.ad_1st.read(ch).unwrap().into(),
                None => panic!(),
            }
        }
    }

    (mcutemp, tmp36, adcs)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    let (mut mcutemp, mut tmp36, mut adcs) = setup();

    /*
      TMP35 has linear output with scale calculation as follows.
      Vin = 3.3v * ADCvalue / 4096     (12 bit adc has  2**12 = 4096 steps)
      TMP35 scale is 100 deg C per 1.0v (slope 10mV/deg C) and goes through
         <50C, 1.0v>,  so 0.0v is  -50C.
      see https://www.analog.com/media/en/technical-documentation/data-sheets/TMP35_36_37.pdf
      so temp = (100 * 3.3 * ADCvalue / 4096 )  - 50 = 0.0805664 * ADCvalue - 50

     This compiles but the link fails because the bin is too big for flash on bluepill
       let adc_temp: f64 = (0.0805664 * adc_value as f64 ) - 50.0 ;
     these work
       let adc_temp:  i16 = ((0.0805664f32 * adc_value as f32 ) - 50.0f32) as i16 ;
       let adc_temp:  i16 = (0.0805664f32 * adc_value as f32 ) as i16 - 50  ;
       let adc_temp:  i16 = (adc_value as f32 / 12.412122 ) as i16 - 50  ;
     and this works but the rounding is bad (a few degrees off)
       let adc_temp:  i16 = (adc_value / 12 ) as i16 - 50  ;
    */

    loop {
        let mcu_value = mcutemp.read_tempC(&mut adcs);
        hprintln!("inaccurate MCU temp: {}", mcu_value).unwrap();

        let tmp36_mv: u32 = tmp36.read_mv(&mut adcs);
        let tmp36_temp: i32 = tmp36.read_tempC(&mut adcs);
        hprintln!("external sensor: {} mV,   {} C.", tmp36_mv, tmp36_temp).unwrap();
    }
}
