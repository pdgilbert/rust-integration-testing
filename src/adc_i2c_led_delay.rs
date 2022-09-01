//! Note that pin settings are specific to a board pin configuration used for testing,
//! despite the cfg feature flags suggesting it may be for a HAL.

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;


pub struct Sensor<U, A> { ch: U, adc: A }

pub trait ReadAdc {
    // for reading on channel(self.ch) in mV.
    fn read_mv(&mut self)    -> u32;
}

pub use crate::led::{setup_led, LED, LedType};
pub use crate::i2c::{setup_i2c1, I2c1Type};



#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    adc::Adc,
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    delay::Delay,
    pac::{CorePeripherals, Peripherals},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
type SensorType = Sensor<PA1<Analog>, Adc>;

#[cfg(feature = "stm32f0xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32f0xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, Delay) {
    let mut dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);

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

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), &mut rcc);
    let led = setup_led(dp.GPIOC.split(&mut rcc));
    let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &rcc);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    adc::Adc,
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    timer::SysDelay as Delay,
    pac::{CorePeripherals, Peripherals, ADC1,},
    prelude::*,
};

#[cfg(feature = "stm32f1xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f1xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32f1xx")]
pub type DelayType = Delay;


#[cfg(feature = "stm32f1xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, DelayType) {
    //            (..., BlockingI2c<I2C1, impl Pins<I2C1>>, ...) 
    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    let mut gpioa = dp.GPIOA.split();
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

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &mut dp.AFIO.constrain(), &clocks);
    let led = setup_led(dp.GPIOC.split());

    //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &clocks);
    let delay = CorePeripherals::take().unwrap().SYST.delay(&clocks);
    // or let delay = dp.TIM2.delay_us(&clocks);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    adc::{Adc, CommonAdc, config::Config},
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    delay::Delay,
    pac::{CorePeripherals, Peripherals, ADC1,},
    prelude::*,
};

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f3xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32f3xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, Delay) {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

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

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb), clocks, rcc.apb1);
    let led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
    let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    //  1 second delay (for DHT11 setup?) Wait on  sensor initialization?
    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    adc::{config::AdcConfig, Adc}, //SampleTime
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    timer::SysDelay,
    pac::{CorePeripherals, Peripherals, ADC1,},
    prelude::*,
};

#[cfg(feature = "stm32f4xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f4xx")] // Use HSE oscillator
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32f4xx")]
pub type DelayType = SysDelay;

#[cfg(feature = "stm32f4xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, DelayType) {
                //(SensorType, I2c<I2C1, impl Pins<I2C1>>, LedType, DelayType) {
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
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

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)
    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

    let led = setup_led(dp.GPIOC.split());
    //let delay = Delay::new(CorePeripherals::take().unwrap().SYST, &clocks);
    let cp = CorePeripherals::take().unwrap();
    let mut delay = cp.SYST.delay(&clocks);
    
    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    adc::Adc,
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    timer::SysDelay,
    pac::{CorePeripherals, Peripherals, ADC1, },
    prelude::*,
};

#[cfg(feature = "stm32f7xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32f7xx")] // Use HSE oscillator
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32f7xx")]
pub type DelayType = SysDelay;

#[cfg(feature = "stm32f7xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, DelayType) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split();
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

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);
    let led = setup_led(dp.GPIOC.split());
    //let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);
    let cp = CorePeripherals::take().unwrap();
    let mut delay = cp.SYST.delay(&clocks);

    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    adc,
    adc::{Adc, Enabled, },
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    delay::Delay,
    pac::{CorePeripherals, Peripherals, ADC1,},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1, Enabled>>;

#[cfg(feature = "stm32h7xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32h7xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, Delay) {
                //(SensorType, I2c<I2C1>, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);
    
    let mut adc1 = Adc::adc1(dp.ADC1, &mut delay, ccdr.peripheral.ADC12, &ccdr.clocks);
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
    let i2cx = ccdr.peripheral.I2C1;

    let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
    let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));

    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    adc::{Adc, Ready},
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    delay::Delay,
//    gpio::{OpenDrain, Output, PushPull,
//           gpiob::{PB8, PB9, Parts as PartsB},
//           gpioc::{PC13, Parts as PartsC},
//    },
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, ADC, I2C1},  //ADC1 ?
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
};

#[cfg(feature = "stm32l0xx")]
type SensorType = Sensor<PA1<Analog>, Adc<Ready>>;
//type SensorType = Sensor<PA1<Analog>, Adc<ADC>>;

#[cfg(feature = "stm32l0xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32l0xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let clocks = rcc.clocks;

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


    let gpiob =dp.GPIOB.split(&mut rcc);
    let i2c = setup_i2c1(dp.I2C1, gpiob, rcc, &clocks);
    let led = setup_led(dp.GPIOC.split(&mut rcc));
    let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    adc::{Adc, Precision},
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    delay::Delay,
    prelude::*,
    rcc, // for ::Config but avoid name conflict with serial
    stm32::{CorePeripherals, Peripherals,},
};

#[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
type SensorType = Sensor<PA1<Analog>, Adc>;

#[cfg(feature = "stm32l1xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32l1xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, Delay) {
                //(SensorType, I2c<I2C1, impl Pins<I2C1>>, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    let clocks = rcc.clocks;

    let mut adc = dp.ADC.adc(&mut rcc);
    adc.set_precision(Precision::B_12);

    let gpioa = dp.GPIOA.split(&mut rcc);
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

    let led = setup_led(dp.GPIOC.split(&mut rcc).pc9);
    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);
    let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}



#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    adc::{Adc, AdcCommon},
    gpio::{Analog, Output, OpenDrain,
           gpioa::{PA1, PA8},
    },
    delay::Delay,
    pac::{CorePeripherals, Peripherals, ADC1},
    prelude::*,
};

#[cfg(feature = "stm32l4xx")]
type SensorType = Sensor<PA1<Analog>, Adc<ADC1>>;

#[cfg(feature = "stm32l4xx")]
type DhtType = PA8<Output<OpenDrain>>;

#[cfg(feature = "stm32l4xx")]
pub fn setup() -> (SensorType, DhtType, I2c1Type, LedType, Delay) {
    let dp = Peripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc.cfgr.sysclk(80.MHz()).pclk1(80.MHz()).pclk2(80.MHz()).freeze(&mut flash.acr, &mut pwr);

    let mut delay = Delay::new(CorePeripherals::take().unwrap().SYST, clocks);

    let adc_common = AdcCommon::new(dp.ADC_COMMON, &mut rcc.ahb2);
    let adc = Adc::adc1(dp.ADC1, adc_common, &mut rcc.ccipr, &mut delay );
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
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

    let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
    let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));

    delay.delay_ms(1000_u16);

    (sens, dht, i2c, led, delay)
}

// End of HAL/MCU specific setup.
