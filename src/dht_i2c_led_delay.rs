
#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use crate::dp::{Peripherals};
pub use crate::delay::{DelayType};
pub use crate::led::{setup_led, LED, LedType};

    #[cfg(not(feature = "stm32f0xx"))]
    pub use crate::i2c::{setup_i2c1, I2c1Type as I2cType};
    #[cfg(feature = "stm32f0xx")]
    pub use crate::i2c::{setup_i2c2, I2c2Type as I2cType};

    pub use embedded_hal::blocking::delay::DelayUs;
    pub use embedded_hal::prelude::_embedded_hal_blocking_delay_DelayMs as DelayMs;

    // A delay is used in sensor (dht) initialization and read. 
    // Systick is used by monotonic (for spawn), so delay needs to use a timer other than Systick
    // asm::delay used in AltDelay is not an accurate timer but gives a delay at least 
    //  number of indicated clock cycles.


pub fn setup() ->  (DhtPin, I2cType, LedType, DelayType) {    
   let dp = Peripherals::take().unwrap();
   let (dht, i2c, led,delay) = setup_dp(dp);

   (dht, i2c, led, delay)
}



    #[cfg(feature = "stm32f0xx")]
    use stm32f0xx_hal::{
        delay::Delay,
        gpio::{gpioa::PA8, OpenDrain, Output},
        pac::{CorePeripherals},
        prelude::*,
    };
 
    #[cfg(feature = "stm32f0xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f0xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32f0xx")]
    pub fn setup_dp(mut dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {    
       let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);
       let gpioa = dp.GPIOA.split(&mut rcc);
       let mut dht = cortex_m::interrupt::free(move |cs| gpioa.pa8.into_open_drain_output(cs));
       dht.set_high().ok();

       let i2c = setup_i2c2(dp.I2C2, dp.GPIOB.split(&mut rcc),  &mut rcc);

       let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
       led.off();
       
       //let delay = DelayType{};
       let cp = CorePeripherals::take().unwrap();
       let delay = Delay::new(cp.SYST, &rcc);
       //let delay = cp.SYST.delay(&rcc);
       //let delay = dp.TIM3.delay_us(&rcc);

       (dht, i2c, led, delay)
    }



    #[cfg(feature = "stm32f1xx")]
    use stm32f1xx_hal::{
        gpio::{Output, gpioa::PA8, OpenDrain},   //, gpioc::PC13, PushPull, State},
        prelude::*,
    };

    #[cfg(feature = "stm32f1xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f1xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32f1xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let mut gpioa = dp.GPIOA.split();
       let dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);

       let rcc = dp.RCC.constrain();
       let mut afio = dp.AFIO.constrain();
       let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

        //hprintln!("hclk {:?}",   clocks.hclk()).unwrap();
        //hprintln!("sysclk {:?}", clocks.sysclk()).unwrap();
        //hprintln!("pclk1 {:?}",  clocks.pclk1()).unwrap();
        //hprintln!("pclk2 {:?}",  clocks.pclk2()).unwrap();
        //hprintln!("pclk1_tim {:?}", clocks.pclk1_tim()).unwrap();
        //hprintln!("pclk2_tim {:?}", clocks.pclk2_tim()).unwrap();
        //hprintln!("adcclk {:?}",    clocks.adcclk()).unwrap();
        //hprintln!("usbclk_valid {:?}", clocks.usbclk_valid()).unwrap(); not fo all MCUs


       //afio  needed for i2c1 (PB8, PB9) but not i2c2
       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &mut afio, &clocks);

       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       // This delay used for dht initialization and read cannot be systick which is used by spawn.
       //let delay = DelayType{};
       let delay = dp.TIM2.delay_us(&clocks);

       (dht, i2c, led, delay)
       }



    #[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
    use stm32f3xx_hal::{
        gpio::{Output, OpenDrain,
               gpioa::{PA8,}, 
        },
        prelude::*,
    };

    #[cfg(feature = "stm32f3xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f3xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32f3xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);
    
       let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
       let dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

       // setup_i2c2 does not work. There is a "value used here after partial move" problem with I2C2 on gpioa
       //    because gpioa is used above for dht. And the only option for I2C2 seems to be gpioa.
       //    gpioa can be used without setup_i2c2 using
       // let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
       // let scl =  gpioa.pa9.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
       // let sda = gpioa.pa10.into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
       //    // //NOT sure if pull up is needed
       //    scl.internal_pull_up(&mut gpiob.pupdr, true);
       //    sda.internal_pull_up(&mut gpiob.pupdr, true);
       // let i2c = I2c::new(dp.I2C2, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);

       let gpiob = dp.GPIOB.split(&mut rcc.ahb);
       let i2c = setup_i2c1(dp.I2C1, gpiob, clocks, rcc.apb1);

       let mut led = setup_led(dp.GPIOE.split(&mut rcc.ahb));
       led.off();

       let delay = DelayType{};
       //let delay = dp.TIM2.delay_us(&clocks);
       //let mut delay = Delay::new(dp.TIM2, clocks);

       (dht, i2c, led, delay)
    }



    #[cfg(feature = "stm32f4xx")]
    use stm32f4xx_hal::{
        gpio::{Output, gpioa::PA8, OpenDrain}, 
        prelude::*,
    };

    #[cfg(feature = "stm32f4xx")]
    pub const MONOCLOCK: u32 = 16_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f4xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32f4xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let gpioa = dp.GPIOA.split();
       let dht = gpioa.pa8.into_open_drain_output();

       let rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks);

       let mut led = setup_led(dp.GPIOC.split()); 
       led.off();

       //let delay = DelayType{};
       let delay = dp.TIM2.delay_us(&clocks);

       (dht, i2c, led, delay)
   }



    #[cfg(feature = "stm32f7xx")]
    use stm32f7xx_hal::{
        gpio::{gpioa::PA8, Output, OpenDrain},
        prelude::*,
    };

    #[cfg(feature = "stm32f7xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32f7xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32f7xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let dht = dp.GPIOA.split().pa8.into_open_drain_output();

       let mut rcc = dp.RCC.constrain();
       let clocks = rcc.cfgr.freeze();
       //let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(), &clocks, &mut rcc.apb1);

       let led = setup_led(dp.GPIOC.split());
       let delay = DelayType{};
       //let delay = dp.TIM2.delay_us(&clocks);

       (dht, i2c, led, delay)
    }



    #[cfg(feature = "stm32h7xx")]
    use stm32h7xx_hal::{
        gpio::{Output, OpenDrain, 
               gpioa::PA8,
        },
        prelude::*,
    };

    #[cfg(feature = "stm32h7xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32h7xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32h7xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let pwr = dp.PWR.constrain();
       let vos = pwr.freeze();
       let rcc = dp.RCC.constrain();
       let ccdr = rcc.sys_ck(100.MHz()).freeze(vos, &dp.SYSCFG); // calibrate for correct blink rate
       let clocks = ccdr.clocks;

       let dht = dp.GPIOA.split(ccdr.peripheral.GPIOA).pa8.into_open_drain_output();

       let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
       let i2cx = ccdr.peripheral.I2C1;

       let i2c = setup_i2c1(dp.I2C1, gpiob, i2cx, &clocks);
       let led = setup_led(dp.GPIOC.split(ccdr.peripheral.GPIOC));
       let delay = DelayType{};
       //let delay = dp.TIM2.delay_us(&clocks);

       (dht, i2c, led, delay)
    }



    #[cfg(feature = "stm32l0xx")]
    use stm32l0xx_hal::{
        gpio::{gpioc::PC13, Output, PushPull},
        prelude::*,
        rcc, // for ::Config but note name conflict with serial
    };

    #[cfg(feature = "stm32l0xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l0xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32l0xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       // UNTESTED
       let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
       let clocks = rcc.clocks;

       let mut dht = dp.GPIOA.split(&mut rcc).pa8.into_open_drain_output();
 
       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), dp.AFIO.constrain(), &clocks);
       let led = setup_led(dp.GPIOC.split(&mut rcc));
       //let delay = DelayType{};
       let delay = dp.TIM2.delay_us(&clocks);

       (dht, i2c, led, delay)
    }



    #[cfg(feature = "stm32l1xx")] // eg  Discovery STM32L100 and Heltec lora_node STM32L151CCU6
    use stm32l1xx_hal::{
        gpio::{OpenDrain, Output,
               gpioa::PA8,
        },
        rcc, // for ::Config but note name conflict with serial
        prelude::*,
    };

    #[cfg(feature = "stm32l1xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l1xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32l1xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

       let dht = dp.GPIOA.split(&mut rcc).pa8.into_open_drain_output();

       let led = setup_led(dp.GPIOC.split(&mut rcc).pc9);
       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc), rcc);
       let delay = DelayType{};
       //let delay = dp.TIM2.delay_us(&rcc.clocks);

       (dht, i2c, led, delay)
    }



    #[cfg(feature = "stm32l4xx")]
    use stm32l4xx_hal::{
        gpio::{Output, OpenDrain,
               gpioa::PA8,
        },
        prelude::*,
    };

    #[cfg(feature = "stm32l4xx")]
    pub const MONOCLOCK: u32 = 8_000_000; //should be set for board not for HAL

    #[cfg(feature = "stm32l4xx")]
    pub type DhtPin = PA8<Output<OpenDrain>>;

    #[cfg(feature = "stm32l4xx")]
    pub fn setup_dp(dp: Peripherals) ->  (DhtPin, I2cType, LedType, DelayType) {
       let mut flash = dp.FLASH.constrain();
       let mut rcc = dp.RCC.constrain();
       let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
       let clocks = rcc.cfgr.sysclk(80.MHz()).pclk1(80.MHz()).pclk2(80.MHz()).freeze(&mut flash.acr, &mut pwr);

       let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
       let dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);

       let i2c = setup_i2c1(dp.I2C1, dp.GPIOB.split(&mut rcc.ahb2), &clocks, &mut rcc.apb1r1);
       let led = setup_led(dp.GPIOC.split(&mut rcc.ahb2));
       let delay = DelayType{};
       //let delay = dp.TIM2.delay_us(&clocks);

       (dht, i2c, led, delay)
    }


