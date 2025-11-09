//! Blink  onboard LED on PC13.  Using high speed external (hse) oscillator.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
//use cortex_m_semihosting::hprintln;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

pub trait LED: OutputPin {
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }

    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }

    fn blink(&mut self, time: u32, delay: &mut impl DelayNs) -> () {
        self.on();
        delay.delay_ms(time);
        self.off()
    }

}

use stm32f4xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    pac::{Peripherals},
    rcc::Config,
    prelude::*,
    time::Hertz,
};

impl LED for PC13<Output<PushPull>>{}

#[cfg(feature = "stm32f411")]
fn setup() -> (impl LED, impl DelayNs) {
     let dp = Peripherals::take().unwrap();

    // Config creates a structure for passing to freeze, but does not itself set the clocks.    
    //let config = Config::hse(25.MHz());
    // or 
    let config = Config::hse(25.MHz()).hclk(100.MHz()).sysclk(100.MHz()).pclk1(50.MHz()).pclk2(50.MHz());

    // Note `pclk1` can not be bigger than `sysclk` 
    //Above works if timer tick is not at 1Khz.
    //To do timer tick  at 1Khz hclk must be less than 65 MHz.
    //let config = Config::hse(25.MHz()).hclk(50.MHz()).sysclk(100.MHz()).pclk1(50.MHz()).pclk2(50.MHz());

    // RCC configuration is done here. Freeze before using `rcc`. 
    let mut rcc = dp.RCC.freeze(config);

    //hprintln!("clocks() {:?}", rcc.clocks);

    // `Clocks` structure lives inside `Rcc`. Clone is possible: `rcc.clocks.clone()`
    //assert_eq!(rcc.clocks.hse(),     Hertz::MHz(25));
    assert_eq!(rcc.clocks.sysclk(),  Hertz::MHz(100)); 
    assert_eq!(rcc.clocks.hclk(),    Hertz::MHz(100));
    assert_eq!(rcc.clocks.pclk1(),   Hertz::MHz(50));
    assert_eq!(rcc.clocks.pclk2(),   Hertz::MHz(50));

    // Delay constructor requires Rcc to be already frozen. 
    let mut delay = dp.TIM2.delay_us(&mut rcc);              //Tick at 1Mhz
    //let mut delay = dp.TIM2.delay::<25000000>(&mut rcc);     //Tick at 25Mhz
    //let mut delay = dp.TIM2.delay::<50000000>(&mut rcc);       //Tick at 50Mhz
    //let mut delay = dp.TIM2.delay::<100000000>(&mut rcc);    //Tick at 100Mhz

    //  need  PCLK/65535 <= freq <= PCLK  
    //  Next would need  system frequency (hclk) less than 65 MHz or there will be panic with
    //       called `Result::unwrap()` on an `Err` value: TryFromIntError(())
    //let mut delay = dp.TIM2.delay_ms(&mut rcc);       //Tick at 1Khz

    let gpioc = dp.GPIOC.split(&mut rcc);
    
    // these both work here
    delay.delay(20.millis());
    delay.delay_ms(20);

    // return tuple  (led, delay)
    (
        gpioc.pc13.into_push_pull_output(), // led on pc13 with on/off
        delay,
    )
}


#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    time::{ExtU32},
    timer::{Timer},
    delay::DelayFromCountDownTimer,
    gpio::{gpioc::PC6, Output, PushPull},
    prelude::*,
    pac::{Peripherals},
    rcc::{Prescaler, MCOSrc},
};

// weact-stm32g474CEU6 has onboard led on PC6
#[cfg(feature = "stm32g4xx")]
    impl LED for PC6<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high()
        }
        fn off(&mut self) -> () {
            self.set_low()
        }
    }

#[cfg(feature = "stm32g4xx")]
pub fn setup() -> (impl LED, impl DelayNs) {
//pub fn setup() -> (PC6<Output<PushPull>>, impl DelayNs) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    
    let gpioc = dp.GPIOC.split(&mut rcc);
    let led = gpioc.pc6.into_push_pull_output();    
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    //  See  stm32g4xx-hal/src/rcc/clockout.rs
    // MCOSrc::{SysClk, HSI, HSE, PLL, LSI, LSE}
    //Prescaler::{NotDivided, Div2, Div4, Div8, Div16, Div32, Div64}
    //let pin = gpioa.pa8.into_alternate(); // Set PA8 as alternate function
    //let mco = pin.mco(MCOSrc::LSI, Prescaler::NotDivided, &mut rcc); 
    let mco = gpioa.pa8.mco(MCOSrc::SysClk, Prescaler::Div64, &mut rcc); // Also sets PA8 as alternate function

    // Enable clock output on mco pin
    mco.enable();
//SHOULD FREEZE HERE?

    (led, delay)
}



#[entry]
fn main() -> ! {
    let (mut led, mut delay) = setup();

    led.blink(1000, &mut delay); // blink 1s to indicate setup complete.

    led.off();    
    delay.delay_ms(2000);  //off for  2s
    //delay.delay(2000.millis());  //Does not work here, See note in blink_default.

    led.on();     
    delay.delay_ms(20000);   // on for 20s to check delay timer

    led.off();    
    delay.delay_ms(3000);  //off for  3s

    // blink once per second
    let on: u32  = 20; // on for 20 ms
    let off: u32 = 980; //off for remainder of second

    // blink LED and sleep
    // Note that blinking on blackpill is very dim and hard to see.
    // It is brighter on t16-f411.
    loop {
        led.blink(on, &mut delay);
        delay.delay_ms(off);
    }
}
