//! Blink  onboard LED on PC13. Using  TIM2 delay and hsi, which is 16MHz.
//! The clock here should be the same as blink_default, but hsi is explicity set here.
//! From the datasheet DS10314 Rev 8, p20 
//! "The 16 MHz internal RC oscillator is factory-trimmed to offer 1% accuracy at 25C."

#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;

pub trait LED: OutputPin {
    // default methods
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
    let config = Config::hsi().pclk1(8.MHz());  // hsi() is 16.MHz()

    // RCC configuration is done here. Freeze before using `rcc`. 
    let mut rcc = dp.RCC.freeze(config);

    let gpioc = dp.GPIOC.split(&mut rcc);

    assert_eq!(rcc.clocks.sysclk(),  Hertz::MHz(16));
    assert_eq!(rcc.clocks.hclk(),    Hertz::MHz(16));
    assert_eq!(rcc.clocks.pclk1(),   Hertz::MHz(8));
    assert_eq!(rcc.clocks.pclk2(),   Hertz::MHz(16));
    //assert_eq!(rcc.clocks.hsi,     Hertz::MHz(16)); // no query method for hsi

    let mut delay = dp.TIM2.delay::<16000000>(&mut rcc);       //Tick at 32Mhz

    delay.delay(20.millis());
    delay.delay_ms(20);

    // output MCO
//    This does not yet seem to be implemented in stm32f4xx_hal, but see stm32g4xx_hal
//    let gpioa = dp.GPIOA.split(&mut rcc);
//    rcc.cfgr().mco1(Mco1Clock::HSI);      // Set MCO1 to HSI,  options HSI, HSE, PLL
//    let mco = gpioa.pa8.into_alternate(); // Set PA8 as alternate function    
//    rcc.mco1.set_mco1(mco);               // Enable clock output on mco pin PA8

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

    led.blink(1000, &mut delay); // blink  to indicate setup complete.

    led.off();    
    delay.delay_ms(5000);  //off for  5 s

    led.on();     
    delay.delay_ms(20000);   // on for 20s to check delay timer

    led.off();    
    delay.delay_ms(5000);  //off for  5 s

    let on: u32  = 10; // on for 10 ms
    let off: u32 = 990; //off for remainder of second

    // blink LED and sleep
    loop {
        led.blink(on, &mut delay);
        delay.delay_ms(off);
    }
}
