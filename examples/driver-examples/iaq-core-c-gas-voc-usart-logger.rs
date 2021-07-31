//!  THIS COMPILES on bluepill BUT I DON'T SEE HOW IT CAN WORK.
//!  Compare rtic/blink_rtic for handling of delay and consider other remarks in code below.
//!
//! Measures the CO2 and TVOC equivalents in the air with an iAQ-Core-C module,
//! logs the values and sends them through the serial interface every 10 seconds.
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1 and USART1.
//!
//! To setup the serial communication, have a look at the discovery book:
//! https://rust-embedded.github.io/discovery/10-serial-communication/index.html
//!
//! ```
//! BP   <-> iAQ-Core-C <-> Serial module
//! GND  <-> GND        <-> GND
//! 3.3V <-> VCC        <-> VDD
//! PB8  <-> SCL     
//! PB9  <-> SDA     
//! PB6                 <-> RX
//! ```
//!
//! Run with:
//! `cargo embed --example iaq-core-c-gas-voc-usart-logger-bp`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::fmt::Write;
use iaq_core::{IaqCore, Measurement};
use nb::block;
use panic_rtt_target as _;
use rtic::app;
use rtic::cyccnt::U32Ext;
use rtt_target::{rprintln, rtt_init_print};

const PERIOD: u32 = 1_000_000_000; // 10 seconds

pub trait LED {
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // implementation should deal with this difference
    fn on(&mut self) -> ();
    fn off(&mut self) -> ();

    // default methods
    fn blink(&mut self, time: u16, delay: &mut Delay) -> () {
        self.on();
        delay.delay_ms(time);
        self.off()
    }
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    device::USART1,
    gpio::{
        gpiob::{PB8, PB9},
        gpioc::PC13,
        Alternate, OpenDrain, Output, PushPull, State,
    },
    i2c::{BlockingI2c, Mode},
    pac,
    pac::Peripherals, //I2C1
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f1xx")]
type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f1xx")]
type I2cBus = BlockingI2c<pac::I2C1, (PB8<Alternate<OpenDrain>>, PB9<Alternate<OpenDrain>>)>;

#[cfg(feature = "stm32f1xx")]
fn setup(dp: Peripherals) -> (I2cBus, LedType, Delay, Tx<USART1>, Rx<USART1>) {
    //fn setup(dp: Peripherals) -> (BlockingI2c<I2C1, impl Pins<I2C1>>, impl LED, Delay, Tx<USART1>, Rx<USART1>) {

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();
    let mut afio = dp.AFIO.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .pclk1(36.mhz())
        .freeze(&mut flash.acr);

    // WHY DOES NEXT NOT CAUSE BULD PROBLEM WITH  let mut core = cx.core; IN INTI?
    let cp = cortex_m::Peripherals::take().unwrap();
    let delay = Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb9.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c1(
        dp.I2C1,
        (scl, sda),
        &mut afio.mapr,
        Mode::Standard {
            frequency: 100_000.hz(),
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );
    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;
    let serial = Serial::usart1(
        dp.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(115200.bps()),
        clocks,
    );
    let (tx, rx) = serial.split();

    let mut gpioc = dp.GPIOC.split();
    let mut led = gpioc
        .pc13
        .into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
    //let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    led.off();

    (i2c, led, delay, tx, rx)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB6, PB7},
        gpioe::PE9,
        Alternate, OpenDrain,
        Otyper::Otype,
        Output, PushPull, AF4,
    },
    i2c::{I2c, SclPin, SdaPin},
    pac,
    pac::{CorePeripherals, Peripherals, I2C1, USART1},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
type LedType = PE9<Output<PushPull>>;

#[cfg(feature = "stm32f3xx")]
type I2cBus = I2c<pac::I2C1, (PB6<Alternate<Otype, AF4>>, PB7<Alternate<Otype, AF4>>)>;
//type I2cBus = I2c<pac::I2C1, (PB6<Alternate<OpenDrain, <AF4>>>, PB7<Alternate<OpenDrain, <AF4>>>)>;
//type I2cBus = I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>;

#[cfg(feature = "stm32f3xx")]
fn setup(
    dp: Peripherals,
) -> (
    I2cBus,
    LedType,
    Delay,
    Tx<USART1, impl TxPin<USART1>>,
    Rx<USART1, impl RxPin<USART1>>,
) {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // WHY DOES NEXT NOT CAUSE BUILD PROBLEM WITH  let mut core = cx.core; IN INTI?
    let cp = cortex_m::Peripherals::take().unwrap();
    let delay = Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob
        .pb6
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let i2c = I2c::new(dp.I2C1, (scl, sda), 100_000.Hz(), clocks, &mut rcc.apb1);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let (tx, rx) = Serial::usart1(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        115200.Bd(),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);
    let led = gpioe
        .pe9
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);
    let delay = Delay::new(cp.SYST, clocks);

    impl LED for PE9<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }
    led.off();

    (i2c, led, delay, tx, rx)
}

#[cfg(feature = "stm32f4xx")]
use stm32f4xx_hal::{
    delay::Delay,
    gpio::{
        gpiob::{PB8, PB9},
        gpioc::PC13,
        AlternateOD, Output, PushPull, AF4,
    },
    i2c::I2c, //Pins Mode
    pac,
    pac::{Peripherals, USART1}, //I2C1
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f4xx")]
type LedType = PC13<Output<PushPull>>;

#[cfg(feature = "stm32f4xx")]
type I2cBus = I2c<pac::I2C1, (PB8<AlternateOD<AF4>>, PB9<AlternateOD<AF4>>)>; //NO BlockingI2c

#[cfg(feature = "stm32f4xx")]
fn setup(dp: Peripherals) -> (I2cBus, LedType, Delay, Tx<USART1>, Rx<USART1>) {
    //fn setup(dp: Peripherals) -> (BlockingI2c<I2C1, impl Pins<I2C1>>, impl LED, Delay, Tx<USART1>, Rx<USART1>) {
    //fn setup(dp: Peripherals) -> (I2c<I2C2, impl Pins<I2C2>, LedType, Delay, Tx<USART1>, Rx<USART1>> {

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // WHY DOES NEXT NOT CAUSE BUILD PROBLEM WITH  let mut core = cx.core; IN INTI?
    let cp = cortex_m::Peripherals::take().unwrap();
    let delay = Delay::new(cp.SYST, &clocks);

    let gpiob = dp.GPIOB.split();

    let scl = gpiob.pb8.into_alternate().set_open_drain();
    let sda = gpiob.pb9.into_alternate().set_open_drain();

    let i2c = I2c::new(dp.I2C1, (scl, sda), 100.khz(), clocks);

    let gpioa = dp.GPIOA.split();
    let tx = gpioa.pa9.into_alternate();
    let rx = gpioa.pa10.into_alternate();
    let (tx, rx) = Serial::new(
        dp.USART1,
        (tx, rx),
        Config::default().baudrate(115200.bps()),
        clocks,
    )
    .unwrap()
    .split();

    let gpioc = dp.GPIOC.split();
    //let mut led = gpioc.pc13.into_push_pull_output_with_state(&mut gpioc.crh, State::Low);
    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }
    //led.off();   NEED TO FIX

    (i2c, led, delay, tx, rx)
}

//  NEED TO SPECIFY DEVICE HERE FOR DIFFERENT HALs

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
//#[app(device = stm32f3xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
//#[app(device = stm32f4xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]

const APP: () = {
    struct Resources {
        led: LedType,
        sensor: IaqCore<I2cBus>,
        tx: Tx<pac::USART1>,
    }

    #[init(schedule = [measure])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
        rprintln!("iAQ-Core-C example");

        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        //let device: stm32f1xx_hal::stm32::Peripherals = cx.device;
        let device: Peripherals = cx.device;

        let (i2c, mut led, _delay, mut tx, _rx) = setup(device);

        led.off();

        cx.schedule.measure(cx.start + PERIOD.cycles()).unwrap();

        let sensor = IaqCore::new(i2c);

        writeln!(tx, "start\r",).unwrap();
        init::LateResources { led, sensor, tx }
    }

    #[task(resources = [led, sensor, tx], schedule = [measure])]
    fn measure(cx: measure::Context) {
        // Use the safe local `static mut` of RTIC
        static mut LED_STATE: bool = false;
        static mut MEASUREMENTS: [Measurement; 2400] = [Measurement {
            co2: 0,
            tvoc: 0,
            resistance: 0,
        }; 2400];
        static mut INDEX: usize = 0;

        if *LED_STATE {
            cx.resources.led.set_high();
            *LED_STATE = false;
        } else {
            cx.resources.led.set_low();
            *LED_STATE = true;
        }

        let default = Measurement {
            co2: 1,
            tvoc: 1,
            resistance: 1,
        };
        if *INDEX < MEASUREMENTS.len() {
            let data = block!(cx.resources.sensor.data()).unwrap_or(default);
            MEASUREMENTS[*INDEX] = data;
            *INDEX += 1;
        }
        for i in 0..*INDEX {
            let data = MEASUREMENTS[i];
            writeln!(
                cx.resources.tx,
                "{},{},{},{}\r",
                i, data.co2, data.tvoc, data.resistance
            )
            .unwrap();
        }
        cx.schedule.measure(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
