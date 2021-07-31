//!  THIS COMPILES on bluepill BUT I DON'T SEE HOW IT CAN WORK.
//!  Compare rtic/blink_rtic for handling of delay and consider other remarks in code below.
//!
//! Continuously measure the eCO2 and eTVOC in the air, logs the values and sends
//! them through the serial interface every 10 seconds.
//! In order to compensate for the ambient temperature and humidity, an HDC2080
//! sensor is used.
//!
//! Introductory blog post with some pictures here:
//! https://blog.eldruin.com/ccs811-indoor-air-quality-sensor-driver-in-rust/
//!
//! This example is runs on the STM32F103 "Bluepill" board using I2C1 and USART1.
//!
//! To setup the serial communication, have a look at the discovery book:
//! https://rust-embedded.github.io/discovery/10-serial-communication/index.html
//!
//! ```
//! BP   <-> CCS811 <-> HDC2080 <-> Serial module
//! GND  <-> GND    <-> GND     <-> GND
//! 3.3V <-> VCC    <-> VCC     <-> VDD
//! PB8  <-> SCL    <-> SCL      
//! PB9  <-> SDA    <-> SDA      
//! PB6             <-> RX
//! GND  <-> nWAKE
//! 3.3V <-> RST
//! ```
//!
//! Run with:
//! `cargo embed --example ccs811-gas-voc-usart-logger`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use core::fmt::Write;
use embedded_ccs811::{
    mode as Ccs811Mode, prelude::*, AlgorithmResult, Ccs811Awake, MeasurementMode,
    SlaveAddr as Ccs811SlaveAddr,
};
use hdc20xx::{mode as Hdc20xxMode, Hdc20xx, SlaveAddr as Hdc20xxSlaveAddr};
use nb::block;
use panic_rtt_target as _;
use rtic::app;
use rtic::cyccnt::U32Ext;
use rtt_target::{rprintln, rtt_init_print};
use shared_bus_rtic::SharedBus;

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

const PERIOD: u32 = 1_000_000_000; // 10 seconds

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    device::USART1,
    gpio::{
        gpiob::{PB8, PB9},
        gpioc::PC13,
        Alternate, OpenDrain, Output, PushPull, State,
    },
    i2c::{BlockingI2c, Mode}, //Pins
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

/*
 * shared-bus-rtic aggregate: multiple peripherals on a single i2c bus
 *
 * According to shared-bus-rtic docs:
 * Note that all of the drivers that use the same underlying bus **must** be stored within a single
 * resource (e.g. as one larger `struct`) within the RTIC resources. This ensures that RTIC will
 * prevent one driver from interrupting another while they are using the same underlying bus.
 */

pub struct I2cDevs {
    ccs811: Ccs811Awake<SharedBus<I2cBus>, Ccs811Mode::App>,
    hdc2080: Hdc20xx<SharedBus<I2cBus>, Hdc20xxMode::OneShot>,
}

//  NEED TO SPECIFY DEVICE HERE FOR DIFFERENT HALs

#[app(device = stm32f1xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
//#[app(device = stm32f3xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
//#[app(device = stm32f4xx_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]

const APP: () = {
    struct Resources {
        led: LedType,
        i2c: I2cDevs,
        tx: Tx<pac::USART1>,
    }

    #[init(schedule = [measure])]
    fn init(cx: init::Context) -> init::LateResources {
        rtt_init_print!();
        rprintln!("CCS811/HDC2080 example");

        let mut core = cx.core;
        core.DWT.enable_cycle_counter();

        let device: Peripherals = cx.device;

        let (i2c, mut led, mut delay, mut tx, _rx) = setup(device);

        led.off();

        let manager = shared_bus_rtic::new!(i2c, I2cBus);
        let mut hdc2080 = Hdc20xx::new(manager.acquire(), Hdc20xxSlaveAddr::default());
        let mut ccs811 = Ccs811Awake::new(manager.acquire(), Ccs811SlaveAddr::default());
        ccs811.software_reset().unwrap();
        delay.delay_ms(10_u16);

        let mut ccs811 = ccs811.start_application().ok().unwrap();
        let env = block!(hdc2080.read()).unwrap();
        ccs811
            .set_environment(env.temperature, env.humidity.unwrap_or(0.0))
            .unwrap();
        ccs811.set_mode(MeasurementMode::ConstantPower1s).unwrap();

        cx.schedule.measure(cx.start + PERIOD.cycles()).unwrap();

        writeln!(tx, "start\r",).unwrap();
        init::LateResources {
            led,
            i2c: I2cDevs { ccs811, hdc2080 },
            tx,
        }
    }

    #[task(resources = [led, i2c, tx], schedule = [measure])]
    fn measure(cx: measure::Context) {
        // Use the safe local `static mut` of RTIC
        static mut LED_STATE: bool = false;
        static mut ENV: [(f32, f32); 1200] = [(0.0, 0.0); 1200];
        static mut MEASUREMENTS: [AlgorithmResult; 1200] = [AlgorithmResult {
            eco2: 0,
            etvoc: 0,
            raw_current: 0,
            raw_voltage: 0,
        }; 1200];
        static mut INDEX: usize = 0;

        if *LED_STATE {
            cx.resources.led.set_high();
            *LED_STATE = false;
        } else {
            cx.resources.led.set_low();
            *LED_STATE = true;
        }

        let default = AlgorithmResult::default();
        if *INDEX < MEASUREMENTS.len() {
            let data = block!(cx.resources.i2c.ccs811.data()).unwrap_or(default);
            MEASUREMENTS[*INDEX] = data;
            let env = block!(cx.resources.i2c.hdc2080.read()).unwrap();
            let temp = env.temperature;
            let humidity = env.humidity.unwrap_or(0.0);
            ENV[*INDEX] = (temp, humidity);
            *INDEX += 1;
            cx.resources
                .i2c
                .ccs811
                .set_environment(temp, humidity)
                .unwrap();
        }
        writeln!(cx.resources.tx, "\rstart\r",).unwrap();
        for i in 0..*INDEX {
            let data = MEASUREMENTS[i];
            let env = if i == 0 { (0.0, 0.0) } else { ENV[i - 1] };
            writeln!(
                cx.resources.tx,
                "{},{},{},{},{},{:.2},{:.2}\r",
                i, data.eco2, data.etvoc, data.raw_current, data.raw_voltage, env.0, env.1
            )
            .unwrap();
        }
        cx.schedule.measure(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    extern "C" {
        fn EXTI0();
    }
};
