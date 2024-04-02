//    //////////////////////////////////////////
//        f0, f1, f4,       stm32g4
//   sck     on PA5
//  miso     on PA6
//  mosi     on PA7

//  CsPin    on PA1
//  BusyPin  on PB8 DIO0
//  ReadyPin on PB9 DIO1
//  ResetPin on PA0

//  tx  GPS  on PA2   to  GPS rx
//  rx  GPS  on PA3  from GPS tx

//  scl      on PB10           PA13
//  sda      on PB11           PA14

//  led      on PC13

//    //////////////////////////////////////////

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

pub use embedded_hal::delay::DelayNs;
//use crate::delay::{Delay2Type as Delay};

use embedded_hal::i2c::I2c as I2cTrait;
//use embedded_hal_async::i2c::I2c as I2cTrait;

//use crate::led::{setup_led, LED, LedType};

pub type LedType = LEDPIN<Output<PushPull>>;

pub use embedded_hal::digital::OutputPin;

pub trait LED: OutputPin {  // see The Rust Programming Language, section 19, Using Supertraits...
    // This does not have method blink which uses a delay.
    // depending on board wiring, on may be set_high or set_low, with off also reversed
    // A default of set_low() for on is defined here, but implementation should deal with a difference
    fn on(&mut self) -> () {
        self.set_low().unwrap()
    }
    fn off(&mut self) -> () {
        self.set_high().unwrap()
    }
}

//use crate::i2c::{setup_i2c1, I2c1Type as I2cType,};

//use crate::delay::{Delay2Type as Delay};


pub use crate::stm32xxx_as_hal::hal;
pub use hal::{
      pac::{Peripherals, CorePeripherals, USART1},
      serial::{Serial, Tx, Error},
      gpio::{gpioa::PA8, Output, OpenDrain},
      //prelude::*,
};



// MODE needs the old version as it is passed to the device hal crates
use embedded_hal::spi::{Mode, Phase, Polarity};

use radio_sx127x::{
    base::Base,
    device::lora::{
        Bandwidth, CodingRate, FrequencyHopping, LoRaChannel, LoRaConfig, PayloadCrc,
        PayloadLength, SpreadingFactor,
    },
    device::{Channel, Modem, PaConfig, PaSelect},
    //Error as sx127xError, // Error name conflict with hals
    prelude::*, // prelude has Sx127x,
};

// trait needs to be in scope to find  methods start_transmit and check_transmit.
pub use radio::{Receive, Transmit};

// lora and radio parameters

pub const MODE: Mode = Mode {
    //  SPI mode for radio
    phase: Phase::CaptureOnSecondTransition,
    polarity: Polarity::IdleHigh,
};

pub const FREQUENCY: u32 = 907_400_000; // frequency in hertz ch_12: 915_000_000, ch_2: 907_400_000

pub const CONFIG_CH: LoRaChannel = LoRaChannel {
    freq: FREQUENCY as u32, // frequency in hertz
    bw: Bandwidth::Bw125kHz,
    sf: SpreadingFactor::Sf7,
    cr: CodingRate::Cr4_8,
};

pub const CONFIG_LORA: LoRaConfig = LoRaConfig {
    preamble_len: 0x8,
    symbol_timeout: 0x64,
    payload_len: PayloadLength::Variable,
    payload_crc: PayloadCrc::Enabled,
    frequency_hop: FrequencyHopping::Disabled,
    invert_iq: false,
};

//   compare other settings in python version
//    lora.set_mode(sx127x_lora::RadioMode::Stdby).unwrap();
//    set_tx_power(level, output_pin) level >17 => PA_BOOST.
//    lora.set_tx_power(17,1).unwrap();
//    lora.set_tx_power(15,1).unwrap();

//baud = 1000000 is this needed for spi or just USART ?

pub const CONFIG_PA: PaConfig = PaConfig {
    output: PaSelect::Boost,
    power: 10,
};

//let CONFIG_RADIO = Config::default() ;

pub const CONFIG_RADIO: radio_sx127x::device::Config = radio_sx127x::device::Config {
    modem: Modem::LoRa(CONFIG_LORA),
    channel: Channel::LoRa(CONFIG_CH),
    pa_config: CONFIG_PA,
    xtal_freq: 32000000, // CHECK
    timeout_ms: 100,
};


//  /////////////// setup() does all  hal/MCU specific setup ///////////////////////////

pub fn setup() -> (LoraSpiType,  TxType, RxType, impl I2cTrait, LedType) {
    let dp = Peripherals::take().unwrap();
    setup_from_dp(dp)
}


//  ///////////////////////////////////////////////////////////////////

#[cfg(feature = "stm32f030xc")]
use stm32f0xx_hal::pac::I2C2 as I2C;
#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc, stm32f042
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, USART2},
    prelude::*,
    serial::{Rx, Serial, Tx},
    spi::{Error, Spi},
};

#[cfg(feature = "stm32f042")]
use stm32f0xx_hal::pac::I2C1 as I2C;

#[cfg(feature = "stm32f042")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32f042")]
pub type RxType = Rx<USART2>;

#[cfg(feature = "stm32f0xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    impl DelayMs<u32>
        + Transmit<Error = sx127xError<Error, Infallible, Infallible>>
        + Receive<Info = PacketInfo, Error = sx127xError<Error, Infallible, Infallible>>,
    TxType, RxType,
    I2c<I2C, impl SclPin<I2C>, impl SdaPin<I2C>>,
    PC13<Output<PushPull>>,
) {
    //  Infallible, Infallible   reflect the error type on the spi and gpio traits.

    let cp = CorePeripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().freeze(&mut p.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let (sck, miso, mosi, _rst, pa1, pb8, pb9, pa0, tx, rx, scl, sda, led) =
        cortex_m::interrupt::free(move |cs| {
            (
                gpioa.pa5.into_alternate_af0(cs), //    sck     on PA5
                gpioa.pa6.into_alternate_af0(cs), //   miso     on PA6
                gpioa.pa7.into_alternate_af0(cs), //   mosi     on PA7
                //gpioa.pa1.into_push_pull_output(cs),  //  cs     on PA1
                gpiob.pb1.into_push_pull_output(cs), //   reset    on PB1
                gpioa.pa1.into_push_pull_output(cs), //   CsPin    on PA1
                gpiob.pb8.into_floating_input(cs),   //   BusyPin  on PB8 DIO0
                gpiob.pb9.into_floating_input(cs),   //   ReadyPin on PB9 DIO1
                gpioa.pa0.into_push_pull_output(cs), //   ResetPin on PA0
                gpioa.pa2.into_alternate_af1(cs),    //tx pa2  for GPS
                gpioa.pa3.into_alternate_af1(cs),    //rx pa3  for GPS
                gpiob.pb10.into_alternate_af1(cs),   // scl on PB10
                gpiob.pb11.into_alternate_af1(cs),   // sda on PB11
                gpioc.pc13.into_push_pull_output(cs), //led
            )
        });

    let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), MODE, 8.mhz(), &mut rcc);

    let delay = Delay::new(cp.SYST, &rcc);

    // Create lora radio instance

    let lora = Sx127x::spi(spi, pa1, pb8, pb9, pa0, delay, &CONFIG_RADIO, )
                          .unwrap(); // should handle error

    //  stm32f030xc builds with gpiob..into_alternate_af4(cs) USART3 on tx pb10, rx pb11
    //    but stm32f042  only has 2 usarts.
    //  Both have gpioa..into_alternate_af1(cs) USART2 with tx on pa2 and rx pa3

    // This is done for tx, rx above because move |cs| consumes gpioa
    // let (tx, rx) = cortex_m::interrupt::free(move |cs| {...});

    let (tx, rx) = Serial::usart2(dp.USART2, (tx, rx), 9600.bps(), &mut rcc).split();

    #[cfg(feature = "stm32f030xc")]
    let i2c = I2c::i2c2(dp.I2C2, (scl, sda), 400.khz(), &mut rcc);
    #[cfg(feature = "stm32f042")]
    let i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), &mut rcc);

    // led on pc13 with on/off  above
    impl LED for LedType {}  //using default method on = low  

    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    //delay::Delay,
    pac::{I2C2, USART2, SPI1, TIM2, TIM5},
    gpio::{gpioc::PC13 as LEDPIN, PushPull, Pin},
    i2c::{BlockingI2c, DutyCycle, Pins},
    spi::{Spi, Error as SpiError, Pins as SpiPins, Spi1NoRemap},
    serial::{Config, Rx}, //, StopBits
    timer::{TimerExt, Delay},
    prelude::*,
};


#[cfg(feature = "stm32f1xx")]
pub type LoraSpiType =     Sx127x<Base<Spi<SPI1, Spi1NoRemap, impl SpiPins<Spi1NoRemap>, u8>, 
                                  Pin<'A', 1, Output>, Pin<'B', 8>, Pin<'B', 9>, Pin<'A', 0, Output>, 
                                  impl DelayNs>>;
                                  //Delay<TIM5, 1000000>>>;

#[cfg(feature = "stm32f1xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32f1xx")]
pub type RxType = Rx<USART2>;


#[cfg(feature = "stm32f1xx")]
pub fn setup_from_dp(dp: Peripherals) -> (LoraSpiType, TxType, RxType, impl I2cTrait, LedType,) {
    let cp = CorePeripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut dp.FLASH.constrain().acr);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl), //   sck 
            gpioa.pa6.into_floating_input(&mut gpioa.crl),      //   miso
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl), //   mosi
        ),
        &mut afio.mapr,
        MODE,
        8.MHz(),
        clocks,
    );

    //let delay = Delay::new(cp.SYST, clocks);
    let mut delay = dp.TIM2.delay::<1000000_u32>(&clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi,                                             //Spi
        gpioa.pa1.into_push_pull_output(&mut gpioa.crl), //CsPin        
        gpiob.pb8.into_floating_input(&mut gpiob.crh),   //BusyPin  DIO0
        gpiob.pb9.into_floating_input(&mut gpiob.crh),   //ReadyPin DIO1
        gpioa.pa0.into_push_pull_output(&mut gpioa.crl), //ResetPin     
        delay,                                           //Delay
        &CONFIG_RADIO,                                             //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = Serial::new(
        dp.USART2,
        (
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl), //tx pa2  for GPS rx
            gpioa.pa3,                                          //rx pa3  for GPS tx
        ),
        &mut afio.mapr,
        Config::default().baudrate(9_600.bps()),
        &clocks,
    )
    .split();

    let i2c = BlockingI2c::i2c2(
        dp.I2C2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 but not i2c2
        stm32f1xx_hal::i2c::Mode::Fast {
            frequency: 400.kHz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh); // led on pc13 with on/off
    impl LED for LedType {}  //using default method on = low  


    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioe::PE15, Output, PushPull},
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C2, USART2},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
    spi::{Error, Spi},
};

#[cfg(feature = "stm32f3xx")]
pub type TxType = Tx<USART2, impl TxPin<USART2>>;

#[cfg(feature = "stm32f3xx")]
pub type RxType = Rx<USART2, impl RxPin<USART2>>;

#[cfg(feature = "stm32f3xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    impl DelayMs<u32>
        + Transmit<Error = sx127xError<Error, Infallible, Infallible>>
        + Receive<Info = PacketInfo, Error = sx127xError<Error, Infallible, Infallible>>,
    TxType, RxType,
    I2c<I2C2, (impl SclPin<I2C2>, impl SdaPin<I2C2>)>,
    PE15<Output<PushPull>>,
) {
    let cp = CorePeripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut dp.FLASH.constrain().acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    let mut gpioe = dp.GPIOE.split(&mut rcc.ahb);

    let spi = Spi::new(
        dp.SPI1,
        (
            gpioa
                .pa5
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
            gpioa
                .pa6
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // miso  on PA6
            gpioa
                .pa7
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // mosi  on PA7
        ),
        //MODE,
        8.MHz(),
        clocks,
        &mut rcc.apb2,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi, //Spi
        gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper), //CsPin   on PA1
        gpiob
            .pb8
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr), //BusyPin  DIO0 on PB8
        gpiob
            .pb9
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr), //ReadyPin DIO1 on PB9
        gpioa
            .pa0
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper), //ResetPin      on PA0
        delay, //Delay
        &CONFIG_RADIO, //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = Serial::new(
        dp.USART2,
        (
            gpioa
                .pa2
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2 for GPS rx
            gpioa
                .pa3
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3 for GPS tx
        ),
        9600.Bd(), // 115_200.bps(),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    let scl = gpioa
        .pa9
        .into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let sda = gpioa
        .pa10
        .into_af_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);

    let i2c = I2c::new(dp.I2C2, (scl, sda), 400_000.Hz(), clocks, &mut rcc.apb1);

    impl LED for PE15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    //led on pe15 with on/off
    let led = gpioe
        .pe15
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32f4xx")]  // eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{
    pac::{USART2, SPI1, TIM5},
    i2c::{I2c},
    serial::{config::Config, Rx},
    spi::{Spi},
    timer::{TimerExt, Delay},
    gpio::{GpioExt, Pin, PushPull}, 
    gpio::{gpioc::PC13 as LEDPIN},
    prelude::*,
};

// LoraSpiType   might be something like
// impl DelayNs
//    + Transmit<Error = sx127xError<Error>>
//    + Receive<Info = PacketInfo, Error = sx127xError<Error>>,

// If the type for the lora object is needed somewhere other than just in the setup() return type then it
// may be better to explicitly define it as follows.

#[cfg(feature = "stm32f4xx")]
pub type LoraSpiType =     Sx127x<Base<Spi<SPI1>, 
                                  Pin<'A', 1, Output>, Pin<'B', 8>, Pin<'B', 9>, Pin<'A', 0, Output>, 
                                  Delay<TIM5, 1000000>>>;

#[cfg(feature = "stm32f4xx")]
pub type TxType =  Tx<USART2>;
#[cfg(feature = "stm32f4xx")]
pub type RxType =  Rx<USART2>;

#[cfg(feature = "stm32f4xx")]
pub fn setup_lora_from_dp(dp: Peripherals) -> LoraSpiType {
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();

    let spi = Spi::new(
        dp.SPI1,
        (
            gpioa.pa5.into_alternate(), // sck  
            gpioa.pa6.into_alternate(), // miso 
            gpioa.pa7.into_alternate(), // mosi 
        ),
        MODE,
        8.MHz(),
        &clocks,
    );

    let delay = dp.TIM5.delay::<1000000_u32>(&clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         
        gpiob.pb8.into_floating_input(),   //BusyPin  DI00 
        gpiob.pb9.into_floating_input(),   //ReadyPin DI01 
        gpioa.pa0.into_push_pull_output(), //ResetPin      
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    ).unwrap(); // should handle error

    //DIO0  triggers RxDone/TxDone status.
    //DIO1  triggers RxTimeout and other errors status.
    //D02, D03 ?

    //lora.lora_configure( config_lora, &config_ch ).unwrap(); // not yet pub, need to change something?
    //let () =lora;

    lora
}

#[cfg(feature = "stm32f4xx")]
impl LED for LedType {}  //using default method on = low  

#[cfg(feature = "stm32f4xx")]
pub fn setup_led_from_dp(dp: Peripherals) -> LedType {
    let gpioc = dp.GPIOC.split();

    let mut led: LedType  = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off (note delay uused in lora)
    led.off();

    led
}

#[cfg(feature = "stm32f4xx")]
pub fn setup_i2c_from_dp(dp: Peripherals) -> impl I2cTrait {
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze();

    let gpiob = dp.GPIOB.split();

    let i2c = I2c::new(
                    dp.I2C2, 
                    (gpiob.pb10.into_alternate().set_open_drain(), // scl
                     gpiob.pb3.into_alternate().set_open_drain()   // sda
                    ), 
                    400.kHz(), 
                    &clocks);

    i2c
}

#[cfg(feature = "stm32f4xx")]
pub fn setup_from_dp(dp: Peripherals) -> (LoraSpiType, TxType, RxType, impl I2cTrait, LedType) {
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let spi = Spi::new(
        dp.SPI1,
        (
            gpioa.pa5.into_alternate(), // sck  
            gpioa.pa6.into_alternate(), // miso 
            gpioa.pa7.into_alternate(), // mosi 
        ),
        MODE,
        8.MHz(),
        &clocks,
    );

    //let delay = Delay::new(cp.SYST, &clocks);
    let delay = dp.TIM5.delay::<1000000_u32>(&clocks);

    // Create lora radio instance

    // open_drain_output is really input and output. BusyPin is just input.

    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         
        gpiob.pb8.into_floating_input(),   //BusyPin  DI00 
        gpiob.pb9.into_floating_input(),   //ReadyPin DI01 
        gpioa.pa0.into_push_pull_output(), //ResetPin      
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    ).unwrap(); // should handle error

    //DIO0  triggers RxDone/TxDone status.
    //DIO1  triggers RxTimeout and other errors status.
    //D02, D03 ?

    //lora.lora_configure( config_lora, &config_ch ).unwrap(); // not yet pub, need to change something?
    //let () =lora;

    let (tx, rx) = Serial::new(
        dp.USART2,
        (
            gpioa.pa2.into_alternate(), //tx  for GPS rx
            gpioa.pa3.into_alternate(), //rx  for GPS tx
        ),
        Config::default().baudrate(9600.bps()),
        &clocks,
    ).unwrap().split();

    let i2c = I2c::new(
                    dp.I2C2, 
                    (gpiob.pb10.into_alternate().set_open_drain(), // scl
                     gpiob.pb3.into_alternate().set_open_drain()   // sda
                    ), 
                    400.kHz(), 
                    &clocks);

    // Note that blackpill with stm32f411 and nucleo-64 with stm32f411 have onboard led wired
    // differently. Next will be reversed for nucleo-64 (in addition to PA5 vs PC13).

    let mut led: LedType  = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off (note delay uused in lora)
    led.off();

    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{BlockingI2c, PinScl, PinSda},
    pac::{CorePeripherals, Peripherals, I2C2, USART2},
    prelude::*,
    serial::{Config, Oversampling, Rx, Serial, Tx},
    spi::{ClockDivider, Error, Spi},
};

#[cfg(feature = "stm32f7xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32f7xx")]
pub type RxType = Rx<USART2>;

#[cfg(feature = "stm32f7xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    impl DelayMs<u32>
        + Transmit<Error = sx127xError<Error, Infallible, Infallible>>
        + Receive<Info = PacketInfo, Error = sx127xError<Error, Infallible, Infallible>>,
    TxType, RxType,
    BlockingI2c<I2C2, impl PinScl<I2C2>, impl PinSda<I2C2>>,
    PC13<Output<PushPull>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();

    let mut rcc = p.RCC.constrain();

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();
    let gpioc = p.GPIOC.split();

    let sck = gpioa.pa5.into_alternate(); // sck   on PA5
    let miso = gpioa.pa6.into_alternate(); // miso  on PA6
    let mosi = gpioa.pa7.into_alternate(); // mosi  on PA7

    //   somewhere 8.mhz needs to be set in spi

    let spi =
        Spi::new(p.SPI1, (sck, miso, mosi)).enable::<u8>(&mut rcc.apb2, ClockDivider::DIV32, MODE);

    //let clocks = rcc.cfgr.sysclk(216.mhz()).freeze();
    let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze();

    let delay = Delay::new(cp.SYST, clocks);

    // Create lora radio instance

    // spi::new  partially consumes rcc which causes problem for second use of clocks
    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         on PA1
        gpiob.pb8.into_floating_input(),   //BusyPin  DIO0 on PB8
        gpiob.pb9.into_floating_input(),   //ReadyPin DIO1 on PB9
        gpioa.pa0.into_push_pull_output(), //ResetPin      on PA0
        delay,                             //Delay
        &CONFIG_RADIO,                     //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = Serial::new(
        p.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2  for GPS
            gpioa.pa3.into_alternate(), //rx pa3  for GPS
        ),
        clocks,
        Config {
            baud_rate: 9600.Bps(),
            oversampling: Oversampling::By16,
            character_match: None,
        },
    )
    .split();

    let scl = gpiob.pb10.into_alternate_open_drain();
    let sda = gpiob.pb11.into_alternate_open_drain();

    let i2c = BlockingI2c::i2c2(
        p.I2C2,
        (scl, sda),
        stm32f7xx_hal::i2c::Mode::standard(400_000.Hz()),
        clocks,
        &mut rcc.apb1,
        1000,
    );

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let led = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off

    (lora, tx, rx, i2c, led)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c},
    pac::{Peripherals, I2C2, USART2, SPI1, TIM5},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
    spi::{Spi},
    timer::{TimerExt, Delay},
    gpio::{GpioExt, Pin, Output}, 
    serial::{FullConfig},
};

#[cfg(feature = "stm32g0xx")]
pub type LoraSpiType =     Sx127x<Base<Spi<SPI1>, 
                                  Pin<'A', 1, Output>, Pin<'B', 8>, Pin<'B', 9>, Pin<'A', 0, Output>, 
                                  Delay<TIM5, 1000000>>>;

#[cfg(feature = "stm32g0xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32g0xx")]
pub type RxType = Rx<USART2>;

#[cfg(feature = "stm32g0xx")]
pub fn setup_from_dp(dp: Peripherals) -> (LoraSpiType, TxType, RxType, I2c<I2C2>, PC13<Output<PushPull>>) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let spi = dp.SPI1.spi((gpioa.pa5, gpioa.pa6, gpioa.pa7), //sck, miso, mosi
        mcp4x::MODE, 
        3.MHz(), &mut rcc, );

    let mut cs = gpioa.pa1.into_push_pull_output();
    cs.set_high().unwrap();


//    let i2c = setup_i2c1(dp.I2C1, gpiob, &mut rcc);

    let mut led = setup_led(dp.GPIOC.split(&mut rcc)); 
    led.off();

    let tx = gpioa.pa9; 
    let rx = gpioa.pa10;
    let (tx, _rx) = dp.USART1.usart((tx, rx), FullConfig::default(), &mut rcc).unwrap().split();

    let delay = dp.TIM2.delay(&mut rcc);

    (lora, tx, rx, i2c, led)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    pac::{USART2, SPI1, TIM5},
    spi::{Spi, MODE_0, SpiExt},
    serial::{FullConfig, NoDMA, Rx, SerialExt},
    i2c::{Config, I2cExt}, //I2c in prelude?
    time::{ExtU32, RateExtU32},
    timer::{Timer, CountDownTimer},
    delay::DelayFromCountDownTimer,
    gpio::{Input, Alternate, Floating, PushPull, GpioExt,
           gpioa::{ PA0, PA1, PA2, PA3, PA5, PA6, PA7},
           gpiob::{  PB8, PB9} 
    },
    gpio::{gpioc::PC13 as LEDPIN},
    rcc::RccExt,
    prelude::*,
};


#[cfg(feature = "stm32g4xx")]
pub type LoraSpiType = Sx127x<Base<
                     Spi<SPI1,(PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>, 
                     PA1<Output<PushPull>>, PB8<Input<Floating>>, PB9<Input<Floating>>, PA0<Output<PushPull>>, 
                     DelayFromCountDownTimer<CountDownTimer<TIM5>>>>;

#[cfg(feature = "stm32g4xx")]
pub type TxType = Tx<USART2, PA2<Alternate<7_u8>>, NoDMA >;

#[cfg(feature = "stm32g4xx")]
pub type RxType = Rx<USART2, PA3<Alternate<7_u8>>, NoDMA >;

#[cfg(feature = "stm32g4xx")]
pub fn setup_lora_from_dp(dp: Peripherals) -> LoraSpiType {
    let mut rcc = dp.RCC.constrain();
    
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let spi = dp.SPI1.spi(
            (gpioa.pa5.into_alternate(), // sck 
             gpioa.pa6.into_alternate(), // miso
             gpioa.pa7.into_alternate(), // mosi
            ),
        MODE_0,
        400.kHz(),
        &mut rcc,
    );

    let clocks = rcc.clocks;  // not sure if this is right

    let timer2 = Timer::new(dp.TIM5, &clocks);
    let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         
        gpiob.pb8.into_floating_input(),   //BusyPin  DI00 
        gpiob.pb9.into_floating_input(),   //ReadyPin DI01 
        gpioa.pa0.into_push_pull_output(), //ResetPin      
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    ).unwrap(); // should handle error

    lora
}



#[cfg(feature = "stm32g4xx")]
pub fn setup_from_dp(dp: Peripherals) -> (LoraSpiType, TxType, RxType, impl I2cTrait, LedType) {
    let mut rcc = dp.RCC.constrain();
    
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let spi = dp.SPI1.spi(
            (gpioa.pa5.into_alternate(), // sck 
             gpioa.pa6.into_alternate(), // miso
             gpioa.pa7.into_alternate(), // mosi
            ),
        MODE_0,
        400.kHz(),
        &mut rcc,
    );

    let clocks = rcc.clocks;  // not sure if this is right

    //let delay = Delay::new(cp.SYST, &clocks);
    //let delay = dp.TIM5.delay::<1000000_u32>(&clocks);
    let timer2 = Timer::new(dp.TIM5, &clocks);
    let delay = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    // Create lora radio instance

    // open_drain_output is really input and output. BusyPin is just input, but I think this should work
    //            gpiob.pb8.into_alternate_open_drain(&mut gpiob.crh),
    // however, gives trait bound  ... InputPin` is not satisfied

    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         
        gpiob.pb8.into_floating_input(),   //BusyPin  DI00 
        gpiob.pb9.into_floating_input(),   //ReadyPin DI01 
        gpioa.pa0.into_push_pull_output(), //ResetPin      
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    ).unwrap(); // should handle error

    //let () = lora;

    let (tx, rx) = dp.USART2.usart(
                           gpioa.pa2.into_alternate(),    //tx  was pa9
                           gpioa.pa3.into_alternate(),   //rx  was pa10
                           FullConfig::default().baudrate(115200.bps()),
                           &mut rcc).unwrap().split();

    let i2c = dp.I2C1.i2c(
                       gpioa.pa14.into_alternate_open_drain(),  //sda
                       gpioa.pa13.into_alternate_open_drain(),  //scl
                       Config::new(400.kHz()), 
                       &mut rcc
    );

    let mut led: LedType = gpioc.pc13.into_push_pull_output();

    impl LED for LedType {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    led.off();
    
    (lora, tx, rx, i2c, led)
}


#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    pac::{USART2, SPI1},
    spi::{Spi, SpiExt, Enabled},
    serial::{Rx},
    delay::Delay,
    gpio::{Input, PushPull, GpioExt,
           gpioa::{ PA0, PA1},
           gpiob::{ PB8, PB9} 
    },
    gpio::{gpioc::PC13 as LEDPIN},
    prelude::*,
};

#[cfg(feature = "stm32h7xx")]
pub type LoraSpiType = Sx127x<Base<Spi<SPI1, Enabled>,
                     PA1<Output<PushPull>>, PB8<Input>, PB9<Input>, PA0<Output<PushPull>>, 
                     Delay>>;


//pub type LoraSpiType = Sx127x<Base<
//                     Spi<SPI1, Enabled, (PA5<Alternate<5>>, PA6<Alternate<5>>, PA7<Alternate<5>>)>, 
//                     PA1<Output<PushPull>>, PB8<Input>, PB9<Input>, PA0<Output<PushPull>>, 
//                     Delay>>;

#[cfg(feature = "stm32h7xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32h7xx")]
pub type RxType = Rx<USART2>;

#[cfg(feature = "stm32h7xx")]
pub fn setup_from_dp(dp: Peripherals) -> (LoraSpiType, TxType, RxType, impl I2cTrait, LedType) {
    let cp = CorePeripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    // following github.com/stm32-rs/stm32h7xx-hal/blob/master/examples/spi.rs
    let spi = dp.SPI1.spi(
        (
            gpioa.pa5.into_alternate(), // sck 
            gpioa.pa6.into_alternate(), // miso
            gpioa.pa7.into_alternate(), // mosi
        ),
        MODE,
        8.MHz(),
        ccdr.peripheral.SPI1,
        &clocks,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         on PA1
        gpiob.pb8.into_floating_input(),   //BusyPin  DIO0 on PB8
        gpiob.pb9.into_floating_input(),   //ReadyPin DIO1 on PB9
        gpioa.pa0.into_push_pull_output(), //ResetPin      on PA0
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = dp
        .USART2
        .serial(
            (
                gpioa.pa2.into_alternate(), //tx for GPS rx
                gpioa.pa3.into_alternate(), //rx for GPS tx
            ),
            9600.bps(),
            ccdr.peripheral.USART2,
            &clocks,
        )
        .unwrap()
        .split();

    let scl = gpiob.pb10.into_alternate().set_open_drain();
    let sda = gpiob.pb11.into_alternate().set_open_drain();

    let i2c = dp
        .I2C2
        .i2c((scl, sda), 400.kHz(), ccdr.peripheral.I2C2, &clocks);


    let led = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off
    impl LED for LedType {}  //using default method on = low  

    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{I2c, SCLPin, SDAPin},
    pac::{CorePeripherals, Peripherals, I2C2, USART2},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, Serial2Ext, Tx},
    spi::Error,
};

#[cfg(feature = "stm32l0xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32l0xx")]
pub type RxType = Rx<USART2>;

#[cfg(feature = "stm32l0xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    impl DelayMs<u32>
        + Transmit<Error = sx127xError<Error, void::Void, Infallible>>
        + Receive<Info = PacketInfo, Error = sx127xError<Error, void::Void, Infallible>>,
    TxType, RxType,
    I2c<I2C2, impl SDAPin<I2C2>, impl SCLPin<I2C2>>,
    PC13<Output<PushPull>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    // following  github.com/stm32-rs/stm32l0xx-hal/blob/master/examples/spi.rs
    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5
            gpioa.pa6, // miso  on PA6
            gpioa.pa7, // mosi  on PA7
        ),
        MODE,
        8_000_000.Hz(),
        &mut rcc,
    );

    let delay = cp.SYST.delay(rcc.clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa1.into_push_pull_output(), //CsPin         on PA1
        gpiob.pb8.into_floating_input(),   //BusyPin  DIO0 on PB8
        gpiob.pb9.into_floating_input(),   //ReadyPin DIO1 on PB9
        gpioa.pa0.into_push_pull_output(), //ResetPin      on PA0
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = dp
        .USART2
        .usart(
            gpioa.pa2, //tx pa2  for GPS
            gpioa.pa3, //rx pa3  for GPS
            Config::default().baudrate(9600.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let scl = gpiob.pb10.into_open_drain_output();
    let sda = gpiob.pb11.into_open_drain_output();

    let i2c = dp.I2C2.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let led = gpioc.pc13.into_push_pull_output(); // led on pc13 with on/off

    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    gpio::{gpiob::PB6, Output, PushPull},
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, SerialExt, Tx},
    spi::Error,
    stm32::{CorePeripherals, Peripherals, I2C1, USART1},
};

#[cfg(feature = "stm32l1xx")]
pub type TxType = Tx<USART1>;

#[cfg(feature = "stm32l1xx")]
pub type RxType = Rx<USART1>;



#[cfg(feature = "stm32l1xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    impl DelayMs<u32>
        + Transmit<Error = sx127xError<Error, Infallible, Infallible>>
        + Receive<Info = PacketInfo, Error = sx127xError<Error, Infallible, Infallible>>,
    TxType, RxType,
    I2c<I2C1, impl Pins<I2C1>>,
    PB6<Output<PushPull>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5  in board on Heltec
            gpioa.pa6, // miso  on PA6  in board on Heltec
            gpioa.pa7, // mosi  on PA7  in board on Heltec
        ),
        MODE,
        8.mhz(),
        &mut rcc,
    );

    let delay = cp.SYST.delay(rcc.clocks);

    // Create lora radio instance

    //  Heltec lora_node STM32L151CCU6
    let lora = Sx127x::spi(
        spi,                               //Spi
        gpioa.pa4.into_push_pull_output(), //CsPin         on PA4  in board on Heltec
        gpiob.pb11.into_floating_input(),  //BusyPin  DIO0 on PB11 in board on Heltec
        gpiob.pb10.into_floating_input(),  //ReadyPin DIO1 on PB10 in board on Heltec
        gpioa.pa3.into_push_pull_output(), //ResetPin      on PA3  in board on Heltec
        delay,                             //Delay
        &CONFIG_RADIO,                               //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = dp
        .USART1
        .usart(
            (
                gpioa.pa9,  //tx pa9   for GPS rx
                gpioa.pa10, //rx pa10  for GPS tx
            ),
            Config::default().baudrate(9600.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let scl = gpiob.pb8.into_open_drain_output();
    let sda = gpiob.pb9.into_open_drain_output();

    let i2c = dp.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

    impl LED for LEDPIN {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let led = gpiob.pb6.into_push_pull_output(); // led on pb6 with on/off

    (lora, tx, rx, i2c, led)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioc::PC13, Output, PushPull},
    i2c::{Config, I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    serial, //for Config
    serial::{Rx, Serial, Tx},
    spi::{Error, Spi},
};

#[cfg(feature = "stm32l4xx")]
pub type TxType = Tx<USART2>;

#[cfg(feature = "stm32l4xx")]
pub type RxType = Rx<USART2>;

#[cfg(feature = "stm32l4xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    impl DelayMs<u32>
        + Transmit<Error = sx127xError<Error, Infallible, Infallible>>
        + Receive<Info = PacketInfo, Error = sx127xError<Error, Infallible, Infallible>>,
    TxType, RxType,
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    PC13<Output<PushPull>>,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa
                .pa5.into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
                //.into_af5_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
            gpioa
                .pa6
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // miso  on PA6
            gpioa
                .pa7
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // mosi  on PA7
        ),
        MODE,
        8.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let delay = Delay::new(cp.SYST, clocks);

    // Create lora radio instance

    let lora = Sx127x::spi(
        spi, //Spi
        gpioa
            .pa1
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            , //CsPin   on PA1
        gpiob
            .pb8
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr)
            , //BusyPin  DIO0 on PB8
        gpiob
            .pb9
            .into_floating_input(&mut gpiob.moder, &mut gpiob.pupdr)
            , //ReadyPin DIO1 on PB9
        gpioa
            .pa0
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper)
            , //ResetPin      on PA0
        delay, //Delay
        &CONFIG_RADIO, //&Config
    )
    .unwrap(); // should handle error

    let (tx, rx) = Serial::usart2(
        dp.USART2,
        (
            gpioa
                .pa2
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2  for GPS
            gpioa
                .pa3
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3  for GPS
        ),
        serial::Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

    let mut scl =
        gpioa
            .pa9
            .into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // scl on PA9
    scl.internal_pull_up(&mut gpioa.pupdr, true);

    let mut sda =
        gpioa
            .pa10
            .into_alternate_open_drain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // sda on PA10
    sda.internal_pull_up(&mut gpioa.pupdr, true);

    let i2c = I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        Config::new(400.khz(), clocks),
        &mut rcc.apb1r1,
    );

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    // led on pc13 with on/off
    let led = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

    (lora, tx, rx, i2c, led)
}
