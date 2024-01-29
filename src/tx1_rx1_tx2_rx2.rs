
use crate::stm32xxx_as_hal::hal;

pub use hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    serial::{Serial, Tx, Rx, Error},
    prelude::*,
};

#[cfg(feature = "stm32f0xx")] // eg stm32f030xc  stm32f042
pub use stm32f0xx_hal::{
    serial::{Serial, Tx, Rx},
};

#[cfg(feature = "stm32f0xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
    let mut rcc = dp.RCC.configure().sysclk(48.mhz()).freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx1, rx1, tx2, rx2) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa9.into_alternate_af1(cs),  //tx pa9
            gpioa.pa10.into_alternate_af1(cs), //rx pa10
            gpioa.pa2.into_alternate_af1(cs),  //tx pa2
            gpioa.pa3.into_alternate_af1(cs),  //rx pa3
        )
    });

    let (tx1, rx1) = Serial::usart1(p.USART1, (tx1, rx1), 9600.bps(), &mut rcc).split();

    let (tx2, rx2) = Serial::usart2(p.USART2, (tx2, rx2), 9600.bps(), &mut rcc).split();

    (tx1, rx1, tx2, rx2)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
pub use stm32f1xx_hal::{
    serial::{Config, StopBits},
};

#[cfg(feature = "stm32f1xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART3>, Rx<USART3>) {
    let p = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);
    let mut afio = p.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();

    // next consumes (moves) arguments other than clocks,  &mut rcc.apb2 and afio.
    let (tx1, rx1) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh), //tx pa9   for console
            gpioa.pa10,
        ), //rx pa10  for console
        &mut afio.mapr,
        Config::default()
            .baudrate(9600.bps())
            .stopbits(StopBits::STOP1), //.parity_odd()
        &clocks,
    )
    .split();

    let mut gpiob = p.GPIOB.split();
    let (tx3, rx3) = Serial::new(
        p.USART3,
        (
            gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh), //tx pb10  for GPS rx
            gpiob.pb11,
        ), //rx pb11  for GPS tx
        &mut afio.mapr,
        Config::default().baudrate(9_600.bps()),
        &clocks,
    )
    .split();

    (tx1, rx1, tx3, rx3)
}



#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
pub use stm32f3xx_hal::{
    serial::{RxPin, TxPin},
};

#[cfg(feature = "stm32f3xx")]
pub fn setup_from_dp(dp: Peripherals) -> (
    Tx<USART1, impl TxPin<USART1>>,
    Rx<USART1, impl RxPin<USART1>>,
    Tx<USART2, impl TxPin<USART2>>,
    Rx<USART2, impl RxPin<USART2>>,
) {
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut dp.FLASH.constrain().acr);

    //Why does next need arg, there is only one possibility?
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);

    let (tx1, rx1) = Serial::new(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        9600.Bd(),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    let (tx2, rx2) = Serial::new(
        dp.USART2,
        (
            gpioa
                .pa2
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2
            gpioa
                .pa3
                .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3
        ),
        115_200.Bd(), // 9600.bps(),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    (tx1, rx1, tx2, rx2)
}



#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
pub use stm32f4xx_hal::{
    serial::{config::Config},
};

#[cfg(feature = "stm32f4xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
    let clocks = dp.RCC.constrain().cfgr.freeze();
    let gpioa = dp.GPIOA.split();
    let (tx1, rx1) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(),  //tx pa9   for console
            gpioa.pa10.into_alternate(), //rx pa10  for console
        ),
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap()
    .split();

    // this probably needs fix here. rx2.read() stalls and does not return.
    //p.USART2.cr1.modify(|_,w| w.rxneie().set_bit());  //need RX interrupt?
    let (tx2, rx2) = Serial::new(
        dp.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2  for GPS
            gpioa.pa3.into_alternate(), //rx pa3  for GPS
        ),
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap()
    .split();

    (tx1, rx1, tx2, rx2)
}



#[cfg(feature = "stm32f7xx")]
pub use stm32f7xx_hal::{
    serial::{Config, Oversampling,  DataBits, Parity},
};

#[cfg(feature = "stm32f7xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
    let clocks = dp.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let gpioa = dp.GPIOA.split();

    let (tx1, rx1) = Serial::new(
        dp.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9   for console
            gpioa.pa10.into_alternate(),
        ), //rx pa10  for console
        &clocks,
        Config {
            baud_rate: 9600.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
    )
    .split();

    let (tx2, rx2) = Serial::new(
        dp.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2  for GPS
            gpioa.pa3.into_alternate(),
        ), //rx pa3  for GPS
        &clocks,
        Config {
            baud_rate: 9600.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
    )
    .split();

    (tx1, rx1, tx2, rx2)
}



#[cfg(feature = "stm32g0xx")]
pub use stm32g0xx_hal::{
    serial::{FullConfig,},
};

#[cfg(feature = "stm32g0xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1, FullConfig>, Rx<USART1, FullConfig>, Tx<USART2, FullConfig>, Rx<USART2, FullConfig>) {
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);

    //console
    let (tx1, rx1) = dp.USART1.usart((gpioa.pa9, gpioa.pa10),
                        FullConfig::default(), &mut rcc).unwrap().split();

    //GPS
    let (tx2, rx2) = dp.USART2.usart((gpioa.pa2, gpioa.pa3),
                        FullConfig::default(), &mut rcc).unwrap().split();

    (tx1, rx1, tx2, rx2)
}



#[cfg(feature = "stm32g4xx")]
pub use stm32g4xx_hal::{
    serial::{FullConfig, NoDMA},
    gpio::{Alternate, gpioa::{PA2, PA3, PA9, PA10}},
};

#[cfg(feature = "stm32g4xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1, PA9<Alternate<7_u8>>, NoDMA>, Rx<USART1, PA10<Alternate<7_u8>>, NoDMA>,
               Tx<USART2, PA2<Alternate<7_u8>>, NoDMA>, Rx<USART2, PA3<Alternate<7_u8>>, NoDMA>) {
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx1, rx1) = dp.USART1.usart(   //tx, rx  for console
       gpioa.pa9.into_alternate(), 
       gpioa.pa10.into_alternate(), 
       FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

    let (tx2, rx2) = dp.USART2.usart(   //tx, rx  forGPS
        gpioa.pa2.into_alternate(), 
        gpioa.pa3.into_alternate(),
        FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

    (tx1, rx1, tx2, rx2)
}




#[cfg(feature = "stm32h7xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);

    let (tx1, rx1) = dp.USART1.serial(
            (
                gpioa.pa9.into_alternate(), //tx pa9
                gpioa.pa10.into_alternate(),
            ), //rx pa10
            9600.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split();

    let (tx2, rx2) = dp.USART2.serial(
            (
                gpioa.pa2.into_alternate(), //tx pa2
                gpioa.pa3.into_alternate(),
            ), //rx pa3
            9600.bps(),
            ccdr.peripheral.USART2,
            &clocks,
        )
        .unwrap()
        .split();

    (tx1, rx1, tx2, rx2)
}

#[cfg(feature = "stm32l0xx")]
pub use stm32l0xx_hal::{
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Serial1Ext, Serial2Ext},
};

#[cfg(feature = "stm32l0xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx1, rx1) = dp.USART1.usart(
            gpioa.pa9,  //tx pa9  for console
            gpioa.pa10, //rx pa10 for console
            Config::default().baudrate(9600.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let (tx2, rx2) = dp.USART2.usart(
            gpioa.pa2, //tx pa2  for GPS
            gpioa.pa3, //rx pa3  for GPS
            Config::default().baudrate(9600.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    (tx1, rx1, tx2, rx2)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
pub use stm32l1xx_hal::{
    rcc, // for ::Config but note name conflict with next
    serial::{Config, SerialExt,},
};

// The Heltec lora_node 151 uses USART2 and USART3 pins for on board LoRa connections and power
// detection. See
// https://resource.heltec.cn/download/LoRa_Node_151/LoRa_Node_151_Pinout_Diagram.pdf.
// So only USART1 is available and this example cannot work on Heltec lora_node 151 as
// it needs 2 USARTs. USART1 is used for the GPS as oled_gps and lora_gps examples might work.
// For simplicity of this example the same setup is used on the Discovery kit stm32l100.

#[cfg(feature = "stm32l1xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART2>, Rx<USART2>, Tx<USART1>, Rx<USART1>) {
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());
    //let clocks  = rcc.cfgr.freeze();

    let gpioa = dp.GPIOA.split(&mut rcc);

    let (txc, rxc) = dp.USART2.usart(
            (
                gpioa.pa2, //tx pa2   for console
                gpioa.pa3,
            ), //rx pa3   for console
            Config::default().baudrate(115_200.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let (txg, rxg) = dp.USART1.usart(
            (
                gpioa.pa9, //tx pa9  for GPS rx
                gpioa.pa10,
            ), //rx pa10 for GPS tx
            Config::default().baudrate(9600.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    (txc, rxc, txg, rxg)
}

#[cfg(feature = "stm32l4xx")]
pub use stm32l4xx_hal::{
    serial::{Config},
};

#[cfg(feature = "stm32l4xx")]
pub fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let (tx1, rx1) = Serial::usart1(
        dp.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9  for console
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10 for console
        ),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    let (tx2, rx2) = Serial::usart2(
        dp.USART2,
        (
            gpioa
                .pa2
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2  for GPS
            gpioa
                .pa3
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3  for GPS
        ),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

    (tx1, rx1, tx2, rx2)
}
