//! Serial interface test writing a buffer of bytes between two usarts and
//! echo to the computer console connected by usb-ttl dongle on another usart.
//! This example differs from example serial_char in that it attempts to send
//! a whole buffer rather than a single byte.
//! See example serial_char regarding the usart details, pins connections,
//! and echo_string.rs for additional comments.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use cortex_m::asm;
use cortex_m::singleton;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
//use nb::block;

use core::str::from_utf8;

#[cfg(feature = "stm32f0xx")] //  eg blue pill stm32f103
use stm32f0xx_hal::{
    dma::{
        dma1::{C2, C3, C4, C5, C6, C7},
        RxDma, TxDma,
    },
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Rx, Serial, StopBits, Tx},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    TxDma<Tx<USART1>, C4>,
    RxDma<Rx<USART1>, C5>,
    TxDma<Tx<USART2>, C7>,
    RxDma<Rx<USART2>, C6>,
    TxDma<Tx<USART3>, C2>,
    RxDma<Rx<USART3>, C3>,
) {
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);
    let gpiob = p.GPIOB.split(&mut rcc);

    let (tx1, rx1, tx2, rx2, tx3, rx3) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa9.into_alternate_af1(cs),  //tx pa9
            gpioa.pa10.into_alternate_af1(cs), //rx pa10
            gpioa.pa2.into_alternate_af1(cs),  //tx pa2
            gpioa.pa3.into_alternate_af1(cs),  //rx pa3
            gpiob.pb10.into_alternate_af4(cs), //tx pb10
            gpiob.pb11.into_alternate_af4(cs), //rx pb11
        )
    });

    let (tx1, rx1) = Serial::usart1(p.USART1, (tx1, rx1), 9600.bps(), &mut rcc).split();

    let (tx2, rx2) = Serial::usart2(p.USART2, (tx2, rx2), 9600.bps(), &mut rcc).split();

    let (tx3, rx3) = Serial::usart3(p.USART3, (tx3, rx3), 9600.bps(), &mut rcc).split();

    let channels = p.DMA1.split(&mut rcc.ahb);

    let tx1 = tx1.with_dma(channels.4); // console
    let rx1 = rx1.with_dma(channels.5);

    let tx2 = tx2.with_dma(channels.7);
    let rx2 = rx2.with_dma(channels.6);

    let tx3 = tx3.with_dma(channels.2);
    let rx3 = rx3.with_dma(channels.3);

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    device::{USART1, USART2, USART3},
    dma::dma1,
    //dma::{TxDma, RxDma, dma1::{C2, C3, C4, C5, C6, C7}},
    pac::Peripherals,
    prelude::*,
    serial::{Config, Rx, Serial, StopBits, Tx},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Tx<USART1>,
    dma1::C4,
    Rx<USART1>,
    dma1::C5,
    Tx<USART2>,
    dma1::C7,
    Rx<USART2>,
    dma1::C6,
    Tx<USART3>,
    dma1::C2,
    Rx<USART3>,
    dma1::C3,
) {
    //fn setup() ->  (TxDma<Tx<USART1>, C4>, RxDma<Rx<USART1>, C5>,
    //                TxDma<Tx<USART2>, C7>, RxDma<Rx<USART2>, C6>,
    //                TxDma<Tx<USART3>, C2>, RxDma<Rx<USART3>, C3> )  {

    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut afio = p.AFIO.constrain();

    let channels = p.DMA1.split();

    let mut gpioa = p.GPIOA.split();

    let (tx1, rx1) = Serial::usart1(
        p.USART1,
        (
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh), //tx pa9
            gpioa.pa10,
        ), //rx pa10
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()), //.stopbits(StopBits::STOP1
        clocks,
    )
    .split();

    let (tx2, rx2) = Serial::usart2(
        p.USART2,
        (
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl), //tx pa2
            gpioa.pa3,
        ), //rx pa3
        &mut afio.mapr,
        Config::default()
            .baudrate(9_600.bps())
            .parity_odd()
            .stopbits(StopBits::STOP1),
        clocks,
    )
    .split();

    let mut gpiob = p.GPIOB.split();

    let (tx3, rx3) = Serial::usart3(
        p.USART3,
        (
            gpiob.pb10.into_alternate_push_pull(&mut gpiob.crh), //rx pb10
            gpiob.pb11,
        ), //tx pb11
        &mut afio.mapr,
        Config::default()
            .baudrate(9_600.bps())
            .parity_odd()
            .stopbits(StopBits::STOP1),
        clocks,
    )
    .split();

    //let tx1  = tx1.with_dma(channels.4);                // console
    //let rx1  = rx1.with_dma(channels.5);
    //let tx2  = tx2.with_dma(channels.7);
    //let rx2  = rx2.with_dma(channels.6);
    //let tx3  = tx3.with_dma(channels.2);
    //let rx3  = rx3.with_dma(channels.3);

    let dma1 = p.DMA1.split();
    let (tx1_ch, rx1_ch) = (dma1.4, dma1.5); // console
    let (tx2_ch, rx2_ch) = (dma1.7, dma1.6);
    let (tx3_ch, rx3_ch) = (dma1.2, dma1.3);

    (
        tx1, tx1_ch, rx1, rx1_ch, tx2, tx2_ch, rx2, rx2_ch, tx3, tx3_ch, rx3, rx3_ch,
    )
}

//#[cfg(any(feature = "stm32f1xx", feature = "stm32l1xx"))]
//let channels = p.DMA1.split(&mut rcc.ahb);
//let mut tx = txrx1.split().0.with_dma(channels.4);     //works on stm32f1xx_hal but not others
//let (_, tx) = tx.write(b"The quick brown fox").wait(); //works on stm32f1xx_hal but not others

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    dma::dma1,
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Tx<USART1, impl TxPin<USART1>>,
    dma1::C4,
    Rx<USART1, impl RxPin<USART1>>,
    dma1::C5,
    Tx<USART2, impl TxPin<USART2>>,
    dma1::C7,
    Rx<USART2, impl RxPin<USART2>>,
    dma1::C6,
    Tx<USART3, impl TxPin<USART3>>,
    dma1::C2,
    Rx<USART3, impl RxPin<USART3>>,
    dma1::C3,
) {
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    //Why does next need arg, there is only one possibility?
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);
    let (tx1, rx1) = Serial::new(
        p.USART1,
        (
            gpioa
                .pa9
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        9600.Bd(),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa
                .pa2
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2
            gpioa
                .pa3
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3
        ),
        115_200.Bd(), // or 9600.bps(),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

    let (tx3, rx3) = Serial::new(
        p.USART3,
        (
            gpiob
                .pb10
                .into_af7_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //tx pb10
            gpiob
                .pb11
                .into_af7_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //rx pb11
        ),
        115_200.Bd(), // or 9600.bps(),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    let dma1 = p.DMA1.split(&mut rcc.ahb);
    let (tx1_ch, rx1_ch) = (dma1.ch4, dma1.ch5);
    let (tx2_ch, rx2_ch) = (dma1.ch7, dma1.ch6);
    let (tx3_ch, rx3_ch) = (dma1.ch2, dma1.ch3);

    (
        tx1, tx1_ch, rx1, rx1_ch, tx2, tx2_ch, rx2, rx2_ch, tx3, tx3_ch, rx3, rx3_ch,
    )
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64  stm32f411
use stm32f4xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART6},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART6>,
    Rx<USART6>,
) {
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split();
    p.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx1, rx1) = Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        clocks,
    )
    .unwrap()
    .split();

    p.USART2.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2
            gpioa.pa3.into_alternate(),
        ), //rx pa3
        Config::default().baudrate(115_200.bps()), //.parity_odd() .stopbits(StopBits::STOP1)
        clocks,
    )
    .unwrap()
    .split();

    p.USART6.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx3, rx3) = Serial::new(
        //  NOTE PINS and USART6 !!!
        p.USART6,
        (
            gpioa.pa11.into_alternate(), //tx pa11
            gpioa.pa12.into_alternate(),
        ), //rx pa12
        Config::default().baudrate(115_200.bps()),
        clocks,
    )
    .unwrap()
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Config, Oversampling, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART3>,
    Rx<USART3>,
) {
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.sysclk(216.MHz()).freeze();

    let gpioa = p.GPIOA.split();

    let (tx1, rx1) = Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        clocks,
        Config {
            baud_rate: 9600.Bps(),
            oversampling: Oversampling::By16,
            character_match: None,
        },
    )
    .split();

    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2
            gpioa.pa3.into_alternate(),
        ), //rx pa3
        clocks,
        Config {
            baud_rate: 115_200.Bps(),
            oversampling: Oversampling::By16,
            character_match: None,
        },
    )
    .split();

    let gpiob = p.GPIOB.split();

    let (tx3, rx3) = Serial::new(
        p.USART3,
        (
            gpiob.pb10.into_alternate(), //tx pb10
            gpiob.pb11.into_alternate(),
        ), //rx pb11
        clocks,
        Config {
            baud_rate: 115_200.Bps(),
            oversampling: Oversampling::By16,
            character_match: None,
        },
    )
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Rx, Tx},
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART3>,
    Rx<USART3>,
) {
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = p.GPIOA.split(ccdr.peripheral.GPIOA);

    let (tx1, rx1) = p
        .USART1
        .serial(
            (
                gpioa.pa9.into_alternate_af7(), //tx pa9
                gpioa.pa10.into_alternate_af7(),
            ), //rx pa10
            9600.bps(),
            ccdr.peripheral.USART1,
            &clocks,
        )
        .unwrap()
        .split();

    let (tx2, rx2) = p
        .USART2
        .serial(
            (
                gpioa.pa2.into_alternate_af7(), //tx pa2
                gpioa.pa3.into_alternate_af7(),
            ), //rx pa3
            115_200.bps(),
            ccdr.peripheral.USART2,
            &clocks,
        )
        .unwrap()
        .split();

    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);

    let (tx3, rx3) = p
        .USART3
        .serial(
            (
                gpiob.pb10.into_alternate_af7(), //tx pb10
                gpiob.pb11.into_alternate_af7(),
            ), //rx pb11
            115_200.bps(),
            ccdr.peripheral.USART3,
            &clocks,
        )
        .unwrap()
        .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART6},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART6>,
    Rx<USART6>,
) {
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze();
    let gpioa = p.GPIOA.split();
    p.USART1.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx1, rx1) = Serial::usart1(
        p.USART1,
        (
            gpioa.pa9.into_alternate_af7(), //tx pa9
            gpioa.pa10.into_alternate_af7(),
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        clocks,
    )
    .unwrap()
    .split();

    p.USART2.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx2, rx2) = Serial::usart2(
        p.USART2,
        (
            gpioa.pa2.into_alternate_af7(), //tx pa2
            gpioa.pa3.into_alternate_af7(),
        ), //rx pa3
        Config::default().baudrate(115_200.bps()), //.parity_odd() .stopbits(StopBits::STOP1)
        clocks,
    )
    .unwrap()
    .split();

    p.USART6.cr1.modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx3, rx3) = Serial::usart6(
        //  NOTE PINS and USART6 !!!
        p.USART6,
        (
            gpioa.pa11.into_alternate_af8(), //tx pa11
            gpioa.pa12.into_alternate_af8(),
        ), //rx pa12
        Config::default().baudrate(115_200.bps()),
        clocks,
    )
    .unwrap()
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, SerialExt, Tx},
    stm32::Peripherals,
    stm32::{USART1, USART2, USART3},
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART3>,
    Rx<USART3>,
) {
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());
    //let clocks  = rcc.cfgr.freeze();

    let gpioa = p.GPIOA.split(&mut rcc);

    // Note that setting the alternate function mode and push_pull input/output
    // is not necessary. The hal code knows to do this for a usart.
    let (tx1, rx1) = p
        .USART1
        .usart(
            (
                gpioa.pa9, //tx pa9
                gpioa.pa10,
            ), //rx pa10
            Config::default().baudrate(9600.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let (tx2, rx2) = p
        .USART2
        .usart(
            (
                gpioa.pa2, //tx pa2
                gpioa.pa3,
            ), //rx pa3
            Config::default().baudrate(115_200.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let gpiob = p.GPIOB.split(&mut rcc);

    let (tx3, rx3) = p
        .USART3
        .usart(
            (
                gpiob.pb10, //tx pb10
                gpiob.pb11,
            ), //rx pb11
            Config::default().baudrate(115_200.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART3>,
    Rx<USART3>,
) {
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let rcc = p.RCC.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    let (tx1, rx1) = Serial::usart1(
        p.USART1,
        (
            gpioa
                .pa9
                .into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //rx pa10
        ),
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    )
    .split();

    let (tx2, rx2) = Serial::usart2(
        p.USART2,
        (
            gpioa
                .pa2
                .into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2
            gpioa
                .pa3
                .into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3
        ),
        Config::default().baudrate(115_200.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb2);

    let (tx3, rx3) = Serial::usart3(
        p.USART3,
        (
            gpiob
                .pb10
                .into_af7_pushpull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //tx pb10
            gpiob
                .pb11
                .into_af7_pushpull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //rx pb11
        ),
        Config::default().baudrate(115_200.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

// End of hal/MCU specific setup. Following should be generic code.

const BUF_SIZE: usize = 20;

#[entry]
fn main() -> ! {
    // See echo_string.rs for additional comments.

    let (tx1, tx1_ch, _rx1, _rx1_ch, tx2, tx2_ch, _rx2, _rx2_ch, _tx3, _tx3_ch, rx3, rx3_ch) =
        setup();

    // The first use of read_exact() and write_all() create send1, recv3, and send2 structures that can be
    // modified (even inside a loop). Those structures are then used for additional  read_exact() and write_all().

    hprintln!("testing write to console").unwrap();

    //this is 3-tuple send structure (buf1, tx1_ch, tx1)
    let buf1 = singleton!(: [u8; BUF_SIZE] = *b"\r\ncheck console...\r\n").unwrap();
    let mut send1 = tx1.write_all(buf1, tx1_ch).wait();

    *send1.0 = *b"Display on console\r\n"; // BUF_SIZE characters
    send1 = send1.2.write_all(send1.0, send1.1).wait();

    hprintln!("testing  tx2 to rx3").unwrap();

    // rx process should be started before tx, or rx misses the transmition and stalls waiting.

    //send1 using recv3 requires buf3 has same size as buf1
    let buf3 = singleton!(: [u8; BUF_SIZE] = [0; BUF_SIZE]).unwrap();
    let mut rx = rx3.read_exact(buf3, rx3_ch);

    //CHECK IF buf2 REALLY NEED TO BE SAME SIZE?
    let buf2 = singleton!(: [u8; BUF_SIZE] = [b' '; BUF_SIZE]).unwrap();
    let tx = tx2.write_all(buf2, tx2_ch);

    let mut send2 = tx.wait(); //when tx is complete return 3-tuple send structure (buf2, tx2_ch, tx2)
    let mut recv3 = rx.wait(); //when rx is complete return 3-tuple recv structure (buf3, rx3_ch, rx3)

    //hprintln!("  check received = sent,  '{}' = '{}' ", from_utf8(recv3.0), from_utf8(send2.0)).unwrap();
    assert_eq!(recv3.0, send2.0);

    // Now recvX and sendX structures can be modified rather than assigned

    // BUF_SIZE characters each
    let buf4: &[_] = &[
        *b"in for iter 1     \r\n",
        *b"in for iter 2     \r\n",
        *b"in for iter 3     \r\n",
    ];

    //hprintln!(" buf4 {:?}", &buf4).unwrap();

    for i in buf4.iter() {
        hprintln!(" i is '{:?}'", i).unwrap();
        //hprintln!(" i is '{:?}'", from_utf8(i)).unwrap();

        rx = recv3.2.read_exact(send1.0, recv3.1); // rx ready to receive into send1 buf
                                                   // This requires buf3 has same size as buf1.

        *send2.0 = *i; // BUF_SIZE characters
        send2 = send2.2.write_all(send2.0, send2.1).wait(); // tx and return

        recv3 = rx.wait(); // rx returns

        assert_eq!(recv3.0, send2.0); // check received  = sent

        send1 = send1.2.write_all(recv3.0, send1.1).wait(); // send received to console
    }

    hprintln!("entering empty loop. ^C to exit.").unwrap();
    loop {}
}
