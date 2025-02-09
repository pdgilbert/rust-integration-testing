//!  Tested on weact-stm32g474 July 13, 2024. Partially works. 
//!     tx2 to rx3 is working but tx3 to rx2 gives rx2.read error Framing.

//! FAILS TO COMPILE WITH stm32f4xx_hal. See
//! https://github.com/stm32-rs/stm32f4xx-hal/issues/721

//! Serial interface test writing a single character between two usarts and
//! echo to the computer consol connected by usb-ttl dongle on another usart.
//!
//! With all HALs and boards the console is on USART1 and uses pins pa9 and pa10.
//! With all HALs and boards one of the serially connected ports is USART2 using
//! pins pa2 and pa3.
//! The third USART varies depending on what is available.
//! Some details follow but see also comment in the code.
//!
//! Console connection details:
//! Connect usart1  to serial-usb converter on computer for the console.
//! usart1 connect the Tx pin pa9  to the Rx pin of a serial-usb converter
//! usart1 connect the Rx pin pa10 to the Tx pin of a serial-usb converter
//! Set up the serial console (e.g. minicom) with the same settings used here (8-N-1).
//! (Using 9600bps. This could be higher but needs serial console to be the same.)
//!
//! Serial USART interconnect details:
//! For HALs stm32f1xx, stm32f3xx, stm3g4xx and stm32l1xx connect USART2 to USART3: pa2 to pb11 and pa3 to pb10.
//! For HAL stm32f4xx connect USART2 to USART6:  pa2 to pa12 and pa3 to pa11.

// This example contains the most extensive notes.
// ANY NOTES SHOULD BE EXPANDED HERE IF THEY APPLY HERE,
// OTHERWISE PUT THEM IN THE EXAMPLE WHERE THEY APPLY !

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

//use cortex_m::asm;
use core::str::from_utf8;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
//use nb::block;

use embedded_io::{Read, Write};


// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

// 1. Get access to the device specific peripherals
// 2. Take ownership of raw rcc and flash devices and convert to HAL structs
// 3. Freeze system clocks and store the frozen frequencies in `clocks`
// 4. Prepare the alternate function I/O registers
// 5. Prepare the GPIO peripheral
// 6. Set up usart devices. Take ownership of USART register and tx/rx pins.
//    Other registers are used to enable and configure the device.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Rx, Serial, Tx},
};

#[cfg(feature = "stm32f0xx")] // NOTE this fails on stm32f042 which has only 2 USARTs
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART3>,
    Rx<USART3>,
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

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    pac::{Peripherals,USART1, USART2, USART3},
    prelude::*,
    serial::{Config, Rx, Serial, StopBits, Tx},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART3>,
    Rx<USART3>,
) {
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut afio = p.AFIO.constrain();
    let mut gpioa = p.GPIOA.split();
    // next consumes (moves) arguments other than clocks,  &mut rcc.apb2 and afio.
    let (tx1, rx1) = Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh), //tx pa9
            gpioa.pa10,
        ), //rx pa10
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()), //.stopbits(StopBits::STOP1
        &clocks,
    )
    .split();
    let (tx2, rx2) = Serial::new(
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
        &clocks,
    )
    .split();

    let mut gpiob = p.GPIOB.split();

    let (tx3, rx3) = Serial::new(
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
        &clocks,
    )
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Tx<USART1, impl TxPin<USART1>>,
    Rx<USART1, impl RxPin<USART1>>,
    Tx<USART2, impl TxPin<USART2>>,
    Rx<USART2, impl RxPin<USART2>>,
    Tx<USART3, impl TxPin<USART3>>,
    Rx<USART3, impl RxPin<USART3>>,
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
        p.USART2,
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

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

    let (tx3, rx3) = Serial::new(
        p.USART3,
        (
            gpiob
                .pb10
                .into_af_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //tx pb10
            gpiob
                .pb11
                .into_af_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //rx pb11
        ),
        115_200.Bd(), // 9600.bps(),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
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

    // alternate function modes see https://www.st.com/resource/en/datasheet/stm32f411re.pdf  p47.

    p.USART1.cr1().modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx1, rx1) = Serial::new(
        p.USART1,
        (
            gpioa.pa9.into_alternate(), //tx pa9
            gpioa.pa10.into_alternate(),
        ), //rx pa10
        Config::default().baudrate(9600.bps()),
        &clocks,
    )
    .unwrap()
    .split();

    p.USART2.cr1().modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2
            gpioa.pa3.into_alternate(),
        ), //rx pa3
        Config::default().baudrate(115_200.bps()), //.parity_odd() .stopbits(StopBits::STOP1)
        &clocks,
    )
    .unwrap()
    .split();

    p.USART6.cr1().modify(|_, w| w.rxneie().set_bit()); //need RX interrupt?
    let (tx3, rx3) = Serial::new(
        //  NOTE PINS and USART6 !!!
        p.USART6,
        (
            gpioa.pa11.into_alternate(), //tx pa11
            gpioa.pa12.into_alternate(),
        ), //rx pa12
        Config::default().baudrate(115_200.bps()),
        &clocks,
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
    serial::{Config, Oversampling, Rx, Serial, Tx, DataBits, Parity},
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
        p.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2
            gpioa.pa3.into_alternate(),
        ), //rx pa3
        &clocks,
        Config {
            baud_rate: 115_200.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
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
        &clocks,
        Config {
            baud_rate: 115_200.bps(),
            data_bits: DataBits::Bits9,  // 8 bits of data + 1 for even parity  CHECK THIS FOR HARDWARE
            parity: Parity::ParityEven,
            oversampling: Oversampling::By16,
            character_match: None,
            sysclock: false,
        },
    )
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}



#[cfg(feature = "stm32g0xx")]
use stm32g0xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2, USART3},
    prelude::*,
    serial::{FullConfig, Tx, Rx, BasicConfig},
};

#[cfg(feature = "stm32g0xx")]
fn setup() -> (
    Tx<USART1, FullConfig>,
    Rx<USART1, FullConfig>,
    Tx<USART2, FullConfig>,
    Rx<USART2, FullConfig>,
    Tx<USART3, BasicConfig>,
    Rx<USART3, BasicConfig>,
) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let (tx1, rx1) = dp.USART1.usart((gpioa.pa9, gpioa.pa10),
                        FullConfig::default(), &mut rcc).unwrap().split();

    let (tx2, rx2) = dp.USART2.usart((gpioa.pa2, gpioa.pa3),
                        FullConfig::default(), &mut rcc).unwrap().split();

    let (tx3, rx3) = dp.USART3.usart((gpiob.pb8, gpiob.pb9),
                        BasicConfig::default(), &mut rcc).unwrap().split();


    (tx1, rx1, tx2, rx2, tx3, rx3)
}



#[cfg(feature = "stm32g4xx")]
use stm32g4xx_hal::{
    stm32::Peripherals,
    stm32::{USART1, USART2, USART3},
    prelude::*,
    serial::{FullConfig, Rx, Tx, NoDMA},
    gpio::{Alternate,  
          gpioa::{PA2, PA3, PA9, PA10},
          gpiob::{PB10, PB11}},
};

#[cfg(feature = "stm32g4xx")]
fn setup() -> (
    Tx<USART1, PA9<Alternate<7_u8>>, NoDMA>,
    Rx<USART1, PA10<Alternate<7_u8>>, NoDMA>,
    Tx<USART2, PA2<Alternate<7_u8>>, NoDMA>,
    Rx<USART2, PA3<Alternate<7_u8>>, NoDMA>,
    Tx<USART3, PB10<Alternate<7_u8>>, NoDMA>,
    Rx<USART3, PB11<Alternate<7_u8>>, NoDMA>,
) {
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let (tx1, rx1) = dp.USART1.usart(   
        gpioa.pa9.into_alternate(), 
        gpioa.pa10.into_alternate(),
        FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

    let (tx2, rx2) = dp.USART2.usart(  
        gpioa.pa2.into_alternate(), 
        gpioa.pa3.into_alternate(),
        FullConfig::default().baudrate(115_200.bps()), &mut rcc).unwrap().split();

    // Err(Overrun) using 9600.bps(). Err(Framing) using 115_200.bps()  (with breakpoint).

    let (tx3, rx3) = dp.USART3.usart( 
        gpiob.pb10.into_alternate(),
        gpiob.pb11.into_alternate(), 
        FullConfig::default().baudrate(115_200.bps()), &mut rcc).unwrap().split();

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
    let ccdr = rcc.sys_ck(160.MHz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = p.GPIOA.split(ccdr.peripheral.GPIOA);

    let (tx1, rx1) = p
        .USART1
        .serial(
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

    let (tx2, rx2) = p
        .USART2
        .serial(
            (
                gpioa.pa2.into_alternate(), //tx pa2
                gpioa.pa3.into_alternate(),
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
                gpiob.pb10.into_alternate(), //tx pb10
                gpiob.pb11.into_alternate(),
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
    pac::{USART1, USART2, USART4},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, Serial1Ext, Serial2Ext, Serial4Ext, Tx},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    Tx<USART1>,
    Rx<USART1>,
    Tx<USART2>,
    Rx<USART2>,
    Tx<USART4>,
    Rx<USART4>,
) {
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());

    let gpioa = p.GPIOA.split(&mut rcc);

    let (tx1, rx1) = p
        .USART1
        .usart(
            gpioa.pa9,  //tx pa9  for console
            gpioa.pa10, //rx pa10 for console
            Config::default().baudrate(9600.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let (tx2, rx2) = p
        .USART2
        .usart(
            gpioa.pa2, //tx pa2
            gpioa.pa3, //rx pa3
            Config::default().baudrate(115_200.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    // no USART3 in this HAL
    let (tx3, rx3) = p
        .USART4
        .usart(
            gpioa.pa0, //tx pa0
            gpioa.pa1, //rx pa1
            Config::default().baudrate(115_200.Bd()),
            &mut rcc,
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
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.MHz())
        .pclk1(80.MHz())
        .pclk2(80.MHz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    let (tx1, rx1) = Serial::usart1(
        p.USART1,
        (
            gpioa
                .pa9
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh), //tx pa9
            gpioa
                .pa10
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
        ), //rx pa10
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
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2
            gpioa
                .pa3
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3
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
                .into_alternate_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //tx pb10
            gpiob
                .pb11
                .into_alternate_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh), //rx pb11
        ),
        Config::default().baudrate(115_200.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

    (tx1, rx1, tx2, rx2, tx3, rx3)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    //see  examples/echo_by_char.rs for additional comments.
    hprintln!("initializing ...").unwrap();

    let mut received:  [u8; 80] = [0; 80];
    let mut received2: [u8; 80] = [0; 80];
    //let rec_byte: u8 = 0;

    let (mut tx1, _rx1, mut tx2, mut rx2, mut tx3, mut rx3) = setup();

    hprintln!("test write to console ...").unwrap();
    tx1.write_all("\r\nconsole connect check.\r\n".as_bytes()).unwrap();
//    for byte in b"\r\nconsole connect check.\r\n" {
//        #[cfg(feature = "stm32f4xx")]
//        block!(tx1.write(*byte)).ok();
//        #[cfg(not(feature = "stm32f4xx"))]
//        block!(tx1.write_byte(*byte)).ok();
//    }

    hprintln!("testing  tx2 to rx3").unwrap();
    hprintln!("   sending on tx2 ...").unwrap();

    let send = b'X';

    // Write `X` and wait until the write is successful
    tx2.write_all(&[send]).unwrap(); 
    tx2.flush().unwrap(); 

    hprintln!("   receiving on rx3 ...").unwrap();

    // Read the byte that was just send. Blocks until the read is complete
    // see https://docs.rs/embedded-io/latest/embedded_io/trait.Read.html
    let len = rx3.read(&mut received).unwrap();

    hprintln!("   length received on rx3: {}", len).unwrap();
    hprintln!("   checking tx2 to rx3 received = send,  {:?} = {:?} byte", received[0], send ).unwrap();

    // The send byte should be the one received
    //assert_eq!( received, send,   "testing received = send,  {} = {}", received, send );

    // Maybe put a test here to show failure. assert does panic halt so remaining tests do not run.

    // Now print to semi-host as character rather than byte.
    // Note that send above was u8 byte (b'X') because tx.write() requires that, but
    //    hprintln!() needs a str and from_utf8() needs a slice, thus [send].

    hprintln!("      tx2 to rx3[0]  characters,  {} = {}",
        from_utf8(&[received[0]]).unwrap(),
        from_utf8(&[send]).unwrap()
    ).unwrap();

    hprintln!("   sending received to console on tx1 ...").unwrap();

    tx1.write_all("\r\ntx2 to rx3 with X\r\n".as_bytes()).unwrap();
    tx1.write_all(&received).unwrap();
    tx1.write_all("\r\n".as_bytes()).unwrap();

    // Trigger a breakpoint
    // asm::bkpt();



    hprintln!("testing  tx3 to rx2").unwrap();
    hprintln!("   sending on tx3 ...").unwrap();
    tx1.write_all("\r\ntx3 to rx2 with Y\r\n".as_bytes()).unwrap();

    let send = b'Y';

    // Write `Y` and wait until the write is successful

    tx3.write_all(&[send]).unwrap();
    tx3.flush().unwrap(); 

    // hprintln here can slow enough that transmition is missed
    //hprintln!("   receiving on rx2 ...").unwrap();

    // Read the byte that was just send. Blocks until the read is complete

    let len = match rx2.read(&mut received2) {  
        Ok(length)   =>  length,
        Err(e) =>  {hprintln!("   rx2.read error {:?}", e).unwrap();    
                    //panic!("panic");
                    999
                   },
    };

    hprintln!("   length received on rx2: {}", len).unwrap();
    hprintln!("   checking tx3 to rx2 received = send,  {:?} = {:?}  byte", received2[0], send).unwrap();

    // The send byte should be the one received
    //assert_eq!(received2[0], send, "testing received2 = send,  {} = {}", received2[0], send);

    hprintln!("      tx3 to rx2[0] characters,  {} = {}",
        from_utf8(&[received2[0]]).unwrap(),
        from_utf8(&[send]).unwrap()
    ).unwrap();

    hprintln!("   sending received from rx2  to console on tx1 ...").unwrap();

    tx1.write_all(b"tx3 to rx2 test with Y\r\n").unwrap();
    tx1.write_all(&received2).unwrap();
    tx1.write_all(b"\r\n").unwrap();

    // Trigger a breakpoint to inspect the values
    //asm::bkpt();

    hprintln!("entering empty loop. ^C to exit.").unwrap();
    loop {}
}
