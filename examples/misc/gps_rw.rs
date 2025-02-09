//!  Tested on weact-stm32g474 July 12, 2024.
//!  Works, but looping is so fast that read buffer only ever has 0 or 1 length.
//!  This does not give a very good test of buffered read and it is possible that chars are lost.

//! FAILS TO COMPILE WITH stm32f4xx_hal. See
//! https://github.com/stm32-rs/stm32f4xx-hal/issues/721

//! Serial interface read GPS one usart and write on another usart to USB-TTL console (minicom).
//!
//! usart1 connect the Tx pin (pa9  on bluepill) to the Rx pin of a serial-usb converter
//! usart1 connect the Rx pin (pa10 on bluepill) to the Tx pin of a serial-usb converter

//! Set up the serial console (e.g. minicom) with the same settings used here.
//! (Using 9600bps, could be higher but needs serial console to be the same.)
//!
//! GPS uses 9600bps, 8bit, odd parity, 1 stopbit. This can be confirmed by connecting GPS
//!  directly to the  USB-TTL and terminal with these settings (minicom 8-N-1)
//! The usart and pins for the GPS depend on the board. For specifics see setup() sections below.
//!
//! See examples/serial_char.rs for notes about connecting usart1 to
//!   serial-usb converter on computer for console output.
//! That file also has for more notes regarding setup below.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
//use core::str;
//use core::ascii;
//use nb::block;

// see  embedded-hal/embedded-io/src/lib.rs re Read, Write, BufRead
use embedded_io::{Read, Write};

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] // eg stm32f030xc  stm32f042
use stm32f0xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{Rx, Serial, Tx},
};

#[cfg(feature = "stm32f0xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
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
use stm32f1xx_hal::{
    pac::{Peripherals, USART1, USART3},
    prelude::*,
    serial::{Config, Rx, Serial, StopBits, Tx},
};

#[cfg(feature = "stm32f1xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART3>, Rx<USART3>) {
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
use stm32f3xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup_from_dp(dp: Peripherals) -> (
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
use stm32f4xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx, Error},
};

#[cfg(feature = "stm32f4xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
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
use stm32f7xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{Config, Oversampling, Rx, Serial, Tx, DataBits, Parity},
};

#[cfg(feature = "stm32f7xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
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
use stm32g0xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{FullConfig, Tx, Rx},
};

#[cfg(feature = "stm32g0xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1, FullConfig>, Rx<USART1, FullConfig>, Tx<USART2, FullConfig>, Rx<USART2, FullConfig>) {
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
use stm32g4xx_hal::{
    pac::{Peripherals, USART1, USART2},
    prelude::*,
    serial::{FullConfig, Rx, Tx, NoDMA},
    gpio::{Alternate, gpioa::{PA2, PA3, PA9, PA10}},
};

#[cfg(feature = "stm32g4xx")]
fn setup_from_dp(dp: Peripherals) -> (
               Tx<USART1, PA9<Alternate<7_u8>>, NoDMA>, Rx<USART1, PA10<Alternate<7_u8>>, NoDMA>,
               Tx<USART2, PA2<Alternate<7_u8>>, NoDMA>, Rx<USART2, PA3<Alternate<7_u8>>,  NoDMA>
) {
    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split(&mut rcc);

    let (tx1, rx1) = dp.USART1.usart(   //tx, rx  for console
       gpioa.pa9.into_alternate(), 
       gpioa.pa10.into_alternate(), 
       FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

    let (tx2, rx2) = dp.USART2.usart(   //tx, rx  for GPS
        gpioa.pa2.into_alternate(), 
        gpioa.pa3.into_alternate(),
        FullConfig::default().baudrate(9600.bps()), &mut rcc).unwrap().split();

    (tx1, rx1, tx2, rx2)
}



#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{Rx, Tx},
};

#[cfg(feature = "stm32h7xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
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
use stm32l0xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, Serial1Ext, Serial2Ext, Tx},
};

#[cfg(feature = "stm32l0xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
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
use stm32l1xx_hal::{
    prelude::*,
    rcc, // for ::Config but note name conflict with next
    serial::{Config, Rx, SerialExt, Tx},
    stm32::Peripherals,
    stm32::{USART1, USART2},
};

// The Heltec lora_node 151 uses USART2 and USART3 pins for on board LoRa connections and power
// detection. See
// https://resource.heltec.cn/download/LoRa_Node_151/LoRa_Node_151_Pinout_Diagram.pdf.
// So only USART1 is available and this example cannot work on Heltec lora_node 151 as
// it needs 2 USARTs. USART1 is used for the GPS as oled_gps and lora_gps examples might work.
// For simplicity of this example the same setup is used on the Discovery kit stm32l100.

#[cfg(feature = "stm32l1xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART2>, Rx<USART2>, Tx<USART1>, Rx<USART1>) {
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
use stm32l4xx_hal::{
    pac::Peripherals,
    pac::{USART1, USART2},
    prelude::*,
    serial::{Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
fn setup_from_dp(dp: Peripherals) -> (Tx<USART1>, Rx<USART1>, Tx<USART2>, Rx<USART2>) {
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

// End of hal/MCU specific setup. Following should be generic code.



#[entry]

fn main() -> ! {

    // byte buffer up to 80  u8 elements on stack
    let mut buffer_r: [u8; 80] = [0; 80];

    //let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
    //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap(); //0 of 80
    //buffer.clear();

    let dp = Peripherals::take().unwrap();
    let (mut tx_con, mut _rx_con, mut _tx_gps, mut rx_gps) = setup_from_dp(dp); // console, GPS

    let gt: u8 = 62;
    tx_con.write_all(&[gt, gt, gt]).unwrap();
    tx_con.write_all("\r\nconsole connect check.\r\n\r\n".as_bytes()).unwrap();  
    tx_con.write_all(&[gt]).unwrap();

    // read gps on usart2
    hprintln!("about to read GPS").unwrap();
    hprintln!("going into GPS read/write loop ^C to exit ...").unwrap();

    // note that putting hprintln! in loop slows it too much and loses data.
     
    loop {
        //buffer.clear();
        // see https://docs.rs/embedded-io/latest/embedded_io/trait.Read.html
        // Previously this was byte by byte. Now, embedded-hal 1.0 support read() for a buffer.
        // But the GPS serial is slow so the buffer usually has 0 or 1 char in each pass of the loop.

        // See https://github.com/stm32-rs/stm32f4xx-hal/issues/721
        let len = embedded_io::Read::read(&mut rx_gps, &mut buffer_r);
        //let len = rx_gps.read(&mut buffer_r);
        // hprintln!(" buffer_r {:?}",  buffer_r).unwrap();
        // hprintln!(" buffer {:?}",  buffer).unwrap();
        // hprintln!(" buffer len {:?}",  buffer.len()).unwrap();

        match len {
          Ok(length) => {  if length != 0 {
                              //if buffer[0]  == 36  {//  $ is 36. start of a line (sentence) in GPS protocol
                              if buffer_r[0]  == 36  {//  $ is 36. start of a line (sentence) in GPS protocol
                                 //tx_con.write(&[gt]).unwrap();  // show > for new sentence
                                 embedded_io::Write::write(&mut tx_con, &[gt]).unwrap();  // show > for new sentence
                              };
                              // possibly write_all() should be used, but something does not work.
                              // With write() part of the buffer may be lost?
                              //tx_con.write(&buffer_r).unwrap(); // echo everything to console
                              embedded_io::Write::write(&mut tx_con, &buffer_r).unwrap(); // echo everything to console
                              //let _ = tx_con.flush();
                              let _ = embedded_io::Write::flush(&mut tx_con);
                              //tx_con.write_all(&buffer).unwrap(); // echo everything to console
                           };
                        },
          Err(_error) =>  {//tx_con.write("x".as_bytes()).unwrap(); // would be good to have actual error rather than assume overrun
                           // tx_con.write_all("Overrun\r\n".as_bytes()).unwrap(); // would be good to have actual error rather than assume overrun
                           //hprintln!("{:?}", error).unwrap(); //this is so slow it causes overrun in next pass of loop
                          },
       };
    }
}
