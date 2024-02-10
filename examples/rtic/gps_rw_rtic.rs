//!   NOT HARDWARE TESTED SINCE EMBEDDED-HAL V1.0.0 CHANGES
//!
//! Two UARTS, no external device crates

//! Serial interface read GPS one usart and write on another usart to USB-TTL console (minicom).
//!
//! usart1 connect the Tx pin pa9  to the Rx pin of a serial-usb converter
//! usart1 connect the Rx pin pa10 to the Tx pin of a serial-usb converter
//! Set up the serial console (e.g. minicom) with the same settings used here.
//! (Using 9600bps, could be higher but needs serial console to be the same.)
//!
//! GPS uses 9600bps, 8bit, odd parity, 1 stopbit. This can be confirmed by connecting GPS
//!  directly to the  USB-TTL and terminal with these settings (minicom 8-N-1)
//! The usart and pins for the GPS depend on the board. For specifics see setup_from_dp() sections below.
//!
//! See examples/serial_char.rs for notes about connecting usart1 to
//!   serial-usb converter on computer for console output.
//! That file also has for more notes regarding setup below.

#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;



use rtic::app;

#[cfg_attr(feature = "stm32f0xx", app(device = stm32f0xx_hal::pac,   dispatchers = [ TIM3 ]))]
#[cfg_attr(feature = "stm32f1xx", app(device = stm32f1xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f3xx", app(device = stm32f3xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f4xx", app(device = stm32f4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32f7xx", app(device = stm32f7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g0xx", app(device = stm32g0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32g4xx", app(device = stm32g4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32h7xx", app(device = stm32h7xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l0xx", app(device = stm32l0xx_hal::pac,   dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l1xx", app(device = stm32l1xx_hal::pac, dispatchers = [TIM2, TIM3]))]
#[cfg_attr(feature = "stm32l4xx", app(device = stm32l4xx_hal::pac,   dispatchers = [TIM2, TIM3]))]

mod app {
    //use core::fmt::Write;
    use cortex_m_semihosting::{hprintln};
    
    use rtic;
    use rtic_monotonics::systick::Systick;
    //use rtic_monotonics::systick::fugit::{ExtU32};

    //use nb::block;
    //use rtt_target::{rprintln, rtt_init_print};

    use rust_integration_testing_of_examples::stm32xxx_as_hal::hal;

    pub use hal::{
        pac::Peripherals,
        pac::{USART1, USART2},
        serial::{Serial, Tx, Rx, Error},
        prelude::*,
    };

    use rust_integration_testing_of_examples::monoclock::MONOCLOCK;

    use rust_integration_testing_of_examples::tx1_rx1_tx2_rx2;
    use rust_integration_testing_of_examples::tx1_rx1_tx2_rx2::{Tx1Type, Rx2Type};

    use embedded_io::{Read, Write};

//    fn writeln(con: &mut Tx<USART1>, buf: &str) -> () {
//        for byte in buf.bytes() {write_b(con, byte)};
//    ()
//    }
//    
//    fn write_b(con: &mut Tx<USART1>, b: u8) -> () {
//            #[cfg(feature = "stm32f4xx")]
//            block!(con.write(b)).ok();
//            #[cfg(not(feature = "stm32f4xx"))]
//            block!(con.write_byte(b)).ok();
//    ()
//    }
//    
//    fn read_b<E>(con: &mut Rx<USART2>) -> Result<u8, Error> {
//            #[cfg(feature = "stm32f4xx")]
//            let b = block!(con.read());
//            #[cfg(not(feature = "stm32f4xx"))]
//            let b = block!(con.read_byte());
//    b
//    }

    //something like this might also work
    //fn read_b(con: &mut Rx<USART2>) -> Result<u8, Error> {
    //        #[cfg(feature = "stm32f4xx")]
    //        let b = con.read();
    //        #[cfg(not(feature = "stm32f4xx"))]
    //        let b = con.read_byte();
    //b
    //}


    #[shared]
    struct Shared {
        //led: LedType,
        //buffer: [u8; 80],
        buffer: heapless::Vec<u8, 80>,
    }

    #[local]
    struct Local {
        tx_con: Tx1Type, 
        rx_gps: Rx2Type,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local ) {
        let mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, MONOCLOCK, mono_token);

        // byte buffer up to 80  u8 elements on stack
        //let mut buffer: [u8; 80] = [0; 80];
        let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
        hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap(); //0 of 80
        buffer.clear();

        let dp = Peripherals::take().unwrap();
        let (mut tx_con, mut _rx_con, mut _tx_gps, rx_gps) = tx1_rx1_tx2_rx2::setup_from_dp(dp); // console, GPS

        //writeln(&mut tx_con, "\r\nconsole connect check.\r\n");
        tx_con.write("\r\nconsole connect check.\r\n".as_bytes()).unwrap();   // does this need block!() ?

        // read gps on usart2
        //    while (i < r.len()) && !buffer.push(r[i]).is_err() {

        // note that putting hprintln! in loop slows it too much and loses data.
    
    // CLEANUP
//    let _e: u8 = 9;
//    let mut good = false;
 
        // read gps on usart2
        hprintln!("starting read GPS").unwrap();
        read_gps::spawn().unwrap();

        (Shared {buffer}, Local {tx_con, rx_gps} )
    }

    #[task(shared = [buffer], local = [rx_gps])]
    async fn read_gps(cx: read_gps::Context) {
        let mut buffer = cx.shared.buffer;
        let rx_gps     = cx.local.rx_gps;
        //let mut tx_con = cx.local.tx_con;
       
//        let e: u8 = 9;                                        // crude error indicator
//        let mut good = false;
        loop {
            let _len = buffer.lock(|buf| {rx_gps.read(buf)});
            //let byte = match read_b::<Error>(&mut rx_gps) {
            //    Ok(byt) => byt,
            //    Err(_error) => e,
            //};

            //tx_con.write(&buffer).unwrap();  // echo everything to console
            write_con::spawn().unwrap(); 
          
            //  THE LOGIC OF THIS NEEDS TO BE FIXED
//            let byte= 35;  //fake
//            if byte == 36 { //  $ is 36. start of a line      // watch for start of line
//                buffer.lock(|buf| {buf.clear(); 
//                                   let _ = buf.push(byte);
//                                   }
//                );  
//                good = true; //start capturing line
//            };
//
//            if good {                                       
//                if buffer.lock(|buf| buf.push(byte)).is_err() || byte == 13 { // watch for end of line, \r is 13, \n is 10
//                      write_con::spawn().unwrap(); 
//                      //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();
//                      buffer.lock(|buf| buf.clear() );
//                      good = false;
//                };
//            };
        }
    }

    #[task(shared = [ buffer], local = [tx_con])]
    async fn write_con(cx: write_con::Context) {
        let mut buffer = cx.shared.buffer;
        let tx_con = cx.local.tx_con;

        //write_b(&mut tx_con, byte);
        //for byte in &cx.shared.buffer.lock(|buffer| {write_b(&mut &cx.local.tx_con, *byte) });
        //let _ = &cx.shared.buffer.lock(|buf| for byte in buf {write_b(cx.local.tx_con, *byte) });
        let _ = buffer.lock(|buf| {tx_con.write(&buf).unwrap()});
        ()
    }
}
