//! Read GPS on usart serial interface and display on OLED with i2c.
//! GPS longitude and latitude in 100ths of a degree (taken as characters
//! from GPS messages without any conversion).
//! Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display
//! Compare this example with gps_rw, lora_gps and text_i2c.

// Example use of impl trait: If scl and sda are on PB10 and PB11 (eg in stm32f1xx below) then
//    fn setup() ->  (Tx<USART3>, Rx<USART3>,
//                    BlockingI2c<I2C2,  (PB10<Alternate<OpenDrain>>, PB11<Alternate<OpenDrain>>) >,
//                    Delay )  {
// is changed to
//    fn setup() ->  (Tx<USART2>, Rx<USART2>,
//                    BlockingI2c<I2C2, impl Pins<I2C2>>,
//                    Delay )  {
// Also
//   use stm32f1xx_hal::{ gpio::{gpiob::{PB10, PB11}, Alternate, OpenDrain, },
// will be needed.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

//use core::fmt::Write;  // for writeln
use cortex_m_semihosting::hprintln;
use nb::block;

//use embedded_hal::blocking::delay::DelayMs;

//builtin include FONT_6X10, FONT_8X13, ....
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg  stm32f030xc  stm32f042
use stm32f0xx_hal::{
    delay::Delay,
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    serial::{Rx, Serial, Tx},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);
    let gpiob = p.GPIOB.split(&mut rcc);

    let (tx2, rx2, scl, sda) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa2.into_alternate_af1(cs), //tx pa2
            gpioa.pa3.into_alternate_af1(cs), //rx pa3
            gpiob.pb8.into_alternate_af1(cs), // scl on PB8
            gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });

    let (tx2, rx2) = Serial::usart2(p.USART2, (tx2, rx2), 9600.bps(), &mut rcc).split();

    let i2c = I2c::i2c1(p.I2C1, (scl, sda), 400.khz(), &mut rcc);

    (tx2, rx2, i2c, Delay::new(cp.SYST, &rcc))
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    delay::Delay,
    device::I2C2,
    device::USART2,
    i2c::{BlockingI2c, DutyCycle, Mode, Pins},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    serial::{Config, Rx, Serial, Tx}, //, StopBits
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    BlockingI2c<I2C2, impl Pins<I2C2>>,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut afio = p.AFIO.constrain();

    let mut gpioa = p.GPIOA.split();
    let mut gpiob = p.GPIOB.split();

    let (tx, rx) = Serial::usart2(
        p.USART2,
        (
            gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl), //tx pa2  for GPS
            gpioa.pa3,                                          //rx pa3  for GPS
        ),
        &mut afio.mapr,
        Config::default().baudrate(9_600.bps()),
        clocks,
    )
    .split();

    let i2c = BlockingI2c::i2c2(
        p.I2C2,
        (
            gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh), // scl on PB10
            gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh), // sda on PB11
        ),
        //&mut afio.mapr,  need this for i2c1 but not i2c2
        Mode::Fast {
            frequency: 400_000.hz(),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    (tx, rx, i2c, Delay::new(cp.SYST, clocks))
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    serial::{Rx, RxPin, Serial, Tx, TxPin},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Tx<USART2, impl TxPin<USART2>>,
    Rx<USART2, impl RxPin<USART2>>,
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);

    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa
                .pa2
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2 for GPS rx
            gpioa
                .pa3
                .into_af7_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3 for GPS tx
        ),
        9600.Bd(), // 115_200.Bd(),
        clocks,
        &mut rcc.apb1,
    )
    .split();

    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);

    let scl = gpiob
        .pb8
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // scl on PB8
    let sda = gpiob
        .pb9
        .into_af4_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // sda on PB9

    (
        tx2,
        rx2,
        I2c::new(p.I2C1, (scl, sda), 400_000.Hz(), clocks, &mut rcc.apb1), // i2c
        Delay::new(cp.SYST, clocks),
    )
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64, blackpills stm32f401 and stm32f411
use stm32f4xx_hal::{
    delay::Delay,
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C2, USART2},
    prelude::*,
    serial::{config::Config, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (Tx<USART2>, Rx<USART2>, I2c<I2C2, impl Pins<I2C2>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.freeze();
    let gpioa = p.GPIOA.split();

    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa.pa2.into_alternate(), //tx pa2  for GPS rx
            gpioa.pa3.into_alternate(),
        ), //rx pa3  for GPS tx
        Config::default().baudrate(9600.bps()),
        clocks,
    )
    .unwrap()
    .split();

    let gpiob = p.GPIOB.split();

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    //BlockingI2c::i2c2(
    let scl = gpiob.pb10.into_alternate().set_open_drain(); // scl on PB10
    let sda = gpiob.pb3.into_alternate().set_open_drain(); // sda on PB3

    (
        tx2,
        rx2,
        I2c::new(p.I2C2, (scl, sda), 400.khz(), clocks), // i2c
        Delay::new(cp.SYST, &clocks),
    )
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    i2c::{BlockingI2c, Mode, PinScl, PinSda},
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    serial::{Config, Oversampling, Rx, Serial, Tx},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();

    let gpioa = p.GPIOA.split();

    let (tx2, rx2) = Serial::new(
        p.USART2,
        (
            gpioa.pa2.into_alternate_af7(), //tx pa2  for GPS rx
            gpioa.pa3.into_alternate_af7(),
        ), //rx pa3  for GPS tx
        clocks,
        Config {
            baud_rate: 9600.Bps(),
            oversampling: Oversampling::By16,
            character_match: None,
        },
    )
    .split();

    let gpiob = p.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain(); // sda on PB9

    (
        tx2,
        rx2,
        BlockingI2c::i2c1(
            p.I2C1,
            (scl, sda),
            Mode::standard(400_000.Hz()),
            clocks,
            &mut rcc.apb1,
            1000,
        ), // i2c
        Delay::new(cp.SYST, clocks),
    )
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    serial::{Rx, Tx},
};

#[cfg(feature = "stm32h7xx")]
fn setup() -> (Tx<USART2>, Rx<USART2>, I2c<I2C1>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = p.GPIOA.split(ccdr.peripheral.GPIOA);

    let (tx2, rx2) = p
        .USART2
        .serial(
            (
                gpioa.pa2.into_alternate_af7(), //tx pa2 for GPS rx
                gpioa.pa3.into_alternate_af7(),
            ), //rx pa3 for GPS tx
            9600.bps(),
            ccdr.peripheral.USART2,
            &clocks,
        )
        .unwrap()
        .split();

    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);

    let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain(); // sda on PB9

    (
        tx2,
        rx2,
        p.I2C1
            .i2c((scl, sda), 400.khz(), ccdr.peripheral.I2C1, &clocks), // i2c
        Delay::new(cp.SYST, clocks),
    )
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    i2c::{I2c, SCLPin, SDAPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, Serial2Ext, Tx},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());

    let gpioa = p.GPIOA.split(&mut rcc);

    let (tx2, rx2) = p
        .USART2
        .usart(
            gpioa.pa2, //tx pa2  for GPS rx
            gpioa.pa3, //rx pa3  for GPS tx
            Config::default().baudrate(9600.Bd()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let gpiob = p.GPIOB.split(&mut rcc);

    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    let i2c = p.I2C1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    //let i2c =I2c::i2c1(p.I2C1, (scl, sda), 400.khz(), rcc.clocks), // i2c

    (tx2, rx2, i2c, Delay::new(cp.SYST, rcc.clocks))
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    serial::{Config, Rx, SerialExt, Tx},
    stm32::{CorePeripherals, Peripherals, I2C1, USART1},
    //gpio::{gpiob::{PB8, PB9}, Output, OpenDrain, },
};

/*
The Heltec lora_node 151 uses USART2 and USART3 pins for on board LoRa connections and power detection.
See https://resource.heltec.cn/download/LoRa_Node_151/LoRa_Node_151_Pinout_Diagram.pdf.
So only USART1 is available. It is used for the GPS.
For simplicity of this example the same setup is used on the Discovery kit stm32l100.
*/

#[cfg(feature = "stm32l1xx")]
fn setup() -> (Tx<USART1>, Rx<USART1>, I2c<I2C1, impl Pins<I2C1>>, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());

    let gpioa = p.GPIOA.split();

    let (tx, rx) = p
        .USART1
        .usart(
            (
                gpioa.pa9, //tx pa9   for GPS rx
                gpioa.pa10,
            ), //rx pa10  for GPS tx
            Config::default().baudrate(9600.bps()),
            &mut rcc,
        )
        .unwrap()
        .split();

    let gpiob = p.GPIOB.split();

    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9

    (
        tx,
        rx,
        p.I2C1.i2c((scl, sda), 400.khz(), &mut rcc), // i2c
        cp.SYST.delay(rcc.clocks),
    ) // delay
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    i2c::{Config as i2cConfig, I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1, USART2},
    prelude::*,
    serial::{Config as serialConfig, Rx, Serial, Tx},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    Tx<USART2>,
    Rx<USART2>,
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut flash = p.FLASH.constrain();
    let mut rcc = p.RCC.constrain();
    let mut pwr = p.PWR.constrain(&mut rcc.apb1r1);
    let clocks = rcc
        .cfgr
        .sysclk(80.mhz())
        .pclk1(80.mhz())
        .pclk2(80.mhz())
        .freeze(&mut flash.acr, &mut pwr);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);
    //         let mut gpiob  = p.GPIOB.split(&mut rcc.ahb2);

    let (tx2, rx2) = Serial::usart2(
        p.USART2,
        (
            gpioa
                .pa2
                .into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //tx pa2  for GPS rx
            gpioa
                .pa3
                .into_af7_pushpull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), //rx pa3  for GPS tx
        ),
        serialConfig::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb1r1,
    )
    .split();

    // following github.com/stm32-rs/stm32l4xx-hal/blob/master/examples/i2c_write.rs

    let mut scl =
        gpioa
            .pa9
            .into_af4_opendrain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // scl on PA9
    scl.internal_pull_up(&mut gpioa.pupdr, true);

    let mut sda =
        gpioa
            .pa10
            .into_af4_opendrain(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh); // sda on PA10
    sda.internal_pull_up(&mut gpioa.pupdr, true);

    (
        tx2,
        rx2,
        I2c::i2c1(
            p.I2C1,
            (scl, sda),
            i2cConfig::new(400.khz(), clocks),
            &mut rcc.apb1r1,
        ),
        Delay::new(cp.SYST, clocks),
    )
}

// End of hal/MCU specific setup. Following should be generic code.

fn to_str(x: &[u8]) -> &str {
    match core::str::from_utf8(x) {
        Ok(str) => &str,
        Err(_error) => "problem converting u8 to str ",
    }
}

#[entry]

fn main() -> ! {
    let (mut _tx_gps, mut rx_gps, i2c, mut delay) = setup(); //  GPS, i2c, delay

    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    // A symptom of improper DisplaySize setting can be clipping font on top and/or bottom.

    //builtin include Font6x6, Font6x8, Font6x12, Font8x16, Font12x16, Font24x32
    // printing 14 characters, font width must be less than 9  (128/14)
    // Font6x6   extremely small.
    // Font6x8   very small.
    // Font6x12  clear but small.
    // Font8x16  good.
    // Font12x16 too wide for 128x displays.
    // Font24x32 too wide and too high for two lines. Causes panic.

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    let mut line1 = Text::new("----", Point::zero(), text_style);
    let mut line2 = Text::new("----", Point::new(0, 20), text_style);

    line1.draw(&mut display).unwrap();
    line2.draw(&mut display).unwrap();
    display.flush().unwrap();

    delay.delay_ms(2000_u16);

    // Would this approach in loop give smaller code? or faster?
    // Need to avoid  mutable/immutable borrow.
    line1.text = "xxxx";
    line1.draw(&mut display).unwrap();
    line2.text = "zzzz";
    line2.draw(&mut display).unwrap();
    display.flush().unwrap();
    delay.delay_ms(1000_u16);

    // byte buffer length 80
    let mut buffer: heapless::Vec<u8, 80> = heapless::Vec::new();
    //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();  //0 of 80
    buffer.clear();

    let e: u8 = 9;
    let mut good = false;
    //let mut size: usize = 0;

    //asm::bkpt();

    loop {
        let byte = match block!(rx_gps.read()) {
            Ok(byt) => byt,
            Err(_error) => e,
        };
        //hprintln!("{}", byte).unwrap();
        if byte == 36 {
            //  $ is 36. start of a line
            buffer.clear();
            good = true; //start capturing line
        };
        if good {
            if buffer.push(byte).is_err() || byte == 13 {
                //end of line. \r is 13, \n is 10

                //hprintln!("buffer at {} of {}", buffer.len(), buffer.capacity()).unwrap();
                //hprintln!("read buffer {:?}", to_str(&buffer)).unwrap();

                //if buffer[0..6] == [36, 71, 80, 84, 88, 84]   //$GPTXT
                //if buffer[0..6] == [36, 71, 80, 82, 77, 67]   //$GPRMC

                //$GPGLL north ~ to_str(&buffer[7..19])  east ~ to_str(&buffer[19..33])
                //$GPRMC north = to_str(&buffer[19..31]) east = to_str(&buffer[32..45])

                //if to_str(&buffer[0..6]) == "$GPRMC"           // message id
                if &buffer[0..6] == [36, 71, 80, 82, 77, 67] {
                    // message id $GPRMC
                    hprintln!("{}", to_str(&buffer[..])).unwrap();
                    let north = to_str(&buffer[19..31]);
                    hprintln!("north {}", north).unwrap();
                    let east = to_str(&buffer[32..45]);
                    hprintln!("east {}", east).unwrap();
                    Text::new(north, Point::new(0, 0), text_style)
                        .draw(&mut display)
                        .unwrap();
                    Text::new(east, Point::new(0, 20), text_style)
                        .draw(&mut display)
                        .unwrap();
                    display.flush().unwrap();
                };

                buffer.clear();
                good = false;
                //delay.delay(4000.ms());
                delay.delay_ms(4000_u16);
            };
        };
    }
}
