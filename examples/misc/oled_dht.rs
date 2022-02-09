//!  MAYBE CONVERT THIS TO RTIC
//!  Measure the temperature and humidity from a DHT11 or DHT22 on data pin (A8) and display on OLED with i2c.
//!  (Specify feature "dht22"for DHT22).
//!  Compare examples dht, oled_gps, and temperature_display which have more comments.
//!  Note that the DisplaySize setting needs to be adjusted for 128x64 or 128x32 display
//!  Note that '--release' is needed when doing a run test on actual hardware. Otherwise
//!  code is too slow for the timeout set in the crate and run gives 'Error Timeout'.

#![deny(unsafe_code)]
#![no_main]
#![no_std]

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

//use nb::block;
use core::fmt::Write;
//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

//https://github.com/michaelbeaumont/dht-sensor
#[cfg(not(feature = "dht22"))]
use dht_sensor::dht11::Reading;
#[cfg(feature = "dht22")]
use dht_sensor::dht22::Reading;
use dht_sensor::*;

// See https://docs.rs/embedded-graphics/0.7.1/embedded_graphics/mono_font/index.html
// DisplaySize128x32:
//    &FONT_6X10 128 pixels/ 6 per font = 21.3 characters wide.  32/10 = 3.2 characters high
//    &FONT_5X8  128 pixels/ 5 per font = 25.6 characters wide.  32/8  =  4  characters high
//    FONT_8X13  128 pixels/ 8 per font = 16   characters wide.  32/13 = 2.5 characters high
//    FONT_9X15  128 pixels/ 9 per font = 14.2 characters wide.  32/15 = 2.  characters high
//    FONT_9X18  128 pixels/ 9 per font = 14.2 characters wide.  32/18 = 1.7 characters high
//    FONT_10X20 128 pixels/10 per font = 12.8 characters wide.  32/20 = 1.6 characters high

use embedded_graphics::{
    //mono_font::{ascii::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    mono_font::{iso_8859_1::FONT_10X20, MonoTextStyleBuilder, MonoTextStyle}, 
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306, mode::BufferedGraphicsMode};

use rust_integration_testing_of_examples::i2c_led_delay::{setup_led, LED};

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg  stm32f030xc  stm32f042
use stm32f0xx_hal::{
    delay::Delay,
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    PA8<Output<OpenDrain>>,
    I2c<I2C1, impl SclPin<I2C1>, impl SdaPin<I2C1>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let mut p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);
    let mut delay = Delay::new(cp.SYST, &rcc);

    let gpioa = p.GPIOA.split(&mut rcc);
    let gpiob = p.GPIOB.split(&mut rcc);

    let (mut dht, scl, sda) = cortex_m::interrupt::free(move |cs| {
        (   gpioa.pa8.into_open_drain_output(cs), // dht on PA8
            gpiob.pb8.into_alternate_af1(cs), // scl on PB8
            gpiob.pb7.into_alternate_af1(cs), // sda on PB7
        )
    });

    dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

    let i2c = I2c::i2c1(p.I2C1, (scl, sda), 400.khz(), &mut rcc);

    let led = setup_led(p.GPIOC.split(&mut rcc));

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32f1xx")] //  eg blue pill stm32f103
use stm32f1xx_hal::{
    delay::Delay,
    device::I2C2,
    i2c::{BlockingI2c, DutyCycle, Mode, Pins},
    pac::{CorePeripherals, Peripherals},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (PA8<Output<OpenDrain>>, BlockingI2c<I2C2, impl Pins<I2C2>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpioa = p.GPIOA.split();

    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.crh);
    dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

    let mut gpiob = p.GPIOB.split();

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

    let led = setup_led(p.GPIOC.split());
    
    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    i2c::{I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (PA8<Output<OpenDrain>>,
    I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.freeze(&mut p.FLASH.constrain().acr);
    let mut delay = Delay::new(cp.SYST, clocks);

    let mut gpioa = p.GPIOA.split(&mut rcc.ahb);

    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization


    let mut gpiob = p.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob
        .pb8
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // scl on PB8
    let sda = gpiob
        .pb9
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh); // sda on PB9
    let i2c = I2c::new(p.I2C1, (scl, sda), 400_000.Hz(), clocks, &mut rcc.apb1);

    let led = setup_led(p.GPIOE.split(&mut rcc.ahb));

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32f4xx")] // eg Nucleo-64, blackpills stm32f401 and stm32f411
use stm32f4xx_hal::{
    delay::Delay,
    i2c::{I2c, Pins},
    pac::{CorePeripherals, Peripherals, I2C2},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32f4xx")]
fn setup() -> (PA8<Output<OpenDrain>>, I2c<I2C2, impl Pins<I2C2>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let clocks = p.RCC.constrain().cfgr.freeze();
    let mut delay =  Delay::new(cp.SYST, &clocks);

    let mut dht = p.GPIOA.split().pa8.into_open_drain_output();
    dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

    let gpiob = p.GPIOB.split();

    // can have (scl, sda) using I2C1  on (PB8  _af4, PB9 _af4) or on  (PB6 _af4, PB7 _af4)
    //     or   (scl, sda) using I2C2  on (PB10 _af4, PB3 _af9)

    //BlockingI2c::i2c2(
    let scl = gpiob.pb10.into_alternate().set_open_drain(); // scl on PB10
    let sda = gpiob.pb3.into_alternate().set_open_drain(); // sda on PB3

    let i2c   =  I2c::new(p.I2C2, (scl, sda), 400.khz(), &clocks);

    let led = setup_led(p.GPIOC.split());

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    i2c::{BlockingI2c, Mode, PinScl, PinSda},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (PA8<Output<OpenDrain>>, BlockingI2c<I2C1, impl PinScl<I2C1>, impl PinSda<I2C1>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    let mut dht = p.GPIOA.split().pa8.into_open_drain_output();
    dht.set_high();// Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

    let gpiob = p.GPIOB.split();

    let scl = gpiob.pb8.into_alternate_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_open_drain(); // sda on PB9

    let i2c = BlockingI2c::i2c1(p.I2C1, (scl, sda), Mode::standard(400_000.Hz()), clocks, &mut rcc.apb1, 1000);

    let led = setup_led(p.GPIOC.split());

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    i2c::I2c,
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (PA8<Output<OpenDrain>>, I2c<I2C1>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let pwr = p.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = p.RCC.constrain();
    let ccdr = rcc.sys_ck(100.mhz()).freeze(vos, &p.SYSCFG);
    let clocks = ccdr.clocks;
    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer

    let mut dht = p.GPIOA.split(ccdr.peripheral.GPIOA).pa8.into_open_drain_output();
    dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

    let gpiob = p.GPIOB.split(ccdr.peripheral.GPIOB);
    let scl = gpiob.pb8.into_alternate_af4().set_open_drain(); // scl on PB8
    let sda = gpiob.pb9.into_alternate_af4().set_open_drain(); // sda on PB9
    let i2c = p.I2C1.i2c((scl, sda), 400.khz(), ccdr.peripheral.I2C1, &clocks);

    let led = setup_led(p.GPIOC.split(ccdr.peripheral.GPIOC));

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    i2c::{I2c, SCLPin, SDAPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (PA8<Output<OpenDrain>>, I2c<I2C1, impl SDAPin<I2C1>, impl SCLPin<I2C1>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi16());
    let mut delay = cp.SYST.delay(rcc.clocks);

    let mut dht = p.GPIOA.split(&mut rcc).pa8.into_open_drain_output();
    dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization

    let gpiob = p.GPIOB.split(&mut rcc);
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c = p.I2C1.i2c(sda, scl, 400_000.Hz(), &mut rcc);

    let led = setup_led(p.GPIOC.split(&mut rcc));

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    i2c::{I2c, Pins},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    stm32::{CorePeripherals, Peripherals, I2C1},
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32l1xx")]
fn setup() -> (PA8<Output<OpenDrain>>, I2c<I2C1, impl Pins<I2C1>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let p = Peripherals::take().unwrap();
    let mut rcc = p.RCC.freeze(rcc::Config::hsi());
    let mut delay = cp.SYST.delay(rcc.clocks);

    let mut dht = p.GPIOA.split(&mut rcc).pa8.into_open_drain_output();
    dht.set_high().ok(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16);//  1 second delay for sensor initialization

    let gpiob = p.GPIOB.split(&mut rcc);
    let scl = gpiob.pb8.into_open_drain_output(); // scl on PB8
    let sda = gpiob.pb9.into_open_drain_output(); // sda on PB9
    let i2c = p.I2C1.i2c((scl, sda), 400.khz(), &mut rcc);

    let led = setup_led(gpiob.pb6);

    (dht, i2c, led, delay)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    i2c::{Config as i2cConfig, I2c, SclPin, SdaPin},
    pac::{CorePeripherals, Peripherals, I2C1},
    prelude::*,
    gpio::{gpioa::PA8, OpenDrain, Output},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (PA8<Output<OpenDrain>>, I2c<I2C1, (impl SclPin<I2C1>, impl SdaPin<I2C1>)>, impl LED, Delay) {
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

    let mut delay = Delay::new(cp.SYST, clocks); //SysTick: System Timer
    let mut gpioa = p.GPIOA.split(&mut rcc.ahb2);

    let mut dht = gpioa.pa8.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    dht.set_high(); // Pull high to avoid confusing the sensor when initializing.
    delay.delay_ms(1000_u16); //  1 second delay for sensor initialization


    // following github.com/stm32-rs/stm32l4xx-hal/blob/master/examples/i2c_write.rs
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

    let i2c = I2c::i2c1(p.I2C1, (scl, sda), i2cConfig::new(400.khz(), clocks), &mut rcc.apb1r1);

    let led = setup_led(p.GPIOC.split(&mut rcc.ahb2));

    (dht, i2c, led, delay)
}

// End of hal/MCU specific setup. Following should be generic code.

fn show_display<S>(
    temperature: i8,
    relative_humidity: u8,
    text_style: MonoTextStyle<BinaryColor>,
    disp: &mut Ssd1306<impl WriteOnlyDataCommand, S, BufferedGraphicsMode<S>>,
) -> ()
where
    S: DisplaySize,
{
    let mut lines: [heapless::String<32>; 1] = [
        heapless::String::new(),
    ];

    // Many SSD1306 modules have a yellow strip at the top of the display, so first line may be yellow.
    // It is possible to use \n in place of separate writes, with one line rather than vector.

    // UTF-8 text is 2 bytes (2 ascii characters) in strings like the next. Cutting an odd number of character from
    // the next test_text can result in a build error message  `stream did not contain valid UTF-8` even with
    // the line commented out!! The test_txt is taken from 
    //      https://github.com/embedded-graphics/examples/blob/main/eg-0.7/examples/text-extended-characters.rs
    
    //let test_text  = "¡¢£¤¥¦§¨©ª«¬­®¯°±²³´µ¶·¸¹º»¼½¾¿ÀÁÂÃÄÅÆÇÈÉÊËÌÍÎÏÐÑÒÓÔÕÖ×ØÙÚÛÜÝÞßàáâãäåæçèéêëìíîïðñòóôõö÷øùúûüýþÿ";
    //   degree symbol "°" is about                  ^^ here 
    
    write!(lines[0], "{:3}°C {:3}% RH", temperature, relative_humidity).unwrap();
 
    disp.clear();
    for i in 0..lines.len() {
        // start from 0 requires that the top is used for font baseline
        Text::with_baseline(
            &lines[i],
            Point::new(0, i as i32 * 12), //with font 6x10, 12 = 10 high + 2 space
            text_style,
            Baseline::Top,
        )
        .draw(&mut *disp)
        .unwrap();
    }
    disp.flush().unwrap();
    ()
}


#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("oled_dht example");
    //hprintln!("oled_dht example").unwrap();

    let (mut dht, i2c, mut led, mut delay) = setup();

    led.blink(500_u16, &mut delay);  // to confirm startup

    let manager = shared_bus::BusManagerSimple::new(i2c);
    let interface = I2CDisplayInterface::new(manager.acquire_i2c());

    //let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
    let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_10X20)
        .text_color(BinaryColor::On)
        .build();

    loop {
        // Blink LED to check that everything is actually running.
        led.blink(50_u16, &mut delay);

        match Reading::read(&mut delay, &mut dht) {
            Ok(Reading {
                temperature,
                relative_humidity,}) 
               => {//hprintln!("{} deg C, {}% RH", temperature, relative_humidity).unwrap();
                   show_display(temperature, relative_humidity, text_style, &mut display)},
            Err(_e) 
               =>  {//hprintln!("Error {:?}", e).unwrap(); 
                    panic!("Error reading DHT")},
        }

        // (Delay at least 500ms before re-polling DHT, 1 second or more advised)
        delay.delay_ms(2000_u16); // Delay 2 seconds
    }
}
