//! This example is modified from one at https://github.com/eldruin/driver-examples.
//! The example plays the final part of Beethoven's ninth symphony given by
//! its MIDI tones using an AD9833 waveform generator / direct digital synthesizer.
//!
//! You can see a video of the original version running here:
//! https://blog.eldruin.com/ad983x-waveform-generator-dds-driver-in-rust/
//!
//! This example runs on the STM32F103 "Bluepill" board using  SPI1.
//!
//! ```
//! BP   <-> AD9833  <-> Amplifier
//! GND  <-> VSS     <-> GND
//! 3.3V <-> VDD     <-> VCC
//! PA4  <-> FSYNC
//! PA5  <-> CLK
//! PA7  <-> DAT
//!          OUT     <-> IN
//! ```
//!
//! On other boards the pins may vary. Consult code below for hardware specifics.
//!
//! You will need an amplifier like the PAM8403 or similar and a speaker.
//!
//! Run with:
//! `cargo embed --example ad9833-midi-player`,

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use ad983x::{Ad983x, FrequencyRegister, MODE};

use cortex_m_rt::entry;

use libm;
use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};

pub trait LED {
    // depending on board wiring, `on` may be `set_high` or `set_low`, with `off` also reversed
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

// setup() does all  hal/MCU specific setup and returns generic hal device for use in main code.

#[cfg(feature = "stm32f0xx")] //  eg stm32f030xc
use stm32f0xx_hal::{
    delay::Delay,
    gpio::{
        gpioa::{PA1, PA5, PA6, PA7},
        gpioc::PC13,
        Alternate, Output, PushPull, AF0,
    },
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{EightBit, Spi}, //, SckPin, MisoPin, MosiPin},
};

#[cfg(feature = "stm32f0xx")]
fn setup() -> (
    Spi<SPI1, PA5<Alternate<AF0>>, PA6<Alternate<AF0>>, PA7<Alternate<AF0>>, EightBit>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    //fn setup() -> (Spi<SPI1, impl SckPin<SPI1>, MisoPin<SPI1>, MosiPin<SPI1>, EightBit>, PA1<Output<PushPull>>, impl LED, Delay ) {
    //fn setup() -> (Spi<SPI1, impl Pins<SPI1>, MisoPin<SPI1>, MosiPin<SPI1>, EightBit>, PA1<Output<PushPull>>, impl LED, Delay ) {
    //fn setup() -> (Spi<SPI1, impl Pins<SPI1>, EightBit>, PA1<Output<PushPull>>, impl LED, Delay ) {
    let cp = CorePeripherals::take().unwrap();
    let mut dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

    let (sck, miso, mosi, chs, led) = cortex_m::interrupt::free(move |cs| {
        (
            gpioa.pa5.into_alternate_af0(cs),     //   sck   on PA5
            gpioa.pa6.into_alternate_af0(cs),     //   miso  on PA6
            gpioa.pa7.into_alternate_af0(cs),     //   mosi  on PA7
            gpioa.pa1.into_push_pull_output(cs),  // cs   on PA1
            gpioc.pc13.into_push_pull_output(cs), // led
        )
    });

    let spi = Spi::spi1(dp.SPI1, (sck, miso, mosi), MODE, 8.mhz(), &mut rcc);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let delay = Delay::new(cp.SYST, &rcc);

    (spi, chs, led, delay)
}

#[cfg(feature = "stm32f1xx")]
use stm32f1xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA4, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi, Spi1NoRemap},
};

#[cfg(feature = "stm32f1xx")]
fn setup() -> (
    Spi<SPI1, Spi1NoRemap, impl Pins<Spi1NoRemap>, u8>,
    PA4<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain();
    let mut gpioa = dp.GPIOA.split();

    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);
    let mut cs = gpioa.pa4.into_push_pull_output(&mut gpioa.crl);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        MODE,
        1_u32.mhz(),
        clocks,
    );

    cs.set_high();

    let mut gpioc = dp.GPIOC.split();
    let led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let delay = Delay::new(cp.SYST, clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32f3xx")] //  eg Discovery-stm32f303
use stm32f3xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioe::PE15, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32f3xx")]
fn setup() -> (
    Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>), u8>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .freeze(&mut dp.FLASH.constrain().acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
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

    let cs = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    let led = gpioe
        .pe15
        .into_push_pull_output(&mut gpioe.moder, &mut gpioe.otyper);

    impl LED for PE15<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let delay = Delay::new(cp.SYST, clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32f4xx")]
// eg Nucleo-64 stm32f411, blackpill stm32f411, blackpill stm32f401
use stm32f4xx_hal::{
    timer::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{Pins, Spi, TransferModeNormal},
 };

#[cfg(feature = "stm32f4xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>, TransferModeNormal>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(64.MHz()).pclk1(32.MHz()).freeze();

    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let spi = Spi::new(
        dp.SPI1,
        (
            gpioa.pa5.into_alternate(), // sck   on PA5
            gpioa.pa6.into_alternate(), // miso  on PA6
            gpioa.pa7.into_alternate(), // mosi  on PA7
        ),
        MODE,
        8.MHz(),
        &clocks,
    );

    let cs = gpioa.pa1.into_push_pull_output();

    let led = gpioc.pc13.into_push_pull_output();
    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    //let delay = Delay::new(cp.SYST, &clocks);
    let delay = cp.SYST.delay(&clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32f7xx")]
use stm32f7xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{ClockDivider, Enabled, Pins, Spi},
};

#[cfg(feature = "stm32f7xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>, Enabled<u8>>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let mut rcc = dp.RCC.constrain();
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();

    let sck = gpioa.pa5.into_alternate(); // sck   on PA5
    let miso = gpioa.pa6.into_alternate(); // miso  on PA6
    let mosi = gpioa.pa7.into_alternate(); // mosi  on PA7

    //   somewhere 8.mhz needs to be set in spi

    let spi =
        Spi::new(dp.SPI1, (sck, miso, mosi)).enable::<u8>(&mut rcc.apb2, ClockDivider::DIV32, MODE);

    let cs = gpioa.pa1.into_push_pull_output();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let clocks = rcc.cfgr.sysclk(216.MHz()).freeze();
    //let clocks = rcc.cfgr.sysclk(64.mhz()).pclk1(32.mhz()).freeze();

    let delay = Delay::new(cp.SYST, clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32h7xx")]
use stm32h7xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{Enabled, Spi},
};

#[cfg(feature = "stm32h7xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32h7xx")]
fn setup() -> (Spi<SPI1, Enabled>, PA1<Output<PushPull>>, impl LED, Delay) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    let rcc = dp.RCC.constrain();
    let ccdr = rcc.sys_ck(160.mhz()).freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5.into_alternate_af5(), // sck   on PA5
            gpioa.pa6.into_alternate_af5(), // miso  on PA6
            gpioa.pa7.into_alternate_af5(), // mosi  on PA7
        ),
        MODE,
        8.mhz(),
        ccdr.peripheral.SPI1,
        &clocks,
    );

    let cs = gpioa.pa1.into_push_pull_output();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let delay = Delay::new(cp.SYST, clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32l0xx")]
use stm32l0xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    spi::{Pins, Spi},
};

#[cfg(feature = "stm32l0xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi16());
    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioc = dp.GPIOC.split(&mut rcc);

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

    let cs = gpioa.pa1.into_push_pull_output();

    let led = gpioc.pc13.into_push_pull_output();

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_high().unwrap()
        }
    }

    let delay = cp.SYST.delay(rcc.clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32l1xx")] // eg  Discovery kit stm32l100 and Heltec lora_node STM32L151CCU6
use stm32l1xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA4, gpiob::PB6, Output, PushPull},
    prelude::*,
    rcc, // for ::Config but note name conflict with serial
    spi::{Pins, Spi},
    stm32::{CorePeripherals, Peripherals, SPI1},
};

#[cfg(feature = "stm32l1xx")]
use embedded_hal::digital::v2::OutputPin;

#[cfg(feature = "stm32l1xx")]
fn setup() -> (
    Spi<SPI1, impl Pins<SPI1>>,
    PA4<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
    let mut rcc = dp.RCC.freeze(rcc::Config::hsi());

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let spi = dp.SPI1.spi(
        (
            gpioa.pa5, // sck   on PA5   in board on Heltec
            gpioa.pa6, // miso  on PA6   in board on Heltec
            gpioa.pa7, // mosi  on PA7   in board on Heltec
        ),
        MODE,
        8.mhz(),
        &mut rcc,
    );

    let cs = gpioa.pa4.into_push_pull_output();

    let led = gpiob.pb6.into_push_pull_output();

    impl LED for PB6<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_high().unwrap()
        }
        fn off(&mut self) -> () {
            self.set_low().unwrap()
        }
    }

    let delay = cp.SYST.delay(rcc.clocks);

    (spi, cs, led, delay)
}

#[cfg(feature = "stm32l4xx")]
use stm32l4xx_hal::{
    delay::Delay,
    gpio::{gpioa::PA1, gpioc::PC13, Output, PushPull},
    pac::{CorePeripherals, Peripherals, SPI1},
    prelude::*,
    spi::{MisoPin, MosiPin, SckPin, Spi},
};

#[cfg(feature = "stm32l4xx")]
fn setup() -> (
    Spi<SPI1, (impl SckPin<SPI1>, impl MisoPin<SPI1>, impl MosiPin<SPI1>)>,
    PA1<Output<PushPull>>,
    impl LED,
    Delay,
) {
    let cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();
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
    let mut gpioc = dp.GPIOC.split(&mut rcc.ahb2);

    let spi = Spi::spi1(
        dp.SPI1,
        (
            gpioa
                .pa5
                .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl), // sck   on PA5
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

    let cs = gpioa
        .pa1
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);

    let led = gpioc
        .pc13
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

    impl LED for PC13<Output<PushPull>> {
        fn on(&mut self) -> () {
            self.set_low()
        }
        fn off(&mut self) -> () {
            self.set_high()
        }
    }

    let delay = Delay::new(cp.SYST, clocks);

    (spi, cs, led, delay)
}

// End of hal/MCU specific setup. Following should be generic code.

#[entry]
fn main() -> ! {
    rtt_init_print!();
    rprintln!("AD9833 example");

    let (spi, cs, mut led, mut delay) = setup();

    let mut synth = Ad983x::new_ad9833(spi, cs);
    synth.reset().unwrap();
    synth.enable().unwrap();

    let mut current_register = FrequencyRegister::F0;
    let mut table = MidiTable::default();

    loop {
        // Blink LED 0 to check that everything is actually running.
        // If the LED 0 does not blink, something went wrong.
        led.blink(50_u16, &mut delay);

        let midi_number = table.next().unwrap_or(0);
        let midi_number = f64::from(midi_number);
        let frequency_hz = libm::pow(2.0, (midi_number - 69.0) / 12.0) * 440.0;
        let mclk_hz = 25_000_000.0;
        let synth_value = frequency_hz * f64::from(1 << 28) / mclk_hz;

        // To ensure a smooth transition, set the frequency in the frequency
        // register that is not currently in use, then switch to it.
        let opposite = get_opposite(current_register);
        synth.set_frequency(opposite, synth_value as u32).unwrap();
        synth.select_frequency(opposite).unwrap();
        current_register = opposite;
    }
}

fn get_opposite(register: FrequencyRegister) -> FrequencyRegister {
    match register {
        FrequencyRegister::F0 => FrequencyRegister::F1,
        FrequencyRegister::F1 => FrequencyRegister::F0,
    }
}

#[derive(Debug, Default)]
struct MidiTable {
    position: usize,
    duration_counter: usize,
}

impl Iterator for MidiTable {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        let mut silence = None;
        let (_, note_duration, silence_duration) = Self::NOTES[self.position];
        let total_duration = note_duration + silence_duration;
        let is_in_silence =
            self.duration_counter >= note_duration && self.duration_counter < total_duration;
        if is_in_silence {
            self.duration_counter += 1;
            silence = Some(0);
        } else if self.duration_counter >= total_duration {
            self.position = (self.position + 1) % Self::NOTES.len();
            self.duration_counter = 1;
        } else {
            self.duration_counter += 1;
        }
        let tone = Some(Self::NOTES[self.position].0);
        silence.or(tone)
    }
}

impl MidiTable {
    const NOTES: [(u32, usize, usize); 62] = [
        (76, 4, 1),
        (76, 4, 1),
        (77, 4, 1),
        (79, 4, 1),
        //
        (79, 4, 1),
        (77, 4, 1),
        (76, 4, 1),
        (74, 4, 1),
        //
        (72, 4, 1),
        (72, 4, 1),
        (74, 4, 1),
        (76, 4, 1),
        //
        (76, 4, 4),
        (74, 2, 1),
        (74, 6, 4),
        //
        (76, 4, 1),
        (76, 4, 1),
        (77, 4, 1),
        (79, 4, 1),
        //
        (79, 4, 1),
        (77, 4, 1),
        (76, 4, 1),
        (74, 4, 1),
        //
        (72, 4, 1),
        (72, 4, 1),
        (74, 4, 1),
        (76, 4, 1),
        //
        (74, 4, 4),
        (72, 2, 1),
        (72, 6, 4),
        //
        (74, 4, 1),
        (74, 4, 1),
        (76, 4, 1),
        (72, 4, 1),
        //
        (74, 4, 1),
        (76, 2, 1),
        (77, 2, 1),
        (76, 4, 1),
        (72, 4, 1),
        //
        (74, 4, 1),
        (76, 2, 1),
        (77, 2, 1),
        (76, 4, 1),
        (74, 4, 1),
        //
        (72, 4, 1),
        (74, 4, 1),
        (67, 6, 2),
        //
        (76, 4, 1),
        (76, 4, 1),
        (77, 4, 1),
        (79, 4, 1),
        //
        (79, 4, 1),
        (77, 4, 1),
        (76, 4, 1),
        (74, 4, 1),
        //
        (72, 4, 1),
        (72, 4, 1),
        (74, 4, 1),
        (76, 4, 1),
        //
        (74, 6, 2),
        (72, 2, 1),
        (72, 6, 10),
    ];
}
