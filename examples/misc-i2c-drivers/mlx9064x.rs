// compare https://github.com/paxswill/mlx9064x-rs/blob/main/examples/true-frame-rate.rs
//   EXAMPLE NOT WORKING

//  as of Nov 2021 the mlx9064x crate uses some unstable features
//      arbitrary expressions in key-value attributes are unstable
//      unstable library feature 'int_bits_const'
// see   https://github.com/paxswill/mlx9064x-rs/issues/1

//use std::convert::TryInto;
//use std::env;
//use std::error::Error as StdError;
//use std::fmt;
//use std::path::Path;
//use std::time::{Duration, Instant};

//use linux_embedded_hal::I2cdev;

#![deny(unsafe_code)]
#![no_std]
#![no_main]

pub const DEVICE: &str  = "Mlx90640";
pub const CAMERA_BUS_ADR: u8 =  0x16 ;             //CHECK
pub const FRAME_RATE_NUM: f32 =  30.0 ;            //CHECK
pub const NUM_FRAMES: usize = FRAME_RATE_NUM * 10f32 ;

use mlx9064x::{CameraDriver, Mlx90640Driver, Mlx90641Driver};
use libm;

#[cfg(debug_assertions)]
use panic_semihosting as _;

#[cfg(not(debug_assertions))]
use panic_halt as _;

use cortex_m_rt::entry;

//use core::fmt::Write;
use embedded_hal::blocking::i2c::{Write, WriteRead};

//use rtt_target::{rprintln, rtt_init_print};
//use cortex_m_semihosting::hprintln;

use rust_integration_testing_of_examples::i2c_led_delay::{setup, LED};

#[entry]
fn main() -> ! {
    //rtt_init_print!();
    //rprintln!("AHT10 example");
    //hprintln!("AHT10 example").unwrap();

    let (i2c, mut led, mut delay) = setup();

    let manager = shared_bus::BusManager::<cortex_m::interrupt::Mutex<_>, _>::new(i2c);

//fn main() -> Result<(), AnyError> {
//    let args: Vec<String> = env::args().collect();
//    if args.len() < 5 || args.len() > 6 {
//        return Err(AnyError::String(
//            "Four arguments required: [640|641] <I2C bus> <camera address> <frame rate> [num_frames]"
//                .to_string(),
//        ));
//    }
//    let address: u8 = if args[3].starts_with("0x") {
//        let hex_digits = args[3].split_at(2).1;
//        u8::from_str_radix(&hex_digits, 16)?
//    } else {
//        args[3].parse()?
//    };
//    let bus_path = Path::new(&args[2]);
//    let bus = I2cdev::new(bus_path)?;
//    let frame_rate_num: f32 = args[4].parse()?;
//    let num_frames: usize = if args.len() < 6 {
//        (frame_rate_num * 10f32) as usize
//    } else {
//        args[5].parse()?
//    };
    //hprintln!("Starting measurements.");
    let instants = match DEVICE {
        "Mlx90640" => {
            let mut camera = Mlx90640Driver::new(bus, address)?;
            camera.set_frame_rate(FRAME_RATE_NUM);
            find_frequency(&mut camera, NUM_FRAMES)
        }
        "Mlx90641" => {
            let mut camera = Mlx90641Driver::new(bus, address)?;
            camera.set_frame_rate(frame_rate_num.try_into()?)?;
            find_frequency(&mut camera, num_frames)?
        }
    };
    // Find the duration between each instant, then calculate some statistics on those durations.
    let durations: Vec<Duration> = instants
        .windows(2)
        .map(|pair| -> Duration {
            if let [a, b] = pair {
                b.duration_since(*a)
            } else {
                panic!("Non-pair window slice encountered: {:?}", pair)
            }
        })
        .collect();
    let mut sorted_durations = durations.clone();
    sorted_durations.sort();
    //hprintln!("For {}Hz, actual timings (min, max, mean, median, mode):", FRAME_RATE_NUM);
    let min = sorted_durations.first().unwrap();
    //hprintln!("{}", as_frequency(min));
    let max = sorted_durations.last().unwrap();
    //hprintln!("{}", as_frequency(max));
    let mean_duration = durations.iter().sum::<Duration>() / durations.len() as u32;
    //hprintln!("{}", as_frequency(&mean_duration));
    let middle = sorted_durations.len() / 2;
    let median = if sorted_durations.len() % 2 == 0 {
        sorted_durations[middle..middle + 1]
            .iter()
            .sum::<Duration>()
            / 2
    } else {
        sorted_durations[middle]
    };
    //hprintln!("{}", as_frequency(&median));
    let mode = sorted_durations
        .windows(2)
        .map(|pair| {
            if let [prev, next] = pair {
                [*prev, *next]
            } else {
                unreachable!();
            }
        })
        .fold(
            (sorted_durations[0], 1, 1),
            |(mode, mode_count, current_count), [prev, current]| {
                if current != prev {
                    (mode, mode_count, 1)
                } else {
                    let current_count = current_count + 1;
                    if current_count > mode_count {
                        (current, current_count, current_count)
                    } else {
                        (mode, mode_count, current_count)
                    }
                }
            },
        )
        .0;
    //hprintln!("{}", as_frequency(&mode));
    Ok(())
}

fn find_frequency<
    'a,
    Cam,
    Clb,
    I2C,
    const HEIGHT: usize,
    const WIDTH: usize,
    const NUM_BYTES: usize,
>(
    driver: &mut CameraDriver<Cam, Clb, I2C, HEIGHT, WIDTH, NUM_BYTES>,
    num_frames: usize,
) -> Result<Vec<Instant>, AnyError>
where
    Cam: mlx9064x::common::MelexisCamera,
    Clb: mlx9064x::common::CalibrationData<'a>,
    I2C: Write + WriteRead + 'static,
    <I2C as WriteRead>::Error: 'static + StdError + fmt::Debug + Sync + Send,
    <I2C as Write>::Error: 'static + StdError + fmt::Debug + Sync + Send,
{
    let mut results = Vec::new();
    results.reserve(num_frames);
    let mut last_subpage = driver.last_measured_subpage()?;
    for _ in 0..num_frames {
        loop {
            let current_subpage = driver.last_measured_subpage()?;
            if current_subpage != last_subpage {
                results.push(Instant::now());
                last_subpage = current_subpage;
                break;
            }
        }
    }
    Ok(results)
}

fn as_frequency(duration: &Duration) -> f64 {
    duration.as_secs_f64().recip()
}

enum AnyError {
    panic(),  // could do better
}

// //It's anyhow::Error, but less functional and less tested.
//#[derive(Debug)]
//enum AnyError {
//    Wrapped(Box<dyn StdError + 'static>),
//    String(String),
//}
//
//impl std::fmt::Display for AnyError {
//    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
//        match self {
//            AnyError::Wrapped(err) => write!(f, "{}", err),
//            AnyError::String(s) => write!(f, "{}", s),
//        }
//    }
//}
//
//impl<E> From<E> for AnyError
//where
//    E: StdError + 'static,
//{
//    fn from(err: E) -> Self {
//        AnyError::Wrapped(Box::new(err))
//    }
//}
