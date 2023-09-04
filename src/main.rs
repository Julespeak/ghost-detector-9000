//
// The Rust-Powered Ghost Processing Unit
//
// Jules Stuart
//

use std::{
    error::Error,
    thread,
    time::{Duration, Instant},
    fs::File,
    io::Write,
    net::TcpStream,
};

use rppal::{
    gpio::Gpio,
    system::DeviceInfo,
    i2c::I2c
};

use rust_gpu::ahrs::{ Ascale, Gscale, Mscale };

fn main() -> Result<(), Box<dyn Error>> {
    println!("Welcome to the Rust G.P.U. V0.0!");
    println!("Running on a {}.", DeviceInfo::new()?.model());

    // Get current system time in local units
    let current_time = std::time::SystemTime::now();
    let utc = time::OffsetDateTime::UNIX_EPOCH
        + Duration::try_from(current_time.duration_since(std::time::UNIX_EPOCH).unwrap()).unwrap();
    let local_time = utc.to_offset(time::UtcOffset::local_offset_at(utc).unwrap());
    let time_string = local_time
        .format(
            time::macros::format_description!(
                "[day]-[month repr:short]-[year]_[hour]:[minute]:[second]"
            ),
        )
        .unwrap();
    println!("Local time: {}", time_string);

    // Set up connection to log file
    let log_file_name = format!("/home/ghost/rust-stuff/mpu_logs/{}_logfile.txt", time_string); 
    let mut log_file = File::create(log_file_name).expect("Unable to create file.");

    // Set up connection to frontend
    //let output_connected: bool = true;
    //let mut output_stream = TcpStream::connect("192.168.77.189:65432")?;
    //let mut output_bytes: [u8; 32] = [0; 32];

    // Set up connection to AHRS
    let gpu_ahrs = Ahrs::new();
    loop {
        let result = gpu_ahrs::Receiver.try_recv().unwrap();
        println!("{:?}", result)
        thread::sleep(Duration::from_millis(500));
    }

    Ok(())
}
