//
// The Rust-Powered Ghost Processing Unit
//
// Jules Stuart
//

use std::{env, thread, time::Duration, sync::{Arc, Mutex}};
use rppal::{system::DeviceInfo, i2c::I2c};
use rust_gpu::{
    ahrs::AhrsHost,
    socket::SocketHost,
    emf::EmfHost,
};

fn main() {
    println!("Welcome to the Rust G.P.U. V0.0!");
    println!("Running on a {}.", DeviceInfo::new().expect("Could not get device info.").model());

    let args: Vec<String> = env::args().collect();
    let mut verbose = false;
    if args.contains(&String::from("-v")) {
        verbose = true;
    }

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

    // Set up I2C hardware interface
    let i2c = I2c::with_bus(0)
        .expect("GPU Error: Could not create I2C device.");
    let i2c = Arc::new(Mutex::new(i2c));

    // Set up connection to AHRS
    let gpu_ahrs = AhrsHost::new(&time_string, verbose, Arc::clone(&i2c));
    println!("Waiting on the AHRS to come up...");
    thread::sleep(Duration::from_millis(1000));

    // Set up connection to EMF detector
    let gpu_emf = EmfHost::new(&time_string, verbose, Arc::clone(&i2c));
    println!("Waiting on the EMF host to come up...");
    thread::sleep(Duration::from_millis(1000));

    // Set up connection to SocketHost
    let socket_host = SocketHost::new(&time_string);

    println!("Rust G.P.U. is alive; beginning main loop...");

    let mut iterations: u32 = 0;

    loop {
        // Check for new Messages from the SocketHost
        if let Ok(mut message) = socket_host.receiver.as_ref().unwrap().try_recv() {
            let message_response = match message.address {
                0x00 => "RUST_GPU_V0.0".as_bytes().to_vec(),
                0x01 => gpu_ahrs.send_message(0x00, Vec::new()), // Get the latest quaternion
                0x02 => gpu_emf.send_message(0x00, Vec::new()), // Acquire single-shot of ADC data
                0x03 => gpu_ahrs.send_message(0x01, Vec::new()), // Do field recalibration
                0x04 => gpu_emf.send_message(0x01, Vec::new()), // Begin long acquisition of ADC data
                0x05 => gpu_emf.send_message(0x02, Vec::new()), // End long acquisition of ADC data
                _ => "UNKNOWN ADDRESS".as_bytes().to_vec(),
            };

            // println!("GPU - Got a message with address: {}", unwrapped_message.address);
            // println!("GPU - Going to respond with: {:?}", message_response);

            message.response = message_response;

            socket_host.sender.as_ref().unwrap().send(message)
                .expect("Could not send data from GPU.");
        }

        iterations = iterations + 1;
        // println!("{} iterations through the main loop", iterations);

        thread::sleep(Duration::from_millis(10));
    }
}
