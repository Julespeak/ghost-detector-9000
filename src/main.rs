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
    println!("AhrsHost coming online...");
    thread::sleep(Duration::from_millis(1000));

    // Set up connection to EMF detector
    let gpu_emf = EmfHost::new(&time_string, verbose, Arc::clone(&i2c));
    println!("EmfHost coming online...");
    thread::sleep(Duration::from_millis(1000));

    // Set up connection to SocketHost
    let gpu_socket = SocketHost::new(&time_string);
    println!("SocketHost coming online...");
    thread::sleep(Duration::from_millis(100));

    println!("Rust G.P.U. is alive; beginning main loop...");

    let mut iterations: u32 = 0;

    loop {
        // Check for new Messages from the SocketHost
        if let Ok(mut message) = gpu_socket.receiver.as_ref().unwrap().try_recv() {
            let message_response = match message.address {
                0x00 => "RUST_GPU_V0.0".as_bytes().to_vec(),
                0x01 => gpu_ahrs.send_message(0x00, Vec::new()), // Get the latest quaternion
                0x02 => gpu_emf.send_message(0x00, Vec::new()), // Acquire single-shot of ADC data
                0x03 => gpu_ahrs.send_message(0x01, Vec::new()), // Do field recalibration
                0x04 => gpu_emf.send_message(0x01, Vec::new()), // Begin long acquisition of ADC data
                0x05 => gpu_emf.send_message(0x02, Vec::new()), // End long acquisition of ADC data
                0x06 => { // G.P.U. shutdown
                    println!("Rust G.P.U. is going down!");
                    break
                },
                _ => "UNKNOWN ADDRESS".as_bytes().to_vec(),
            };

            message.response = message_response;

            gpu_socket.sender.as_ref().unwrap().send(message)
                .expect("GPU: Could not send data to SocketHost.");
        }

        iterations = iterations + 1;

        thread::sleep(Duration::from_millis(10));
    }

    println!("Did {} iterations through the main loop", iterations);

    drop(gpu_ahrs);
    drop(gpu_emf);
    drop(gpu_socket);

    println!("Rust G.P.U. is dead.  R.I.P.");
}
