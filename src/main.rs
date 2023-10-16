//
// The Rust-Powered Ghost Processing Unit
//
// Jules Stuart
//

use std::{thread, time::Duration};
use rppal::system::DeviceInfo;
use rust_gpu::{
    ahrs::Ahrs,
    sockethost::SocketHost,
};

fn main() {
    println!("Welcome to the Rust G.P.U. V0.0!");
    println!("Running on a {}.", DeviceInfo::new().expect("Could not get device info.").model());

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

    // Set up connection to AHRS
    let gpu_ahrs = Ahrs::new(&time_string)
        .expect("GPU Error: Error creating AHRS interface.");
    println!("Waiting on the AHRS to come up...");
    thread::sleep(Duration::from_millis(1000));

    // Set up connection to SocketHost
    let socket_host = SocketHost::new(&time_string)
        .expect("GPU Error: Error creating SocketHost interface.");

    let mut iterations: u32 = 0;

    loop {
        // Check for new Messages from the SocketHost
        // TODO - Refactor this to a pub fn get_message() which gets an Option<Message>
        let message = match socket_host.receiver.as_ref().unwrap().try_recv() {
                Ok(sockethost_message) => Some(sockethost_message),
                Err(_error) => None,
        };

        // TODO - replace with an "if let"
        if message.is_some() {
            let mut unwrapped_message = message.unwrap();

            let message_response = match unwrapped_message.address {
                0x00 => "RUST_GPU_V0.0".as_bytes().to_vec(),
                0x01 => gpu_ahrs.get_latest_quaternion(),
                _ => "UNKNOWN ADDRESS".as_bytes().to_vec(),
            };

            println!("GPU - Got a message with address: {}", unwrapped_message.address);
            println!("GPU - Going to respond with: {:?}", message_response);

            unwrapped_message.response = message_response;

            socket_host.sender.as_ref().unwrap().send(unwrapped_message)
                .expect("Could not send data from GPU.");
        }

        iterations = iterations + 1;
        // println!("{} iterations through the main loop", iterations);

        thread::sleep(Duration::from_millis(10));
    }
}
