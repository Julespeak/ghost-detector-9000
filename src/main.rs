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
                0x01 => get_encoded_quaternion(&gpu_ahrs),
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

fn get_encoded_quaternion(ahrs: &Ahrs) -> Vec<u8> {
    // the idea here is that there will be a function of the ahrs get_latest_quaternion;
    //  in the future, this thread will not need to have a default value for the quaternion
    let quaternion = match ahrs.receiver.as_ref().unwrap().try_recv() {
        Ok(ahrs_quaternion) => ahrs_quaternion,
        Err(_error) => [1.0, 0.0, 0.0, 0.0],
    };
    let mut encoded_quaternion = Vec::new();
    for _i in 0..4 {
        // https://stackoverflow.com/questions/54142528/how-can-i-concatenate-two-slices-or-two-vectors-and-still-have-access-to-the-ori
        encoded_quaternion = encoded_quaternion.to_vec().into_iter().chain(quaternion[_i].to_be_bytes()).collect();
    }
    encoded_quaternion
}
