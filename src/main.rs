//
// The Rust-Powered Ghost Processing Unit
//
// Jules Stuart
//

use std::{
    io::Write,
    thread,
    time::{Duration},
    net::TcpStream,
};

use rppal::{
    system::DeviceInfo,
};

use rust_gpu::ahrs::Ahrs;

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



    // Set up connection to frontend
    let mut output_stream = TcpStream::connect("192.168.77.189:65432")
        .expect("IO Error: Could not connect to visualizer.");
    let mut output_bytes: [u8; 32] = [0; 32];

    // Replace this with a send() using an output data struct
    //if output_connected && iterations%10 == 0 {
    //    
    //}

    // Set up connection to AHRS
    let gpu_ahrs = Ahrs::new(&time_string)
        .expect("GPU Error: Error creating AHRS interface.");
    println!("Waiting on the AHRS to come up...");
    // thread::sleep(Duration::from_millis(5000));
    
    let mut quaternion: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    loop {
        // TODO - Refactor this to a pub fn get_lastest_quaternion() that can be called on the AHRS
        quaternion = match gpu_ahrs.receiver.as_ref().unwrap().try_recv() {
            Ok(ahrs_quaternion) => ahrs_quaternion,
            Err(_error) => quaternion,
        };

        // Write new quaternion to frontend
        for i in 0..4 {
            let bytes_to_write = quaternion[i].to_be_bytes();
            for j in 0..8 {
                output_bytes[(8*i)+j] = bytes_to_write[j];
            }
        }

        output_stream.write(&output_bytes)
            .expect("IO Error: Could not write to socket.");

        thread::sleep(Duration::from_millis(20));
    }
}
