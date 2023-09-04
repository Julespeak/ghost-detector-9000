//
// The Rust-Powered Ghost Processing Unit
//
// Jules Stuart
//

mod ahrs;

use std::{
    error::Error,
    thread,
    time::{Duration, Instant, SystemTime},
    fs::File,
    io::{Write, prelude::*},
    net::TcpStream,
};

use rppal::{
    gpio::Gpio,
    system::DeviceInfo,
    i2c::I2c
};

pub use crate::ahrs::ahrs;

// Addresses of the chips on the board that I have; determined with i2cdetect
const ADDR_MPU9265: u16 = 0x68;
const ADDR_AK8963:  u16 = 0x0C;

fn main() -> Result<(), Box<dyn Error>> {
    println!("Welcome to the Rust G.P.U. V0.0!");
    println!("Running on a {}.", DeviceInfo::new()?.model());


    // get current system time in local units
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

    let log_file_name = format!("/home/ghost/rust-stuff/mpu_logs/{}_logfile.txt", time_string); 
    let mut log_file = File::create(log_file_name).expect("Unable to create file.");

    // set up connection to frontend
    let mut output_connected: bool = true;
    let mut output_stream = TcpStream::connect("192.168.77.189:65432")?;
    let mut output_bytes: [u8; 32] = [0; 32];

    let a_scale: Ascale = Ascale::Afs2g;
    let g_scale: Gscale = Gscale::Gfs250dps;
    let m_scale: Mscale = Mscale::Mfs16bits; // Choose either 14-bit or 16-bit magnetometer resolution
    let mut a_res: f64 = 0.0; // Scale resolutions per LSB for the sensors
    let mut g_res: f64 = 0.0;
    let mut m_res: f64 = 0.0;

    // Specify magnetometer full scale
    let m_mode: u8 = 0x02; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

    // Arrays for storing raw sensor data
    let mut raw_mag_data: [i16;3] = [0; 3];
    let mut raw_mpu_data: [i16;7] = [0; 7];

    let mut mag_calibration: [f64; 3] = [0.0; 3]; // factory magnetometer calibration
    let mut mag_bias: [f64; 3] = [0.0; 3]; // measured magnetometer bias correction
    let mut mag_scale: [f64; 3] = [0.0; 3]; // measured magnetometer scale correction

    let mut ax: f64 = 0.0; // variables to hold latest sensor data values
    let mut ay: f64 = 0.0; // ""
    let mut az: f64 = 0.0; // ""
    let mut gx: f64 = 0.0; // ""
    let mut gy: f64 = 0.0; // ""
    let mut gz: f64 = 0.0; // ""
    let mut mx: f64 = 0.0; // ""
    let mut my: f64 = 0.0; // ""
    let mut mz: f64 = 0.0; // ""

    let mut q: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    let mut temperature: f64 = 0.0;

    let mut self_test: [f64; 6] = [0.0; 6];

    // Connect using I2C bus #0
    let mut i2c = I2c::with_bus(0)?;

    // Set up IO connections
    let mut led = Gpio::new()?.get(GPIO_LED)?.into_output();
    let mut interrupt_pin = Gpio::new()?.get(MPU_INT)?.into_input();

    // Attach hardware interrupt
    // actually going to try doing this without interrupts for now
    //interrupt_pin.set_async_interrupt(rppal::gpio::Trigger::RisingEdge, READ_MPU)?;

    // Variables for monitoring the acquisition rate
    let mut iterations: u32 = 0;
    let mut mpu_reads: u32 = 0;

    // Set the address of the MPU-9250
    i2c.set_slave_address(ADDR_MPU9265)?;

    // Read the WHO_AM_I register and check the result
    let mut reg = [0u8; 1];
    i2c.block_read(WHO_AM_I_MPU9250 as u8, &mut reg)?;

    if reg[0] == 0x71 {
        println!("Successfully connected to MPU-9250!");

        // Perform a self test and report the values; do not that this value is basically nonsense
        //self_test = mpu9250_self_test(&i2c)?;
        //write!(log_file, "DEBUG: Accelerometer/gyro self test result: {:?}\n", self_test);

        // get sensor resolutions
        a_res = ahrs::get_a_res(&a_scale);
        g_res = ahrs::get_g_res(&g_scale);
        m_res = ahrs::get_m_res(&m_scale);

        // println!("Enum values: {}, {}, {}", a_scale as u8, g_scale as u8, m_scale as u8);

        // Calibrate gyro and accelerometers, load biases in bias registers
        ahrs::calibrate_mpu9250(&i2c, &mut log_file)?;

        // Initialize the MPU9050
        // TODO - the ownership of a_scale and g_scale could go to this function instead
        ahrs::init_mpu9250(&i2c, &a_scale, &g_scale)?;
        println!("initialization done.");

        // read the WHO_AM_I register of the magnetometer, this is a good test of communication
        i2c.set_slave_address(ADDR_AK8963)?;
        i2c.block_read(AK8963_WHO_AM_I as u8, &mut reg).expect("Error writing to magnetometer");
        if reg[0] == 0x48 {
            println!("Successfully connected to AK8963!");

            // TODO - the ownership of m_scale could go to this function instead
            mag_calibration = ahrs::init_ak8963(&i2c, &m_scale, m_mode)?;
            write!(log_file, "DEBUG: Magnetometer factory calibration: {:?}\n", mag_calibration);

            //(mag_bias, mag_scale) = calibrate_ak8963(&i2c, &mag_calibration, &mut log_file)?;
            //write!(log_file, "DEBUG: Magnetometer bias correction: {:?}\n", mag_bias);
            //write!(log_file, "DEBUG: Magnetometer scale correction: {:?}\n", mag_scale);
            // use previously calculated calibration values
            mag_bias = [187.06640625, 376.5234375, 216.59765625];
            mag_scale = [0.9712230215827338, 0.996309963099631, 1.0344827586206897];
        } else {
            println!("Could not connect to AK8963.");
        }

        // Blink the LED by setting the pin's logic level high for 500 ms.
        led.set_high();
        thread::sleep(Duration::from_millis(500));
        led.set_low();
    } else {
        println!("Could not connect to MPU-9250.");
        // should break out of the program or something here
    }

    // Set the address of the MPU-9250
    println!("Begin data acquisition!");
    let mut start = Instant::now();

    let mut last_update_time: u64 = 0;
    let mut current_update_time: u64 = start.elapsed().as_nanos() as u64;
    let mut delta_t: f64 = (current_update_time - last_update_time) as f64 / 1000000000.0;

    const total_reads: u32 = 3000;
    let mut dummy_data: [u8; 1] = [0];
    let mut mpu_sample_start: [u64; total_reads as usize] = [0; total_reads as usize];
    let mut mpu_sample_stop: [u64; total_reads as usize] = [0; total_reads as usize];
    let mut mag_sample_start: [u64; total_reads as usize] = [0; total_reads as usize];
    let mut mag_sample_stop: [u64; total_reads as usize] = [0; total_reads as usize];

    loop {
        i2c.set_slave_address(ADDR_MPU9265)?;
        if interrupt_pin.is_high() { // If new data is available, read it in
            if mpu_reads < total_reads-1 {
                mpu_sample_start[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            // Read data from MPU
            ahrs::read_mpu_data(&i2c, &mut raw_mpu_data)?;

            if mpu_reads < total_reads-1 {
                mpu_sample_stop[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }
            mpu_reads = mpu_reads + 1;

            ax = (raw_mpu_data[0] as f64) * a_res;
            ay = (raw_mpu_data[1] as f64) * a_res;
            az = (raw_mpu_data[2] as f64) * a_res;
            write!(log_file, "DATA: accel_data, 3, 1\n");
            write!(log_file, "{} {} {}\n", ax, ay, az);

            gx = (raw_mpu_data[4] as f64) * g_res;
            gy = (raw_mpu_data[5] as f64) * g_res;
            gz = (raw_mpu_data[6] as f64) * g_res;
            write!(log_file, "DATA: gyro_data, 3, 1\n");
            write!(log_file, "{} {} {}\n", gx, gy, gz);
        }

        i2c.set_slave_address(ADDR_AK8963)?;
        i2c.block_read(AK8963_ST1 as u8, &mut dummy_data)?;

        // TODO - This check for new data is redundant with the one inside the read_mag_data function
        if dummy_data[0] & 0x01 == 0x01 { // If new data is available, read it in
            if mpu_reads < total_reads-1 {
               mag_sample_start[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            // Read data from magnetometer
            ahrs::read_mag_data(&i2c, &mut raw_mag_data)?;

            if mpu_reads < total_reads-1 {
                mag_sample_stop[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            mx = ((raw_mag_data[0] as f64) * mag_calibration[0] - mag_bias[0]) * mag_scale[0] * m_res;
            my = ((raw_mag_data[1] as f64) * mag_calibration[1] - mag_bias[1]) * mag_scale[1] * m_res;
            mz = ((raw_mag_data[2] as f64) * mag_calibration[2] - mag_bias[2]) * mag_scale[2] * m_res;
            write!(log_file, "DATA: mag_data, 3, 1\n");
            write!(log_file, "{} {} {}\n", mx, my, mz);
        }

        // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
        // the magnetometer z-axis (+ down) is misaligned with the z-axis (+ up) of accelerometer and gyro!
        current_update_time = start.elapsed().as_nanos() as u64;
        delta_t = (current_update_time - last_update_time) as f64 / 1000000000.0;
        last_update_time = current_update_time;
        write!(log_file, "DATA: update_period, 1, 1\n");
        write!(log_file, "{}\n", delta_t);

        ahrs::madgwick_quaternion_update(&[ax, ay, az], &[gx*std::f64::consts::PI/180.0, gy*std::f64::consts::PI/180.0, gz*std::f64::consts::PI/180.0], &[my, mx, -mz], &mut q, delta_t)?;

        // Write new quaternion to frontend
        for i in 0..4 {
            let bytes_to_write = q[i].to_be_bytes();
            for j in 0..8 {
                output_bytes[(8*i)+j] = bytes_to_write[j];
            }
        }

        if output_connected && iterations%10 == 0 {
            output_stream.write(&output_bytes)?;
        }

        iterations = iterations + 1;

    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed);
    write!(log_file, "DEBUG: Loop iterations: {}\n", iterations);
    write!(log_file, "DEBUG: MPU reads: {}\n", mpu_reads);

    // Print timing information to file
    write!(log_file, "DATA: mpu_sample_start, 1, {}\n", total_reads);
    for (i, mpu_sample_start_time) in mpu_sample_start.iter().enumerate() {
        write!(log_file, "{}\n", mpu_sample_start_time);
    }
    write!(log_file, "DATA: mpu_sample_stop, 1, {}\n", total_reads);
    for (i, mpu_sample_stop_time) in mpu_sample_stop.iter().enumerate() {
        write!(log_file, "{}\n", mpu_sample_stop_time);
    }
    write!(log_file, "DATA: mag_sample_start, 1, {}\n", total_reads);
    for (i, mag_sample_start_time) in mag_sample_start.iter().enumerate() {
        write!(log_file, "{}\n", mag_sample_start_time);
    }
    write!(log_file, "DATA: mag_sample_stop, 1, {}\n", total_reads);
    for (i, mag_sample_stop_time) in mag_sample_stop.iter().enumerate() {
        write!(log_file, "{}\n", mag_sample_stop_time);
    }

    Ok(())
}