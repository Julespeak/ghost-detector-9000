use std::{
    error::Error,
    fs::File,
    io::Write,
    sync::{
        mpsc,
        mpsc::Sender,
    },
    thread,
    time::{
        Duration,
        Instant,
    }
};

use rppal::{
    i2c::I2c,
    gpio::Gpio,
};

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Mscale {
    Mfs14bits = 0, // 0.6 mG per LSB
    Mfs16bits, // 0.15 mG per LSB
}

pub struct Mpu9250Calibration {
    a_res: f64,
    g_res: f64,
    m_res: f64,
    mag_calibration: [f64; 3],
    mag_bias: [f64; 3],
    mag_scale: [f64; 3],
}

pub struct SocketHost {
    pub receiver: Option<mpsc::Receiver<[f64; 4]>>,
    pub sender: Option<mpsc::Sender<[f64; 4]>>,
    thread: Option<thread::JoinHandle<()>>,
}

impl Ahrs {
    /// Create a new AHRS interface.
    ///
    /// Hello, world, lol.
    ///
    /// # More Documentation
    ///
    /// Would go here eventually...
    pub fn new(time_string: &str) -> Result<Ahrs, Box<dyn Error>> {
        // Set sensor resolution
        let a_scale: Ascale = Ascale::Afs2g;
        let g_scale: Gscale = Gscale::Gfs250dps;
        let m_scale: Mscale = Mscale::Mfs16bits; // Choose either 14-bit or 16-bit magnetometer resolution

        // Specify magnetometer full scale
        let m_mode: u8 = 0x02; // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read

        let mag_calibration: [f64; 3]; // factory magnetometer calibration
        let mag_bias: [f64; 3]; // measured magnetometer bias correction
        let mag_scale: [f64; 3]; // measured magnetometer scale correction

        // Connect using I2C bus #0
        let mut i2c = I2c::with_bus(0)?;

        // Set up IO connections
        let mut led = Gpio::new()
            .expect("GPIO Error: Unable to connect to GPIO manager.")
            .get(crate::GPIO_LED)
            .expect("GPIO Error: Unable to get GPIO_LED pin.")
            .into_output();
        
        // Set up connection to log file
        let log_file_name = format!("/home/ghost/rust-stuff/mpu_logs/{}_mpu_logfile.txt", time_string); 
        let mut log_file = File::create(log_file_name)
            .expect("Unable to create MPU logfile.");

        // Set the address of the MPU-9250
        i2c.set_slave_address(crate::ADDR_MPU9265)?;

        // Read the WHO_AM_I register and check the result
        let mut reg = [0u8; 1];
        i2c.block_read(WHO_AM_I_MPU9250 as u8, &mut reg)?;

        if reg[0] == 0x71 {
            println!("Successfully connected to MPU-9250!");

            // Calibrate gyro and accelerometers, load biases in bias registers
            calibrate_mpu9250(&i2c, &mut log_file)?;

            // Initialize the MPU9050
            // TODO - the ownership of a_scale and g_scale could go to this function instead
            init_mpu9250(&i2c, &a_scale, &g_scale)?;

            // read the WHO_AM_I register of the magnetometer, this is a good test of communication
            i2c.set_slave_address(crate::ADDR_AK8963)?;
            i2c.block_read(AK8963_WHO_AM_I as u8, &mut reg).expect("Error writing to magnetometer");
            if reg[0] == 0x48 {
                println!("Successfully connected to AK8963!");

                // TODO - the ownership of m_scale could go to this function instead
                mag_calibration = init_ak8963(&i2c, &m_scale, m_mode)?;
                write!(log_file, "DEBUG: Magnetometer factory calibration: {:?}\n", mag_calibration)
                    .expect("Error writing to log file.");

                //(mag_bias, mag_scale) = calibrate_ak8963(&i2c, &mag_calibration, &mut log_file)?;
                //write!(log_file, "DEBUG: Magnetometer bias correction: {:?}\n", mag_bias)
                //    .expect("Error writing to log file.");
                //write!(log_file, "DEBUG: Magnetometer scale correction: {:?}\n", mag_scale)
                //    .expect("Error writing to log file.");
                // use previously calculated calibration values
                mag_bias = [-200.515625, 93.515625, 78.1171875];
                mag_scale = [0.9703703703703703, 1.0155038759689923, 1.0155038759689923];
            } else {
                panic!("Could not connect to AK8963.");
            }

            // Blink the LED by setting the pin's logic level high for 500 ms.
            led.set_high();
            thread::sleep(Duration::from_millis(500));
            led.set_low();
        } else {
            panic!("Could not connect to MPU-9250.");
        }

        // Create configuration structure from initialization data
        let calibration = Mpu9250Calibration {
            a_res: get_a_res(&a_scale),
            g_res: get_g_res(&g_scale),
            m_res: get_m_res(&m_scale),
            mag_calibration: mag_calibration,
            mag_bias: mag_bias,
            mag_scale: mag_scale,
        };

        // spawn a thread to start cranking out quaternions forever
        let (soci_sender, soci_receiver) = mpsc::channel();
        let quaternion_thread = thread::spawn(move || compute_quaternions(&mut i2c, &mut log_file, &calibration, soci_sender));

        Ok(Ahrs {
            receiver: Some(soci_receiver),
            thread: Some(quaternion_thread),
        })
    }
}

/// Compute quaternions forever
pub fn compute_quaternions(i2c: &mut I2c, log_file: &mut File, calibration: &Mpu9250Calibration, sender: Sender<[f64; 4]>) {
    let start = Instant::now();

    let interrupt_pin = Gpio::new()
        .expect("GPIO Error: Unable to connect to GPIO manager.")
        .get(crate::MPU_INT)
        .expect("GPIO Error: Unable to get MPU_INT pin.")
        .into_input();

    // Arrays for storing raw sensor data
    let mut raw_mag_data: [i16;3] = [0; 3];
    let mut raw_mpu_data: [i16;7] = [0; 7];

    // Variables to hold latest sensor data values
    let mut ax: f64 = 0.0;
    let mut ay: f64 = 0.0;
    let mut az: f64 = 0.0;
    let mut gx: f64 = 0.0;
    let mut gy: f64 = 0.0;
    let mut gz: f64 = 0.0;
    let mut mx: f64 = 0.0;
    let mut my: f64 = 0.0;
    let mut mz: f64 = 0.0;

    let mut q: [f64; 4] = [1.0, 0.0, 0.0, 0.0];

    let mut last_update_time: u64 = 0;

    // Variables for monitoring the acquisition rate
    let mut iterations: u32 = 0;
    let mut mpu_reads: u32 = 0;

    const TOTAL_READS: u32 = 3000;
    let mut dummy_data: [u8; 1] = [0];
    let mut mpu_sample_start: [u64; TOTAL_READS as usize] = [0; TOTAL_READS as usize];
    let mut mpu_sample_stop: [u64; TOTAL_READS as usize] = [0; TOTAL_READS as usize];
    let mut mag_sample_start: [u64; TOTAL_READS as usize] = [0; TOTAL_READS as usize];
    let mut mag_sample_stop: [u64; TOTAL_READS as usize] = [0; TOTAL_READS as usize];

    // TODO - there should be a way out of this loop when a close method is invoked
    loop {
        i2c.set_slave_address(crate::ADDR_MPU9265)
            .expect("I2C Error: Unable to change address.");
        if interrupt_pin.is_high() { // If new data is available, read it in
            if mpu_reads < TOTAL_READS-1 {
                mpu_sample_start[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            // Read data from MPU
            read_mpu_data(&i2c, &mut raw_mpu_data)
                .expect("I2C Error: Error reading from MPU.");

            if mpu_reads < TOTAL_READS-1 {
                mpu_sample_stop[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }
            mpu_reads = mpu_reads + 1;

            ax = (raw_mpu_data[0] as f64) * calibration.a_res;
            ay = (raw_mpu_data[1] as f64) * calibration.a_res;
            az = (raw_mpu_data[2] as f64) * calibration.a_res;
            write!(log_file, "DATA: accel_data, 3, 1\n")
                .expect("IOError: Error writing to log file.");
            write!(log_file, "{} {} {}\n", ax, ay, az)
                .expect("IOError: Error writing to log file.");

            gx = (raw_mpu_data[4] as f64) * calibration.g_res;
            gy = (raw_mpu_data[5] as f64) * calibration.g_res;
            gz = (raw_mpu_data[6] as f64) * calibration.g_res;
            write!(log_file, "DATA: gyro_data, 3, 1\n")
                .expect("IOError: Error writing to log file.");
            write!(log_file, "{} {} {}\n", gx, gy, gz)
                .expect("IOError: Error writing to log file.");
        }

        i2c.set_slave_address(crate::ADDR_AK8963)
            .expect("I2C Error: Unable to change address.");
        i2c.block_read(AK8963_ST1 as u8, &mut dummy_data)
            .expect("I2C Error: Error reading from magnetometer.");

        // TODO - This check for new data is redundant with the one inside the read_mag_data function
        if dummy_data[0] & 0x01 == 0x01 { // If new data is available, read it in
            if mpu_reads < TOTAL_READS-1 {
               mag_sample_start[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            // Read data from magnetometer
            read_mag_data(&i2c, &mut raw_mag_data)
                .expect("I2C Error: Error reading from magnetometer.");

            if mpu_reads < TOTAL_READS-1 {
                mag_sample_stop[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            mx = ((raw_mag_data[0] as f64) * calibration.mag_calibration[0] - calibration.mag_bias[0])
                * calibration.mag_scale[0] * calibration.m_res;
            my = ((raw_mag_data[1] as f64) * calibration.mag_calibration[1] - calibration.mag_bias[1])
                * calibration.mag_scale[1] * calibration.m_res;
            mz = ((raw_mag_data[2] as f64) * calibration.mag_calibration[2] - calibration.mag_bias[2])
                * calibration.mag_scale[2] * calibration.m_res;

            write!(log_file, "DATA: mag_data, 3, 1\n")
                .expect("IOError: Error writing to log file.");
            write!(log_file, "{} {} {}\n", mx, my, mz)
                .expect("IOError: Error writing to log file.");
        }

        // Sensors x (y)-axis of the accelerometer/gyro is aligned with the y (x)-axis of the magnetometer;
        // the magnetometer z-axis (+ down) is misaligned with the z-axis (+ up) of accelerometer and gyro!
        let current_update_time: u64 = start.elapsed().as_nanos() as u64;
        let delta_t: f64 = (current_update_time - last_update_time) as f64 / 1000000000.0;
        last_update_time = current_update_time;
        write!(log_file, "DATA: update_period, 1, 1\n")
            .expect("IOError: Error writing to log file.");
        write!(log_file, "{}\n", delta_t)
            .expect("IOError: Error writing to log file.");

        madgwick_quaternion_update(&[ax, ay, az], &[gx*std::f64::consts::PI/180.0, gy*std::f64::consts::PI/180.0, gz*std::f64::consts::PI/180.0], &[my, mx, -mz], &mut q, delta_t)
            .expect("IOError: Error in quaternion update.");

        // TODO - Check to see if the host has 
        if iterations%10 == 0 {
           sender.send(q.clone()).expect("Thread Error: Could not send data to client.");
        }

        iterations = iterations + 1;

    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed)
        .expect("Error writing to log file.");
    write!(log_file, "DEBUG: Loop iterations: {}\n", iterations)
        .expect("Error writing to log file.");
    write!(log_file, "DEBUG: MPU reads: {}\n", mpu_reads)
        .expect("Error writing to log file.");

    // Print timing information to file
    write!(log_file, "DATA: mpu_sample_start, 1, {}\n", TOTAL_READS)
        .expect("Error writing to log file.");
    for (_i, mpu_sample_start_time) in mpu_sample_start.iter().enumerate() {
        write!(log_file, "{}\n", mpu_sample_start_time)
            .expect("Error writing to log file.");
    }
    write!(log_file, "DATA: mpu_sample_stop, 1, {}\n", TOTAL_READS)
        .expect("Error writing to log file.");
    for (_i, mpu_sample_stop_time) in mpu_sample_stop.iter().enumerate() {
        write!(log_file, "{}\n", mpu_sample_stop_time)
            .expect("Error writing to log file.");
    }
    write!(log_file, "DATA: mag_sample_start, 1, {}\n", TOTAL_READS)
        .expect("Error writing to log file.");
    for (_i, mag_sample_start_time) in mag_sample_start.iter().enumerate() {
        write!(log_file, "{}\n", mag_sample_start_time)
            .expect("Error writing to log file.");
    }
    write!(log_file, "DATA: mag_sample_stop, 1, {}\n", TOTAL_READS)
        .expect("Error writing to log file.");
    for (_i, mag_sample_stop_time) in mag_sample_stop.iter().enumerate() {
        write!(log_file, "{}\n", mag_sample_stop_time)
            .expect("Error writing to log file.");
    }
}
