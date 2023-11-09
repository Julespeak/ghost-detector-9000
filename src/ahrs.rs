use std::{
    error::Error,
    fs::File,
    io::Write,
    sync::{mpsc::{self, Sender, Receiver}, Arc, Mutex},
    thread,
    time::{Duration, Instant},
};

use rppal::{
    i2c::I2c,
    gpio::Gpio,
};

//////////////////////////////
// MPU-9250 register addresses
//  See here for a full list of addresses: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf
const AK8963_WHO_AM_I:    usize = 0x00; // should return 0x48
const AK8963_INFO:        usize = 0x01;
const AK8963_ST1:         usize = 0x02; // data ready status bit 0
const AK8963_XOUT_L:      usize = 0x03; // data
const AK8963_XOUT_H:      usize = 0x04;
const AK8963_YOUT_L:      usize = 0x05;
const AK8963_YOUT_H:      usize = 0x06;
const AK8963_ZOUT_L:      usize = 0x07;
const AK8963_ZOUT_H:      usize = 0x08;
const AK8963_ST2:         usize = 0x09; // data overflow bit 3 and data read error status bit 2
const AK8963_CNTL:        usize = 0x0A; // power down (0000), signle-measurement (0001), self-test (1000) and fuse ROM (1111) modes on bits 3:0
const AK8963_ASTC:        usize = 0x0C; // self test control
const AK8963_I2CDIS:      usize = 0x0F; // I2C disable
const AK8963_ASAX:        usize = 0x10; // fuse ROM x-axis sensitivity adjustment value
const AK8963_ASAY:        usize = 0x11; // fuse ROM y-axis sensitivity adjustment value
const AK8963_ASAZ:        usize = 0x12; // fuse ROM z-axis sensitivity adjustment value

const SELF_TEST_X_GYRO:   usize = 0x00;                 
const SELF_TEST_Y_GYRO:   usize = 0x01;                                                                         
const SELF_TEST_Z_GYRO:   usize = 0x02;

const SELF_TEST_X_ACCEL:  usize = 0x0D;
const SELF_TEST_Y_ACCEL:  usize = 0x0E;   
const SELF_TEST_Z_ACCEL:  usize = 0x0F;

const SELF_TEST_A:        usize = 0x10;

const XG_OFFSET_H:        usize = 0x13;  // User-defined trim values for gyroscope
const XG_OFFSET_L:        usize = 0x14;
const YG_OFFSET_H:        usize = 0x15;
const YG_OFFSET_L:        usize = 0x16;
const ZG_OFFSET_H:        usize = 0x17;
const ZG_OFFSET_L:        usize = 0x18;
const SMPLRT_DIV:         usize = 0x19;
const CONFIG:             usize = 0x1A;
const GYRO_CONFIG:        usize = 0x1B;
const ACCEL_CONFIG:       usize = 0x1C;
const ACCEL_CONFIG2:      usize = 0x1D;
const LP_ACCEL_ODR:       usize = 0x1E;  
const WOM_THR:            usize = 0x1F;

const MOT_DUR:            usize = 0x20;  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
const ZMOT_THR:           usize = 0x21;  // Zero-motion detection threshold bits [7:0]
const ZRMOT_DUR:          usize = 0x22;  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

const FIFO_EN:            usize = 0x23;
const I2C_MST_CTRL:       usize = 0x24;  
const I2C_SLV0_ADDR:      usize = 0x25;
const I2C_SLV0_REG:       usize = 0x26;
const I2C_SLV0_CTRL:      usize = 0x27;
const I2C_SLV1_ADDR:      usize = 0x28;
const I2C_SLV1_REG:       usize = 0x29;
const I2C_SLV1_CTRL:      usize = 0x2A;
const I2C_SLV2_ADDR:      usize = 0x2B;
const I2C_SLV2_REG:       usize = 0x2C;
const I2C_SLV2_CTRL:      usize = 0x2D;
const I2C_SLV3_ADDR:      usize = 0x2E;
const I2C_SLV3_REG:       usize = 0x2F;
const I2C_SLV3_CTRL:      usize = 0x30;
const I2C_SLV4_ADDR:      usize = 0x31;
const I2C_SLV4_REG:       usize = 0x32;
const I2C_SLV4_DO:        usize = 0x33;
const I2C_SLV4_CTRL:      usize = 0x34;
const I2C_SLV4_DI:        usize = 0x35;
const I2C_MST_STATUS:     usize = 0x36;
const INT_PIN_CFG:        usize = 0x37;
const INT_ENABLE:         usize = 0x38;
const DMP_INT_STATUS:     usize = 0x39;  // Check DMP interrupt
const INT_STATUS:         usize = 0x3A;
const ACCEL_XOUT_H:       usize = 0x3B;
const ACCEL_XOUT_L:       usize = 0x3C;
const ACCEL_YOUT_H:       usize = 0x3D;
const ACCEL_YOUT_L:       usize = 0x3E;
const ACCEL_ZOUT_H:       usize = 0x3F;
const ACCEL_ZOUT_L:       usize = 0x40;
const TEMP_OUT_H:         usize = 0x41;
const TEMP_OUT_L:         usize = 0x42;
const GYRO_XOUT_H:        usize = 0x43;
const GYRO_XOUT_L:        usize = 0x44;
const GYRO_YOUT_H:        usize = 0x45;
const GYRO_YOUT_L:        usize = 0x46;
const GYRO_ZOUT_H:        usize = 0x47;
const GYRO_ZOUT_L:        usize = 0x48;
const EXT_SENS_DATA_00:   usize = 0x49;
const EXT_SENS_DATA_01:   usize = 0x4A;
const EXT_SENS_DATA_02:   usize = 0x4B;
const EXT_SENS_DATA_03:   usize = 0x4C;
const EXT_SENS_DATA_04:   usize = 0x4D;
const EXT_SENS_DATA_05:   usize = 0x4E;
const EXT_SENS_DATA_06:   usize = 0x4F;
const EXT_SENS_DATA_07:   usize = 0x50;
const EXT_SENS_DATA_08:   usize = 0x51;
const EXT_SENS_DATA_09:   usize = 0x52;
const EXT_SENS_DATA_10:   usize = 0x53;
const EXT_SENS_DATA_11:   usize = 0x54;
const EXT_SENS_DATA_12:   usize = 0x55;
const EXT_SENS_DATA_13:   usize = 0x56;
const EXT_SENS_DATA_14:   usize = 0x57;
const EXT_SENS_DATA_15:   usize = 0x58;
const EXT_SENS_DATA_16:   usize = 0x59;
const EXT_SENS_DATA_17:   usize = 0x5A;
const EXT_SENS_DATA_18:   usize = 0x5B;
const EXT_SENS_DATA_19:   usize = 0x5C;
const EXT_SENS_DATA_20:   usize = 0x5D;
const EXT_SENS_DATA_21:   usize = 0x5E;
const EXT_SENS_DATA_22:   usize = 0x5F;
const EXT_SENS_DATA_23:   usize = 0x60;
const MOT_DETECT_STATUS:  usize = 0x61;
const I2C_SLV0_DO:        usize = 0x63;
const I2C_SLV1_DO:        usize = 0x64;
const I2C_SLV2_DO:        usize = 0x65;
const I2C_SLV3_DO:        usize = 0x66;
const I2C_MST_DELAY_CTRL: usize = 0x67;
const SIGNAL_PATH_RESET:  usize = 0x68;
const MOT_DETECT_CTRL:    usize = 0x69;
const USER_CTRL:          usize = 0x6A;  // Bit 7 enable DMP, bit 3 reset DMP
const PWR_MGMT_1:         usize = 0x6B; // Device defaults to the SLEEP mode
const PWR_MGMT_2:         usize = 0x6C;
const DMP_BANK:           usize = 0x6D;  // Activates a specific bank in the DMP
const DMP_RW_PNT:         usize = 0x6E;  // Set read/write pointer to a specific start address in specified DMP bank
const DMP_REG:            usize = 0x6F;  // Register in DMP from which to read or to which to write
const DMP_REG_1:          usize = 0x70;
const DMP_REG_2:          usize = 0x71; 
const FIFO_COUNTH:        usize = 0x72;
const FIFO_COUNTL:        usize = 0x73;
const FIFO_R_W:           usize = 0x74;
const WHO_AM_I_MPU9250:   usize = 0x75; // Should return 0x71
const XA_OFFSET_H:        usize = 0x77;
const XA_OFFSET_L:        usize = 0x78;
const YA_OFFSET_H:        usize = 0x7A;
const YA_OFFSET_L:        usize = 0x7B;
const ZA_OFFSET_H:        usize = 0x7D;
const ZA_OFFSET_L:        usize = 0x7E;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
const GYRO_MEAS_ERROR: f64 = std::f64::consts::PI * (40.0 / 180.0); // gryoscope measurement error in rad/s (start at 40 deg/s)
const GYRO_MEAS_DRIFT: f64 = std::f64::consts::PI * (0.0 / 180.0); // gryoscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

// Set initial input parameters
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Ascale {
    Afs2g = 0,
    Afs4g,
    Afs8g,
    Afs16g,
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum Gscale {
    Gfs250dps = 0,
    Gfs500dps,
    Gfs1000dps,
    Gfs2000dps,
}

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

// some things in this struct should be abstracted one layer deeper
pub struct Ahrs {
    // Sender sends data to the AHRS
    pub sender: Option<mpsc::Sender<crate::Message>>,
    // Receiver gets messages from the AHRS
    pub receiver: Option<mpsc::Receiver<crate::Message>>,
    // Thread reference; TODO - use this for doing elegant shutdowns
    thread: Option<thread::JoinHandle<()>>,
}

impl Ahrs {
    /// Create a new AHRS interface.  The AHRS spawns a thread which endlessly
    ///  computes new quaternions using input from the MPU to feed a sensor
    ///  fustion algorithm.  After each quaternion update, the AHRS thread will
    ///  check to see if new data has been requested; if so, the AHRS thread
    ///  will respond with the relevant response.
    ///
    /// # More Documentation
    ///
    /// Would go here eventually...
    pub fn new(time_string: &str, verbose: bool, i2c: Arc<Mutex<I2c>>) -> Result<Ahrs, Box<dyn Error>> {
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
        // let mut i2c = I2c::with_bus(0)?;

        // Set up IO connections
        let mut led = Gpio::new()
            .expect("GPIO Error: Unable to connect to GPIO manager.")
            .get(crate::GPIO_LED)
            .expect("GPIO Error: Unable to get GPIO_LED pin.")
            .into_output();
        
        // Set up connection to log file
        let log_file_name = format!("/home/ghost/rust-stuff/gpu_logs/{}_mpu_logfile.txt", time_string); 
        let mut log_file = File::create(log_file_name)
            .expect("Unable to create MPU logfile.");

        {
            // Aquire mutex for access to the I2C hardware
            let mut i2c = i2c.lock().unwrap();
            // Set the address of the MPU-9250
            i2c.set_slave_address(crate::ADDR_MPU9265)
                .expect("Unable to set I2C address.");

            // Read the WHO_AM_I register and check the result
            let mut reg = [0u8; 1];
            i2c.block_read(WHO_AM_I_MPU9250 as u8, &mut reg)
                .expect("I2C read error");

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
                    mag_bias = [-85.5859375, 286.69921875, 39.05859375];
                    mag_scale = [1.003875968992248, 0.9961538461538462, 1.0];
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
        }

        // Create configuration structure from initialization data
        // TODO - this should be stored as a property of the AHRS struct; it can be updated
        //  as necessary (e.g. magnetometer recalibration)
        let calibration = Mpu9250Calibration {
            a_res: get_a_res(&a_scale),
            g_res: get_g_res(&g_scale),
            m_res: get_m_res(&m_scale),
            mag_calibration: mag_calibration,
            mag_bias: mag_bias,
            mag_scale: mag_scale,
        };

        // Both the main and the satellite get a sender and receiver
        let (mosi_sender, mosi_receiver) = mpsc::channel();
        let (miso_sender, miso_receiver) = mpsc::channel();

        let quaternion_thread = thread::spawn(move ||
            compute_quaternions(
                i2c,
                &mut log_file,
                verbose,
                &calibration,
                mosi_receiver,
                miso_sender
            )
        );

        Ok(Ahrs {
            sender: Some(mosi_sender),
            receiver: Some(miso_receiver),
            thread: Some(quaternion_thread),
        })
    }

    pub fn get_latest_quaternion(&self) -> Vec<u8> {
        // Prepare request for new quaternion
        let message = crate::Message {
            address: 0x00,
            request: Vec::new(),
            response: Vec::new(),
        };
        // Send message to AHRS thread
        // TODO - understand why these as_ref's are necessary
        self.sender.as_ref().unwrap()
            .send(message).expect("Could not send data message to AHRS thread.");
        // Get return message from AHRS thread
        let return_message = self.receiver.as_ref().unwrap()
            .recv().expect("No response from AHRS thread.");
        // Return encoded quaternion
        return_message.response
    }
}

/// Compute quaternions forever
pub fn compute_quaternions(
    i2c: Arc<Mutex<I2c>>,
    log_file: &mut File,
    verbose: bool,
    calibration: &Mpu9250Calibration,
    receiver: Receiver<crate::Message>,
    sender: Sender<crate::Message>
    ) {
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

        // TODO - could probably get rid of this check and just assume that there is new data; check this against latest performance
        if interrupt_pin.is_high() { // If new data is available, read it in
            if mpu_reads < TOTAL_READS-1 {
                mpu_sample_start[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            {
                // Aquire mutex for access to the I2C hardware
                let mut i2c = i2c.lock().unwrap();
                // Set the address of the MPU-9250
                i2c.set_slave_address(crate::ADDR_MPU9265)
                    .expect("Unable to set I2C address.");
                // Read data from MPU
                read_mpu_data(&i2c, &mut raw_mpu_data)
                    .expect("I2C Error: Error reading from MPU.");
            }

            if mpu_reads < TOTAL_READS-1 {
                mpu_sample_stop[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }
            mpu_reads = mpu_reads + 1;

            ax = (raw_mpu_data[0] as f64) * calibration.a_res;
            ay = (raw_mpu_data[1] as f64) * calibration.a_res;
            az = (raw_mpu_data[2] as f64) * calibration.a_res;
            if verbose {
                write!(log_file, "DATA: accel_data, 3, 1\n")
                    .expect("IOError: Error writing to log file.");
                write!(log_file, "{} {} {}\n", ax, ay, az)
                    .expect("IOError: Error writing to log file.");
            }

            gx = (raw_mpu_data[4] as f64) * calibration.g_res;
            gy = (raw_mpu_data[5] as f64) * calibration.g_res;
            gz = (raw_mpu_data[6] as f64) * calibration.g_res;
            if verbose {
                write!(log_file, "DATA: gyro_data, 3, 1\n")
                    .expect("IOError: Error writing to log file.");
                write!(log_file, "{} {} {}\n", gx, gy, gz)
                    .expect("IOError: Error writing to log file.");
            }
        }

        {
            // Aquire mutex for access to the I2C hardware
            let mut i2c = i2c.lock().unwrap();
            // Set the address of the AK8963
            i2c.set_slave_address(crate::ADDR_AK8963)
                .expect("Unable to set I2C address.");
            i2c.block_read(AK8963_ST1 as u8, &mut dummy_data)
                .expect("I2C Error: Error reading from magnetometer.");


            if mpu_reads < TOTAL_READS-1 {
               mag_sample_start[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
            }

            // Read data from magnetometer
            read_mag_data(&i2c, &mut raw_mag_data)
                .expect("I2C Error: Error reading from magnetometer.");
        }

        if mpu_reads < TOTAL_READS-1 {
            mag_sample_stop[mpu_reads as usize] = start.elapsed().as_nanos() as u64;
        }

        let mx: f64 = ((raw_mag_data[0] as f64) * calibration.mag_calibration[0] - calibration.mag_bias[0])
            * calibration.mag_scale[0] * calibration.m_res;
        let my: f64 = ((raw_mag_data[1] as f64) * calibration.mag_calibration[1] - calibration.mag_bias[1])
            * calibration.mag_scale[1] * calibration.m_res;
        let mz: f64 = ((raw_mag_data[2] as f64) * calibration.mag_calibration[2] - calibration.mag_bias[2])
            * calibration.mag_scale[2] * calibration.m_res;

        if verbose {
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
        if verbose {
            write!(log_file, "DATA: update_period, 1, 1\n")
                .expect("IOError: Error writing to log file.");
            write!(log_file, "{}\n", delta_t)
                .expect("IOError: Error writing to log file.");
        }

        madgwick_quaternion_update(&[ax, ay, az], &[gx*std::f64::consts::PI/180.0, gy*std::f64::consts::PI/180.0, gz*std::f64::consts::PI/180.0], &[my, mx, -mz], &mut q, delta_t)
            .expect("IOError: Error in quaternion update.");

        if verbose {
            write!(log_file, "DATA: quat, 4, 1\n")
                .expect("IOError: Error writing to log file.");
            write!(log_file, "{} {} {} {}\n", q[0], q[1], q[2], q[3])
                .expect("IOError: Error writing to log file.");
        }

        // TODO - Refactor this to a pub fn get_message() which gets an Option<Message>
        let message = match receiver.try_recv() {
                Ok(ahrs_message) => Some(ahrs_message),
                Err(_error) => None,
        };

        // TODO - replace with an "if let"
        if message.is_some() {
            let mut unwrapped_message = message.unwrap();

            let message_response = match unwrapped_message.address {
                0x00 => get_encoded_quaternion(q),
                _ => "UNKNOWN ADDRESS".as_bytes().to_vec(),
            };

            unwrapped_message.response = message_response;

            sender.send(unwrapped_message)
                .expect("Could not send data from AHRS.");
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



fn get_a_res(a_scale: &Ascale) -> f64 {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs (11).
    match a_scale {
        Ascale::Afs2g => 2.0/32768.0,
        Ascale::Afs4g => 4.0/32768.0,
        Ascale::Afs8g => 8.0/32768.0,
        Ascale::Afs16g => 16.0/32768.0,
    }
}


fn get_g_res(g_scale: &Gscale) -> f64 {
    // Possible gyro scales (and their register bit settings) are:
    // 250DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    match g_scale {
        Gscale::Gfs250dps => 250.0/32768.0,
        Gscale::Gfs500dps => 500.0/32768.0,
        Gscale::Gfs1000dps => 1000.0/32768.0,
        Gscale::Gfs2000dps => 2000.0/32768.0,
    }
}


fn get_m_res(m_scale: &Mscale) -> f64 {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    match m_scale {
        Mscale::Mfs14bits => 10.0 * 4912.0 / 8190.0,
        Mscale::Mfs16bits => 10.0 * 4912.0 / 32760.0,
    }
}

fn get_encoded_quaternion(quaternion: [f64; 4]) -> Vec<u8> {
    let mut encoded_quaternion = Vec::new();
    for _i in 0..4 {
        // https://stackoverflow.com/questions/54142528/how-can-i-concatenate-two-slices-or-two-vectors-and-still-have-access-to-the-ori
        encoded_quaternion = encoded_quaternion.to_vec().into_iter().chain(quaternion[_i].to_be_bytes()).collect();
    }
    encoded_quaternion
}

/// Funciton which initializes the AK8963
fn init_ak8963(i2c: &I2c, m_scale: &Mscale, m_mode: u8) -> Result<([f64; 3]), Box<dyn Error>> {
    let mut mag_calibration: [f64; 3] = [0.0; 3];
    let mut raw_data: [u8; 3] = [0; 3];

    // frist extract the factory calibration for each magnetometer axis
    i2c.block_write(AK8963_CNTL as u8, &[0x00])?; // power down magnetometer
    thread::sleep(Duration::from_millis(30));
    i2c.block_write(AK8963_CNTL as u8, &[0x00])?; // enter fuse ROM access mode
    thread::sleep(Duration::from_millis(30));

    i2c.block_read(AK8963_ASAX as u8, &mut raw_data)?; // read the x-, y-, and z-axis calibration values
    mag_calibration[0] = (raw_data[0] as f64 - 128.0)/256.0 + 1.0; // return x-axis sensitivity adjustment values, etc.
    mag_calibration[1] = (raw_data[1] as f64 - 128.0)/256.0 + 1.0;
    mag_calibration[2] = (raw_data[2] as f64 - 128.0)/256.0 + 1.0;

    i2c.block_write(AK8963_CNTL as u8, &[0x00])?; // power down magnetometer
    thread::sleep(Duration::from_millis(30));

    // configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8Hz and 0110 for 100Hz sample rates
    raw_data[0] = (*m_scale as u8) << 4 | m_mode;
    i2c.block_write(AK8963_CNTL as u8, &mut raw_data[0..1])?; // power down magnetometer
    thread::sleep(Duration::from_millis(30));

    Ok(mag_calibration)
}


/// Funciton which accumulates magnetometer data and calculates bias and scale corrections
fn calibrate_ak8963(i2c: &I2c, mag_calibration: &[f64; 3], log_file: &mut File) -> Result<([f64; 3], [f64;3]), Box<dyn Error>> {
    let mut raw_mag_bias: [i32; 3] = [0; 3];
    let mut raw_mag_scale: [i32; 3] = [0; 3];
    let mut mag_bias: [f64; 3] = [0.0; 3];
    let mut mag_scale: [f64; 3] = [0.0; 3];
    let mut mag_max: [i16; 3] = [-32767, -32767, -32767];
    let mut mag_min: [i16; 3] = [32767, 32767, 32767];

    let mut raw_mag_data: [i16; 3] = [0; 3];

    println!("Begin magnetometer calibration.  Wave device in a figure eight until done.");
    thread::sleep(Duration::from_millis(4000));

    // shoot for ~fifteen seconds of mag data
    // TODO - get the m_mode from the register on the magnetometer and turn this back into an if statement
    let sample_count: u16 = 256;
    write!(log_file, "DATA: mag_cal, 3, {}\n", sample_count)
        .expect("Error writing to log file.");
    for _i in 0..sample_count {
        read_mag_data(&i2c, &mut raw_mag_data)?;
        for j in 0..3 {
            if raw_mag_data[j] > mag_max[j] {mag_max[j] = raw_mag_data[j];}
            if raw_mag_data[j] < mag_min[j] {mag_min[j] = raw_mag_data[j];}
        }
        write!(log_file, "{} {} {}\n", raw_mag_data[0], raw_mag_data[1], raw_mag_data[2])
            .expect("Error writing to log file.");
        thread::sleep(Duration::from_millis(135)); // at 8Hz ODR, new mag data is available every 125ms
    }

    // get hard iron correction
    raw_mag_bias[0] = (mag_max[0] as i32 + mag_min[0] as i32)/2; // get average x mag bias in counts
    raw_mag_bias[1] = (mag_max[1] as i32 + mag_min[1] as i32)/2; // get average y mag bias in counts
    raw_mag_bias[2] = (mag_max[2] as i32 + mag_min[2] as i32)/2; // get average z mag bias in counts

    mag_bias[0] = (raw_mag_bias[0] as f64) * mag_calibration[0]; // save mag biases in mG for main program
    mag_bias[1] = (raw_mag_bias[1] as f64) * mag_calibration[1];
    mag_bias[2] = (raw_mag_bias[2] as f64) * mag_calibration[2];

    // get soft iron correction
    raw_mag_scale[0] = (mag_max[0] as i32 - mag_min[0] as i32)/2; // get average x axis max chord length in counts
    raw_mag_scale[1] = (mag_max[1] as i32 - mag_min[1] as i32)/2; // get average y axis max chord length in counts
    raw_mag_scale[2] = (mag_max[2] as i32 - mag_min[2] as i32)/2; // get average z axis max chord length in counts

    let mut avg_reading: f64 = raw_mag_scale[0] as f64 + raw_mag_scale[1] as f64 + raw_mag_scale[2] as f64;
    avg_reading /= 3.0;

    mag_scale[0] = avg_reading/(raw_mag_scale[0] as f64);
    mag_scale[1] = avg_reading/(raw_mag_scale[1] as f64);
    mag_scale[2] = avg_reading/(raw_mag_scale[2] as f64);

    println!("Magnetometer calibration done!");

    Ok((mag_bias, mag_scale))
}


/// Function to read 6 bytes of data from the magnetometer and store the result in a 3-element array
fn read_mag_data(i2c: &I2c, raw_mag_data: &mut [i16; 3]) -> Result<(), Box<dyn Error>> {
    let mut raw_data: [u8; 7] = [0; 7];
    i2c.block_read(AK8963_ST1 as u8, &mut raw_data[0..1])?;
    if raw_data[0] & 0x01 == 0x01 {
        i2c.block_read(AK8963_XOUT_L as u8, &mut raw_data)?;
        if !(raw_data[6] & 0x08 == 0x08) { // check if magnetic sensor overflow is set, if not then report data
            raw_mag_data[0] = (raw_data[1] as i16) << 8 | raw_data[0] as i16; // turn the MSB and LSB into a signed 16-bit value
            raw_mag_data[1] = (raw_data[3] as i16) << 8 | raw_data[2] as i16; // data stored as little Endian
            raw_mag_data[2] = (raw_data[5] as i16) << 8 | raw_data[4] as i16;
        }
    }

    Ok(())
}


/// Function to read 14 byes of data from the MPU
fn read_mpu_data(i2c: &I2c, raw_mpu_data: &mut [i16; 7]) -> Result<(), Box<dyn Error>> {
    let mut raw_data: [u8; 14] = [0; 14];

    i2c.block_read(ACCEL_XOUT_H as u8, &mut raw_data)?;
    raw_mpu_data[0] = (raw_data[0] as i16) << 8 | raw_data[1] as i16; // turn the MSB and LSB into a signed 16-bit value
    raw_mpu_data[1] = (raw_data[2] as i16) << 8 | raw_data[3] as i16;
    raw_mpu_data[2] = (raw_data[4] as i16) << 8 | raw_data[5] as i16;
    raw_mpu_data[3] = (raw_data[6] as i16) << 8 | raw_data[7] as i16;
    raw_mpu_data[4] = (raw_data[8] as i16) << 8 | raw_data[9] as i16;
    raw_mpu_data[5] = (raw_data[10] as i16) << 8 | raw_data[11] as i16;
    raw_mpu_data[6] = (raw_data[12] as i16) << 8 | raw_data[13] as i16;

    Ok(())
}


/// Funciton which initializes the MPU9250
fn init_mpu9250(i2c: &I2c, a_scale: &Ascale, g_scale: &Gscale) -> Result<(), Box<dyn Error>> {
    // wake up device
    i2c.block_write(PWR_MGMT_1 as u8, &[0x00])?; // clear sleep mode bit (6), enable all sensors
    thread::sleep(Duration::from_millis(100)); // wait for all registers to reset

    // get stable time source
    i2c.block_write(PWR_MGMT_1 as u8, &[0x01])?; // auto select clock source to be PLL gyroscope reference if ready
    thread::sleep(Duration::from_millis(200));

    // configure gyro and thermometer
    // disable FSYNC and set thermometer and gyro bandwidth to 41Hz and 42Hz, respectively
    // minimum delay time for this seetting is 5.9ms, which means sensor funsion update rates cannot
    // be higher than 1 / 0.0059 = 170Hz
    // DLPF_CFG = bits 2:0 = 011; this lmits the sample rate to 1000Hz for both
    // with the MPU9250, it is possible to get gyro sample rates of 32kHz (!), 8kHz, or 1kHz
    i2c.block_write(CONFIG as u8, &[0x03])?;

    // set sample rate = gyroscope output rate/ (1 + SMPLRT_DIV)
    //i2c.block_write(SMPLRT_DIV as u8, &[0x04])?; // use a 200 Hz rate; a rate consistent with the fitler update rate
    //                                         // determined inset in CONFIG above
    i2c.block_write(SMPLRT_DIV as u8, &[0x00])?;

    // set gyroscope full scale range
    // range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    let mut raw_data: [u8; 1] = [0];
    i2c.block_read(GYRO_CONFIG as u8, &mut raw_data)?; // get current GYRO_CONFIG register value
    raw_data[0] = raw_data[0] & !0x03; // clear Fchoice bits [1:0]
    raw_data[0] = raw_data[0] & !0x18; // clear GFS bits [4:3]
    raw_data[0] = raw_data[0] | (*g_scale as u8) << 3; // set full scale range for the gyro
    i2c.block_write(GYRO_CONFIG as u8, &mut raw_data)?; // write new GYRO_CONFIG value to register

    // set accelerometer full-scale range configuration
    i2c.block_read(ACCEL_CONFIG as u8, &mut raw_data)?; // get current ACCEL_CONFIG register value
    raw_data[0] = raw_data[0] & !0x18; // clear AFS bits [4:3]
    raw_data[0] = raw_data[0] | (*a_scale as u8) << 3; // set full scale range for the accelerometer
    i2c.block_write(ACCEL_CONFIG as u8, &mut raw_data)?; // write new ACCEL_CONFIG value to register

    // set accelerometer sample rate configuration
    // it is possible to get a 4kHz sample rate from the accelerometer by choosing 1 for
    // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13kHz
    i2c.block_read(ACCEL_CONFIG2 as u8, &mut raw_data)?;
    raw_data[0] = raw_data[0] & !0x0F; // clear accel_fchoice_b (bit 3) and A_DLPFGFG (bits [2:0])
    raw_data[0] = raw_data[0] | 0x03; // set accelerometer rate to 1kHz and bandiwdth to 41Hz
    i2c.block_write(ACCEL_CONFIG2 as u8, &mut raw_data)?; // write new ACCEL_CONFIG2 register value

    // the accelerometer, gyro, and thermometer are set to 1kHz sample rates,
    // but all these rates are further reduced by a factor of 5 to 200Hz because of the SMPLRT_DIV setting

    // configure interrupts and bypass enable
    // set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
    // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the RPi as master
    i2c.block_write(INT_PIN_CFG as u8, &[0x32])?; // latch INT pin high and any read to clear; set bypass_en
    i2c.block_write(INT_ENABLE as u8, &[0x01])?; // enable data ready (bit 0) interrupt
    thread::sleep(Duration::from_millis(100));

    Ok(())
}


/// Funciton which accumulates gyro and accelerometer data after device initialization. It calculates the average
/// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
fn calibrate_mpu9250(i2c: &I2c, log_file: &mut File) -> Result<(), Box<dyn Error>> {
    let mut raw_data: [u8; 12] = [0; 12];
    let mut gyro_bias: [i32; 3] = [0; 3];
    let mut accel_bias: [i32; 3] = [0; 3];
    let mut scaled_gyro_bias: [f64; 3] = [0.0; 3];
    let mut scaled_accel_bias: [f64; 3] = [0.0; 3];
    let mut calibrated_scaled_gyro_bias: [f64; 3] = [0.0; 3];
    let mut calibrated_scaled_accel_bias: [f64; 3] = [0.0; 3];

    i2c.block_write(PWR_MGMT_1 as u8, &[0x80])?; // Reset device
    thread::sleep(Duration::from_millis(200)); // Delay a while to let the device stabilize

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    i2c.block_write(PWR_MGMT_1 as u8, &[0x01])?;
    i2c.block_write(PWR_MGMT_2 as u8, &[0x00])?;
    thread::sleep(Duration::from_millis(200));

    // configure device for bias calculation
    i2c.block_write(INT_ENABLE as u8, &[0x00])?; // disable all interrupts
    i2c.block_write(FIFO_EN as u8, &[0x00])?; // diable FIFO
    i2c.block_write(PWR_MGMT_1 as u8, &[0x00])?; // turn on internal clock source
    i2c.block_write(I2C_MST_CTRL as u8, &[0x00])?; //disable I2C master
    i2c.block_write(USER_CTRL as u8, &[0x00])?; // disable FIFO and I2C master modes
    // NOTE - I don't think the register below should be 0x0C, I think it should be 0x40
    i2c.block_write(USER_CTRL as u8, &[0x0C])?; // reset FIFO and DMP
    thread::sleep(Duration::from_millis(15));

    // configure MPU6050 gyro and accelerometer for bias calculation
    // NOTE - with the settings below, the gyro has a LPF at 184 Hz, and the temperature sensor has a LPF of 188Hz
    i2c.block_write(CONFIG as u8, &[0x01])?; // set low-pass filter to 188Hz
    // NOTE - the line below specifies no division of the internal sample rate of 1kHz
    i2c.block_write(SMPLRT_DIV as u8, &[0x00])?; // set sample rate to 1kHz
    i2c.block_write(GYRO_CONFIG as u8, &[0x00])?; // set gyro full-scale to 250 degrees per second, maximum sensitivity
    i2c.block_write(ACCEL_CONFIG as u8, &[0x00])?; // set accelerometer full-scale to 2g, maximum sensitivity

    // NOTE - the values below can be read off the datasheet given the register settings above
    let gyrosensitivity: u16 = 131; // = 131 LSB/degrees/sec
    let accelsensitivity: u16 = 16384; // = 16384 LSB/g

    // configure FIFO to capture accelerometer and gyro data for bias calculation
    i2c.block_write(USER_CTRL as u8, &[0x40])?; // enable FIFO
    i2c.block_write(FIFO_EN as u8, &[0x78])?; // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9150)
    thread::sleep(Duration::from_millis(40)); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // at the end of the sample accumulation, turn off FIFO sensor read
    i2c.block_write(FIFO_EN as u8, &[0x00])?; // disable gyro and accelerometer sensors for FIFO
    i2c.block_read(FIFO_COUNTH as u8, &mut raw_data[0..2])?; // read FIFO sample count
    let fifo_count: u16 = (raw_data[0] as u16) << 8 | (raw_data[1] as u16);
    let packet_count: u16 = fifo_count/12; // how many sets of full gyro and accelerometer data for averaging

    let mut accel_temp: [i16; 3] = [0; 3];
    let mut gyro_temp: [i16; 3] = [0; 3];

    for _i in 0..packet_count {
        i2c.block_read(FIFO_R_W as u8, &mut raw_data)?; // read data for averaging
        accel_temp[0] = (raw_data[0] as i16) << 8 | raw_data[1] as i16; // form signed 16-bit integer for each sample if FIFO
        accel_temp[1] = (raw_data[2] as i16) << 8 | raw_data[3] as i16;
        accel_temp[2] = (raw_data[4] as i16) << 8 | raw_data[5] as i16;
        gyro_temp[0] = (raw_data[6] as i16) << 8 | raw_data[7] as i16;
        gyro_temp[1] = (raw_data[8] as i16) << 8 | raw_data[9] as i16;
        gyro_temp[2] = (raw_data[10] as i16) << 8 | raw_data[11] as i16;

        accel_bias[0] += accel_temp[0] as i32;
        accel_bias[1] += accel_temp[1] as i32;
        accel_bias[2] += accel_temp[2] as i32;
        gyro_bias[0] += gyro_temp[0] as i32;
        gyro_bias[1] += gyro_temp[1] as i32;
        gyro_bias[2] += gyro_temp[2] as i32;
    }

    accel_bias[0] /= packet_count as i32; // normalizes sums to get average count biases
    accel_bias[1] /= packet_count as i32;
    accel_bias[2] /= packet_count as i32;
    gyro_bias[0] /= packet_count as i32;
    gyro_bias[1] /= packet_count as i32;
    gyro_bias[2] /= packet_count as i32;

    // remove gravity from the z-axis accelerometer bias calculation; assumes unit is sitting face up
    accel_bias[2] -= accelsensitivity as i32;

    // construct the gyro biases for pushing to the hardware bias registers, which are reset to zero upon device startup
    raw_data[0] = ((-gyro_bias[0]/4 >> 8) & 0xFF) as u8; // divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    raw_data[1] = ((-gyro_bias[0]/4     ) & 0xFF) as u8; // biases are additive, so change sign on calculated average gyro biases
    raw_data[2] = ((-gyro_bias[1]/4 >> 8) & 0xFF) as u8;
    raw_data[3] = ((-gyro_bias[1]/4     ) & 0xFF) as u8;
    raw_data[4] = ((-gyro_bias[2]/4 >> 8) & 0xFF) as u8;
    raw_data[5] = ((-gyro_bias[2]/4     ) & 0xFF) as u8;

    // push gyro biases to hardware registers
    i2c.block_write(XG_OFFSET_H as u8, &mut raw_data[0..1])?;
    i2c.block_write(XG_OFFSET_L as u8, &mut raw_data[1..2])?;
    i2c.block_write(YG_OFFSET_H as u8, &mut raw_data[2..3])?;
    i2c.block_write(YG_OFFSET_L as u8, &mut raw_data[3..4])?;
    i2c.block_write(ZG_OFFSET_H as u8, &mut raw_data[4..5])?;
    i2c.block_write(ZG_OFFSET_L as u8, &mut raw_data[5..6])?;

    // output scaled gyro biases for display in the main program
    scaled_gyro_bias[0] = gyro_bias[0] as f64 / gyrosensitivity as f64;
    scaled_gyro_bias[1] = gyro_bias[1] as f64 / gyrosensitivity as f64;
    scaled_gyro_bias[2] = gyro_bias[2] as f64 / gyrosensitivity as f64;

    write!(log_file, "DEBUG: Scaled gyro biases (before calibration): {:?}\n", scaled_gyro_bias)
        .expect("Error writing to log file.");

    // construct the accelerometer biases for pushing to the hardware accelerometer ias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower bytes must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.
    let mut accel_bias_reg: [i32; 3] = [0; 3]; // sotre the factory acceleromeer trim biases
    i2c.block_read(XA_OFFSET_H as u8, &mut raw_data[0..2])?; // read factory accelerometr trim values
    i2c.block_read(YA_OFFSET_H as u8, &mut raw_data[2..4])?;
    i2c.block_read(ZA_OFFSET_H as u8, &mut raw_data[4..6])?;

    accel_bias_reg[0] = ((raw_data[0] as i16) << 8 | (raw_data[1] as i16)) as i32;
    accel_bias_reg[1] = ((raw_data[2] as i16) << 8 | (raw_data[3] as i16)) as i32;
    accel_bias_reg[2] = ((raw_data[4] as i16) << 8 | (raw_data[5] as i16)) as i32;

    let mask: i32 = 1; // define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    let mut mask_bit: [u8; 3] = [0; 3];

    for i in 0..3 {
        if accel_bias_reg[i] & mask == 1 {
            mask_bit[i] = 0xFF;
        } else {
            mask_bit[i] = 0xFE;
        }
    }

    accel_bias_reg[0] -= accel_bias[0]/8;
    accel_bias_reg[1] -= accel_bias[1]/8;
    accel_bias_reg[2] -= accel_bias[2]/8;

    raw_data[0] = ((accel_bias_reg[0] >> 8) & 0xFF) as u8;
    raw_data[1] = ((accel_bias_reg[0]     ) & 0xFF) as u8;
    raw_data[1] = raw_data[1] & mask_bit[0];
    raw_data[2] = ((accel_bias_reg[1] >> 8) & 0xFF) as u8;
    raw_data[3] = ((accel_bias_reg[1]     ) & 0xFF) as u8;
    raw_data[3] = raw_data[3] & mask_bit[1];
    raw_data[4] = ((accel_bias_reg[2] >> 8) & 0xFF) as u8;
    raw_data[5] = ((accel_bias_reg[2]     ) & 0xFF) as u8;
    raw_data[5] = raw_data[5] & mask_bit[2];

    // push accelerometer biases to hardware registers
    i2c.block_write(XA_OFFSET_H as u8, &mut raw_data[0..1])?;
    i2c.block_write(XA_OFFSET_L as u8, &mut raw_data[1..2])?;
    i2c.block_write(YA_OFFSET_H as u8, &mut raw_data[2..3])?;
    i2c.block_write(YA_OFFSET_L as u8, &mut raw_data[3..4])?;
    i2c.block_write(ZA_OFFSET_H as u8, &mut raw_data[4..5])?;
    i2c.block_write(ZA_OFFSET_L as u8, &mut raw_data[5..6])?;

    // output scaled accelerometer biases for display in the main program
    scaled_accel_bias[0] = accel_bias[0] as f64 / accelsensitivity as f64;
    scaled_accel_bias[1] = accel_bias[1] as f64 / accelsensitivity as f64;
    scaled_accel_bias[2] = accel_bias[2] as f64 / accelsensitivity as f64;
    write!(log_file, "DEBUG: Scaled accelerometer biases (before calibration): {:?}\n", scaled_accel_bias)
        .expect("Error writing to log file.");

    // re-measure the biases

    // configure device for bias calculation
    i2c.block_write(FIFO_EN as u8, &[0x00])?; // diable FIFO
    i2c.block_write(PWR_MGMT_1 as u8, &[0x00])?; // turn on internal clock source
    i2c.block_write(I2C_MST_CTRL as u8, &[0x00])?; //disable I2C master
    i2c.block_write(USER_CTRL as u8, &[0x00])?; // disable FIFO and I2C master modes
    // NOTE - I don't think the register below should be 0x0C, I think it should be 0x40
    i2c.block_write(USER_CTRL as u8, &[0x0C])?; // reset FIFO and DMP
    thread::sleep(Duration::from_millis(15));

    // configure MPU6050 gyro and accelerometer for bias calculation
    // NOTE - with the settings below, the gyro has a LPF at 184 Hz, and the temperature sensor has a LPF of 188Hz
    i2c.block_write(CONFIG as u8, &[0x01])?; // set low-pass filter to 188Hz
    // NOTE - the line below specifies no division of the internal sample rate of 1kHz
    i2c.block_write(SMPLRT_DIV as u8, &[0x00])?; // set sample rate to 1kHz
    i2c.block_write(GYRO_CONFIG as u8, &[0x00])?; // set gyro full-scale to 250 degrees per second, maximum sensitivity
    i2c.block_write(ACCEL_CONFIG as u8, &[0x00])?; // set accelerometer full-scale to 2g, maximum sensitivity

    // configure FIFO to capture accelerometer and gyro data for bias calculation
    i2c.block_write(USER_CTRL as u8, &[0x40])?; // enable FIFO
    i2c.block_write(FIFO_EN as u8, &[0x78])?; // enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9150)
    thread::sleep(Duration::from_millis(40)); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // at the end of the sample accumulation, turn off FIFO sensor read
    i2c.block_write(FIFO_EN as u8, &[0x00])?; // disable gyro and accelerometer sensors for FIFO
    i2c.block_read(FIFO_COUNTH as u8, &mut raw_data[0..2])?; // read FIFO sample count
    let fifo_count: u16 = (raw_data[0] as u16) << 8 | (raw_data[1] as u16);
    let packet_count: u16 = fifo_count/12; // how many sets of full gyro and accelerometer data for averaging

    for _i in 0..packet_count {
        i2c.block_read(FIFO_R_W as u8, &mut raw_data)?; // read data for averaging
        accel_temp[0] = (raw_data[0] as i16) << 8 | raw_data[1] as i16; // form signed 16-bit integer for each sample if FIFO
        accel_temp[1] = (raw_data[2] as i16) << 8 | raw_data[3] as i16;
        accel_temp[2] = (raw_data[4] as i16) << 8 | raw_data[5] as i16;
        gyro_temp[0] = (raw_data[6] as i16) << 8 | raw_data[7] as i16;
        gyro_temp[1] = (raw_data[8] as i16) << 8 | raw_data[9] as i16;
        gyro_temp[2] = (raw_data[10] as i16) << 8 | raw_data[11] as i16;

        accel_bias[0] += accel_temp[0] as i32;
        accel_bias[1] += accel_temp[1] as i32;
        accel_bias[2] += accel_temp[2] as i32;
        gyro_bias[0] += gyro_temp[0] as i32;
        gyro_bias[1] += gyro_temp[1] as i32;
        gyro_bias[2] += gyro_temp[2] as i32;
    }

    accel_bias[0] /= packet_count as i32; // normalizes sums to get average count biases
    accel_bias[1] /= packet_count as i32;
    accel_bias[2] /= packet_count as i32;
    gyro_bias[0] /= packet_count as i32;
    gyro_bias[1] /= packet_count as i32;
    gyro_bias[2] /= packet_count as i32;

    // remove gravity from the z-axis accelerometer bias calculation; assumes unit is sitting face up
    accel_bias[2] -= accelsensitivity as i32;

    calibrated_scaled_gyro_bias[0] = gyro_bias[0] as f64 / gyrosensitivity as f64;
    calibrated_scaled_gyro_bias[1] = gyro_bias[1] as f64 / gyrosensitivity as f64;
    calibrated_scaled_gyro_bias[2] = gyro_bias[2] as f64 / gyrosensitivity as f64;
    write!(log_file, "DEBUG: Scaled gyro biases (after calibration): {:?}\n", calibrated_scaled_gyro_bias)
        .expect("Error writing to log file.");

    calibrated_scaled_accel_bias[0] = accel_bias[0] as f64 / accelsensitivity as f64;
    calibrated_scaled_accel_bias[1] = accel_bias[1] as f64 / accelsensitivity as f64;
    calibrated_scaled_accel_bias[2] = accel_bias[2] as f64 / accelsensitivity as f64;

    write!(log_file, "DEBUG: Scaled accelerometer biases (after calibration): {:?}\n", calibrated_scaled_accel_bias)
        .expect("Error writing to log file.");
    
    ///////////////

    Ok(())
}


/// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
/// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
/// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
/// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
/// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
/// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
fn madgwick_quaternion_update(accel_data: &[f64; 3], gyro_data: &[f64; 3], mag_data: &[f64; 3], q: &mut [f64; 4], delta_t: f64) -> Result<(), Box<dyn Error>> {
    // There is a tradeoff in the beta parameter between accuracy and response speed.
    // In the original Madgwick study, beta of 0.041 (corresponding to GYRO_MEAS_ERROR of 2.7 degrees/s) was found to give optimal accuracy.
    // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
    // Subsequent changes also require a longish lag time to a stable output.
    // By increasing beta (GYRO_MEAS_ERROR) by about a factor of fifteen, the response time constant is reduced to ~2 sec.
    // This is the free parameter in the Madgwick filtering and fusion scheme.
    let beta        : f64      = f64::sqrt(3.0/4.0) * GYRO_MEAS_ERROR; // compute beta
    // let zeta        : f64      = f64::sqrt(3.0/4.0) * GYRO_MEAS_DRIFT; // compute zeta
    // Calculated values
    let mut a_norm  : [f64; 3] = [0.0; 3];
    let mut m_norm  : [f64; 3] = [0.0; 3];
    // Auxiliary variables to avoid repeated arithmetic
    let mut _2q0mx  : f64      = 0.0;
    let mut _2q0my  : f64      = 0.0;
    let mut _2q0mz  : f64      = 0.0;
    let mut _2q1mx  : f64      = 0.0;
    let mut _2bx    : f64      = 0.0;
    let mut _2bz    : f64      = 0.0;
    let mut _4bx    : f64      = 0.0;
    let mut _4bz    : f64      = 0.0;
    let _2q0        : f64      = 2.0 * q[0];
    let _2q1        : f64      = 2.0 * q[1];
    let _2q2        : f64      = 2.0 * q[2];
    let _2q3        : f64      = 2.0 * q[3];
    let _2q0q2      : f64      = 2.0 * q[0] * q[2];
    let _2q2q3      : f64      = 2.0 * q[2] * q[3];
    let q0q0        : f64      = q[0] * q[0];
    let q0q1        : f64      = q[0] * q[1];
    let q0q2        : f64      = q[0] * q[2];
    let q0q3        : f64      = q[0] * q[3];
    let q1q1        : f64      = q[1] * q[1];
    let q1q2        : f64      = q[1] * q[2];
    let q1q3        : f64      = q[1] * q[3];
    let q2q2        : f64      = q[2] * q[2];
    let q2q3        : f64      = q[2] * q[3];
    let q3q3        : f64      = q[3] * q[3];

    // Normalize accelerometer data
    let norm: f64 = f64::sqrt(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    if norm == 0.0 {
        //return Err(<dyn Error>::new(ErrorKind::Other, "Norm of accelerometer data is zero!");
        println!("Quaternion update failed!");
        return Ok(());
    }
    let inv_norm: f64 = 1.0/norm;
    a_norm[0] = accel_data[0] * inv_norm;
    a_norm[1] = accel_data[1] * inv_norm;
    a_norm[2] = accel_data[2] * inv_norm;

    // Normalize magnetometer data
    let norm: f64 = f64::sqrt(mag_data[0] * mag_data[0] + mag_data[1] * mag_data[1] + mag_data[2] * mag_data[2]);
    if norm == 0.0 {
        //return Err(<dyn Error>::new(ErrorKind::Other, "Norm of magnetometer data is zero!");
        println!("Quaternion update failed!");
        return Ok(());
    }
    let inv_norm: f64 = 1.0/norm;
    m_norm[0] = mag_data[0] * inv_norm;
    m_norm[1] = mag_data[1] * inv_norm;
    m_norm[2] = mag_data[2] * inv_norm;

    // Reference direction of Earth's magnetic field
    _2q0mx = 2.0 * q[0] * m_norm[0];
    _2q0my = 2.0 * q[0] * m_norm[1];
    _2q0mz = 2.0 * q[0] * m_norm[2];
    _2q1mx = 2.0 * q[1] * m_norm[0];

    let hx: f64 = m_norm[0] * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + m_norm[0] * q1q1 + _2q1 * m_norm[1] * q[2] + _2q1 * m_norm[2] * q[3] - m_norm[0] * q2q2 - m_norm[0] * q3q3;
    let hy: f64 = _2q0mx * q[3] + m_norm[1] * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - m_norm[1] * q1q1 + m_norm[1] * q2q2 + _2q2 * m_norm[2] * q[3] - m_norm[1] * q3q3;
    _2bx = f64::sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * q[2] + _2q0my * q[1] + m_norm[2] * q0q0 + _2q1mx * q[3] - m_norm[2] * q1q1 + _2q2 * m_norm[1] * q[3] - m_norm[2] * q2q2 + m_norm[2] * q3q3;
    _4bx = 2.0 * _2bx;
    _4bz = 2.0 * _2bz;

    // Gradient decent algorithm corrective step
    let mut s0: f64 = -_2q2 * (2.0 * q1q3 - _2q0q2 - a_norm[0]) + _2q1 * (2.0 * q0q1 + _2q2q3 - a_norm[1]) - _2bz * q[2] * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_norm[0]) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_norm[1]) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m_norm[2]);
    let mut s1: f64 = _2q3 * (2.0 * q1q3 - _2q0q2 - a_norm[0]) + _2q0 * (2.0 * q0q1 + _2q2q3 - a_norm[1]) - 4.0 * q[1] * (1.0 - 2.0 * q1q1 - 2. * q2q2 - a_norm[2]) + _2bz * q[3] * (_2bx * (0.5 - q2q2- q3q3) + _2bz * (q1q3 - q0q2) - m_norm[0]) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_norm[1]) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m_norm[2]);
    let mut s2: f64 = -_2q0 * (2.0 * q1q3 - _2q0q2 - a_norm[0]) + _2q3 * (2.0 * q0q1 + _2q2q3 - a_norm[1]) - 4.0 * q[2] * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - a_norm[2]) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_norm[0]) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_norm[1]) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m_norm[2]);
    let mut s3: f64 = _2q1 * (2.0 * q1q3 - _2q0q2 - a_norm[0]) + _2q2 * (2.0 * q0q1 + _2q2q3 - a_norm[1]) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - m_norm[0]) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - m_norm[1]) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.5 - q1q1 - q2q2) - m_norm[2]);
    let norm: f64 = f64::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalise step magnitude
    let inv_norm: f64 = 1.0/norm;
    s0 = s0 * inv_norm;
    s1 = s1 * inv_norm;
    s2 = s2 * inv_norm;
    s3 = s3 * inv_norm;

    // Compute rate of change of quaternion
    let qdot0: f64 = 0.5 * (-q[1] * gyro_data[0] - q[2] * gyro_data[1] - q[3] * gyro_data[2]) - beta * s0;
    let qdot1: f64 = 0.5 * ( q[0] * gyro_data[0] + q[2] * gyro_data[2] - q[3] * gyro_data[1]) - beta * s1;
    let qdot2: f64 = 0.5 * ( q[0] * gyro_data[1] - q[1] * gyro_data[2] + q[3] * gyro_data[0]) - beta * s2;
    let qdot3: f64 = 0.5 * ( q[0] * gyro_data[2] + q[1] * gyro_data[1] - q[2] * gyro_data[0]) - beta * s3;

    // Integrate to yield quaternion
    q[0] = q[0] + qdot0 * delta_t;
    q[1] = q[1] + qdot1 * delta_t;
    q[2] = q[2] + qdot2 * delta_t;
    q[3] = q[3] + qdot3 * delta_t;
    let norm: f64 = f64::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]); // Normalise step magnitude
    let inv_norm: f64 = 1.0/norm;
    q[0] = q[0] * inv_norm;
    q[1] = q[1] * inv_norm;
    q[2] = q[2] * inv_norm;
    q[3] = q[3] * inv_norm;

    Ok(())
}
