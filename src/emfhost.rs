use std::{
    fs::File,
    io::Write,
    sync::mpsc::{self, Sender, Receiver},
    thread,
    time::{Duration, Instant},
};

use rppal::i2c::I2c;

pub struct EmfHost {
    // Sender sends data to the Emf thread
    pub sender: Option<mpsc::Sender<crate::Message>>,
    // Receiver gets messages from the Emf thread
    pub receiver: Option<mpsc::Receiver<crate::Message>>,
    // Thread reference; TODO - use this for doing elegant shutdowns
    thread: Option<thread::JoinHandle<()>>,
}

impl EmfHost {
    /// Create a new EmfHost interface.  The EMFHost spawns a thread which
    ///  waits to receive data requests.  When a request is received, an ADC
    ///  signal is read at a fixed sample rate.  The resulting waveform is sent
    ///  back to the host process.
    ///
    /// # More Documentation
    ///
    /// Would go here eventually...
    pub fn new(time_string: &str) -> EmfHost {
        // Connect using I2C bus #0; this will soon be replaced with a mutex reference
        let mut i2c = I2c::with_bus(0)
            .expect("Could not create I2C device.");
        
        // Set up connection to log file
        let log_file_name = format!("/home/ghost/rust-stuff/gpu_logs/{}_emf_logfile.txt", time_string); 
        let mut log_file = File::create(log_file_name)
            .expect("Unable to create EMF logfile.");

        // Set the address of the ADS1115
        i2c.set_slave_address(crate::ADDR_ADS1115)
            .expect("Unable to set I2C address.");

        // Configure the ADC mdoule
        configure_ads1115(&i2c, &mut log_file);

        // Both the main and the satellite get a sender and receiver
        let (mosi_sender, mosi_receiver) = mpsc::channel();
        let (miso_sender, miso_receiver) = mpsc::channel();

        let emfhost_thread = thread::spawn(move || read_emf_data(&mut i2c, &mut log_file, mosi_receiver, miso_sender));

        EmfHost {
            sender: Some(mosi_sender),
            receiver: Some(miso_receiver),
            thread: Some(emfhost_thread),
        }
    }

    pub fn get_adc_voltage(&self) -> Vec<u8> {
        // Prepare request for ADC voltage data
        let message = crate::Message {
            address: 0x00,
            request: Vec::new(),
            response: Vec::new(),
        };
        // Send message to EMF thread
        // TODO - understand why these as_ref's are necessary
        self.sender.as_ref().unwrap()
            .send(message).expect("Could not send data message to EMF thread.");
        // Get return message from EMF thread
        let return_message = self.receiver.as_ref().unwrap()
            .recv().expect("No response from EMF thread.");
        // Return encoded voltage data
        return_message.response
    }
}

/// Compute quaternions forever
pub fn read_emf_data(i2c: &mut I2c, log_file: &mut File, receiver: Receiver<crate::Message>, sender: Sender<crate::Message>) {
    let start = Instant::now();
    // let mut last_update_time: u64 = 0;

    // Variables for monitoring the acquisition rate
    let mut iterations: u32 = 0;
    // let mut adc_reads: u32 = 0;

    // TODO - there should be a way out of this loop when a close method is invoked
    loop {
        // TODO - Refactor this to a pub fn get_message() which gets an Option<Message>
        let message = match receiver.try_recv() {
                Ok(ahrs_message) => Some(ahrs_message),
                Err(_error) => None,
        };

        // TODO - replace with an "if let"
        if message.is_some() {
            let mut unwrapped_message = message.unwrap();

            let message_response = match unwrapped_message.address {
                0x00 => read_adc_voltage(i2c).to_be_bytes().into_iter().collect(),
                _ => "UNKNOWN ADDRESS".as_bytes().to_vec(),
            };

            unwrapped_message.response = message_response;

            sender.send(unwrapped_message)
                .expect("Could not send data from EMF host.");
        }
        // adc_reads = adc_reads + 1;

        // let current_update_time: u64 = start.elapsed().as_nanos() as u64;
        // let delta_t: f64 = (current_update_time - last_update_time) as f64 / 1000000000.0;
        // last_update_time = current_update_time;
        // write!(log_file, "DATA: update_period, 1, 1\n")
        //     .expect("IOError: Error writing to log file.");
        // write!(log_file, "{}\n", delta_t)
        //     .expect("IOError: Error writing to log file.");

        thread::sleep(Duration::from_millis(50));

        iterations = iterations + 1;
    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed)
        .expect("Error writing to log file.");
    write!(log_file, "DEBUG: Loop iterations: {}\n", iterations)
        .expect("Error writing to log file.");
    // write!(log_file, "DEBUG: ADC reads: {}\n", adc_reads)
    //     .expect("Error writing to log file.");
}

/// Funciton which configures the ADS1115
fn configure_ads1115(i2c: &I2c, log_file: &mut File) {
    // Read from config register
    let mut reg = [0u8; 2];
    i2c.block_write(0b10010001 as u8, &[])
        .expect("I2C write error");
    i2c.block_read(0b00000001u8, &mut reg)
        .expect("I2C read error");

    write!(log_file, "Initial state of configuration register: {:08b} {:08b}", reg[0], reg[1])
        .expect("Error writing to log file.");

    // Write to config register
    i2c.block_write(0b10010000 as u8, &[])
        .expect("I2C write error");
    i2c.block_write(0b00000001u8, &[0b0100_0010, 0b1110_0011])
        .expect("I2C write error");

    // Read from config register
    let mut reg = [0u8; 2];
    i2c.block_write(0b10010001 as u8, &[])
        .expect("I2C write error");
    i2c.block_read(0b00000001u8, &mut reg)
        .expect("I2C read error");

    write!(log_file, "Final state of configuration register: {:08b} {:08b}", reg[0], reg[1])
        .expect("Error writing to log file.");

    ()
}

/// Function which reads ADC data from the ADS1115
fn read_adc_voltage(i2c: &mut I2c) -> f64 {
    i2c.set_slave_address(crate::ADDR_ADS1115)
        .expect("I2C Error: Unable to change address.");
    
    // Read from conversion register
    let mut reg = [0u8; 2];
    i2c.block_write(0b10010001 as u8, &[])
        .expect("I2C write error");
    i2c.block_read(0b00000000u8, &mut reg)
        .expect("I2C read error");

    let raw_data: i16 = (reg[0] as i16) << 8 | reg[1] as i16;

    raw_data as f64 * 4.096 / i32::pow(2, 15) as f64
}