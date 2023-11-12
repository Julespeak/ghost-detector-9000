use std::{
    fs::File,
    io::Write,
    sync::{mpsc::{self, Sender, Receiver}, Arc, Mutex},
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
    /// Create a new EmfHost interface.  The EmfHost spawns a thread which
    ///  waits to receive data requests.  When a request is received, an ADC
    ///  signal is read at a fixed sample rate.  The resulting waveform is sent
    ///  back to the host process.
    ///
    /// # More Documentation
    ///
    /// Would go here eventually...
    pub fn new(time_string: &str, verbose: bool, i2c: Arc<Mutex<I2c>>) -> EmfHost {
        // Set up connection to log file
        let log_file_name = format!("/home/ghost/rust-stuff/gpu_logs/{}_emf_logfile.txt", time_string); 
        let mut log_file = File::create(log_file_name)
            .expect("Emf: Unable to create logfile.");

        {
            // Aquire mutex for access to the I2C hardware
            let mut i2c = i2c.lock().unwrap();
            // Set the address of the ADS1115
            i2c.set_slave_address(crate::ADDR_ADS1115)
                .expect("Emf: Unable to set I2C address.");

            // Configure the ADC mdoule
            configure_ads1115(&i2c, &mut log_file);
        }

        // Both the main and the satellite get a sender and receiver
        let (mosi_sender, mosi_receiver) = mpsc::channel();
        let (miso_sender, miso_receiver) = mpsc::channel();

        let worker_thread = thread::spawn(move ||
            emf_worker(
                i2c,
                &mut log_file,
                verbose,
                mosi_receiver,
                miso_sender
            )
        );

        EmfHost {
            sender: Some(mosi_sender),
            receiver: Some(miso_receiver),
            thread: Some(worker_thread),
        }
    }

    pub fn send_message(&self, address: u8, request: Vec<u8>) -> Vec<u8> {
        // Prepare request message for EMF thread
        let message = crate::Message {
            address: address,
            request: request,
            response: Vec::new(),
        };
        // Send message to EMF thread
        // TODO - understand why these as_ref's are necessary
        self.sender.as_ref().unwrap()
            .send(message).expect("Emf: Could not send data message to worker.");
        // Get return message from EMF thread
        let return_message = self.receiver.as_ref().unwrap()
            .recv().expect("Emf: No response from worker.");
        // Return response from EMF thread
        return_message.response
    }
}

impl Drop for EmfHost {
    fn drop(&mut self) {
        println!("EmfHost is going down!");

        drop(self.sender.take());
        drop(self.receiver.take());

        if let Some(thread) = self.thread.take() {
            thread.join().unwrap();
        }
    }
}

pub fn emf_worker(
    i2c: Arc<Mutex<I2c>>,
    log_file: &mut File,
    verbose: bool,
    receiver: Receiver<crate::Message>,
    sender: Sender<crate::Message>
    ) {
    let start = Instant::now();
    // let mut last_update_time: u64 = 0;

    // Variables for monitoring the acquisition rate
    let mut iterations: u32 = 0;
    let mut adc_reads: u32 = 0;

    loop {
        match receiver.try_recv() {
            Ok(mut message) => {
                let message_response = match message.address {
                    0x00 => { // Read a single value and return
                        let mut i2c = i2c.lock().unwrap();
                        let voltage = read_adc_voltage(&mut i2c);
                        adc_reads = adc_reads + 1;
                        if verbose {
                            let current_update_time: u64 = start.elapsed().as_nanos() as u64;
                            write!(log_file, "DATA: current_time, 1, 1\n")
                                .expect("Emf: Error writing to log file.");
                            write!(log_file, "{}\n", current_update_time)
                                .expect("Emf: Error writing to log file.");
                            write!(log_file, "DATA: voltage, 1, 1\n")
                                .expect("Emf: Error writing to log file.");
                            write!(log_file, "{}\n", voltage)
                                .expect("Emf: Error writing to log file.");
                        }
                        voltage.to_be_bytes().into_iter().collect()
                    },
                    0x01 => { // Keep reading values until a stop message is received; return all recorded data
                        // println!("Beginning ADC acquisition");
                        let begin_message = crate::Message {
                            address: 0x00,
                            request: Vec::new(),
                            response: vec![0x00, 0x01],
                        };
                        sender.send(begin_message)
                            .expect("Emf: Could not send data back from worker.");
                        // Loop while reading data; stop when a stop message is received
                        let mut output_data: Vec<f64> = Vec::new();
                        loop {
                            if let Ok(stop_message) = receiver.try_recv() {
                                if stop_message.address == 0x02 { break; }
                            }
                            {
                                let mut i2c = i2c.lock().unwrap();
                                let voltage = read_adc_voltage(&mut i2c);
                                adc_reads = adc_reads + 1;
                                if verbose {
                                    let current_update_time: u64 = start.elapsed().as_nanos() as u64;
                                    write!(log_file, "DATA: current_time, 1, 1\n")
                                        .expect("Emf: Error writing to log file.");
                                    write!(log_file, "{}\n", current_update_time)
                                        .expect("Emf: Error writing to log file.");
                                    write!(log_file, "DATA: voltage, 1, 1\n")
                                        .expect("Emf: Error writing to log file.");
                                    write!(log_file, "{}\n", voltage)
                                        .expect("Emf: Error writing to log file.");
                                }
                                output_data.push(voltage);
                            }
                        }
                        // Return acquired data
                        // println!("Returning measured data");
                        encode_voltage_vector(output_data)
                    }
                    _ => "UNKNOWN ADDRESS".as_bytes().to_vec(),
                };

                message.response = message_response;

                sender.send(message)
                    .expect("Emf: Could not send data from worker.");
            },
            Err(mpsc::TryRecvError::Empty) => (),
            Err(mpsc::TryRecvError::Disconnected) => {
                break
            },
        }

        thread::sleep(Duration::from_millis(50));

        iterations = iterations + 1;
    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed)
        .expect("Error writing to log file.");
    write!(log_file, "DEBUG: Loop iterations: {}\n", iterations)
        .expect("Error writing to log file.");
    write!(log_file, "DEBUG: ADC reads: {}\n", adc_reads)
        .expect("Error writing to log file.");
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

fn encode_voltage_vector(voltages: Vec<f64>) -> Vec<u8> {
    let mut encoded_voltages = Vec::new();
    for voltage in voltages {
        // https://stackoverflow.com/questions/54142528/how-can-i-concatenate-two-slices-or-two-vectors-and-still-have-access-to-the-ori
        encoded_voltages = encoded_voltages.to_vec().into_iter().chain(voltage.to_be_bytes()).collect();
    }
    encoded_voltages
}

/// Function which reads ADC data from the ADS1115
fn read_adc_voltage(i2c: &mut I2c) -> f64 {
    i2c.set_slave_address(crate::ADDR_ADS1115)
        .expect("Unable to set I2C address.");
    
    // Read from conversion register
    let mut reg = [0u8; 2];
    i2c.block_write(0b10010001 as u8, &[])
        .expect("I2C write error");
    i2c.block_read(0b00000000u8, &mut reg)
        .expect("I2C read error");

    let raw_data: i16 = (reg[0] as i16) << 8 | reg[1] as i16;

    raw_data as f64 * 4.096 / i32::pow(2, 15) as f64
}