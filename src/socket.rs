use std::{
    fs::File,
    io::{prelude::*, Write},
    net::TcpListener,
    sync::mpsc::{self, Sender, Receiver},
    thread,
    time::Instant,
};

pub struct SocketHost {
    // Sender sends data to the SocketHost
    pub sender: Option<mpsc::Sender<crate::Message>>,
    // Receiver gets messages from the SocketHost
    pub receiver: Option<mpsc::Receiver<crate::Message>>,
    // Thread reference; TODO - use this for doing elegant shutdowns
    thread: Option<thread::JoinHandle<()>>,
}

impl SocketHost {
    /// Create a new SocketHost interface.  The SocketHost spawns a thread which waits
    ///  to receive a socket connection.  When a socket connection exists, the SocketHost
    ///  will wait to receive commands, which it will then forward to the GPU.  GPU
    ///  response data is written back to the socket connection.
    ///
    /// # More Documentation
    ///
    /// Would go here eventually...
    pub fn new(time_string: &str) -> SocketHost {
        // Set up connection to log file
        let log_file_name = format!("/home/ghost/rust-stuff/gpu_logs/{}_sockethost_logfile.txt", time_string);
        let mut log_file = File::create(log_file_name)
            .expect("Socket: Unable to create logfile.");

        // Both the main and the satellite get a sender and receiver
        let (mosi_sender, mosi_receiver) = mpsc::channel();
        let (miso_sender, miso_receiver) = mpsc::channel();

        let worker_thread = thread::spawn(move ||
            socket_worker(
                &mut log_file,
                mosi_receiver,
                miso_sender
            )
        );

        SocketHost {
            sender: Some(mosi_sender),
            receiver: Some(miso_receiver),
            thread: Some(worker_thread),
        }
    }
}

/// Wait for a socket connection; when one exists, wait for messages and forward those to the GPU
pub fn socket_worker(
    log_file: &mut File,
    receiver: Receiver<crate::Message>,
    sender: Sender<crate::Message>
    ) {
    let start = Instant::now();

    let listener = TcpListener::bind("ip_address:9005").unwrap();

    for stream in listener.incoming() {
        let mut stream = stream.unwrap();

        loop {
            // First two bytes contain the address and number of bytes in the request
            let mut raw_input: [u8; 2] = [8; 2];
            let _num_bytes = match stream.read(&mut raw_input) {
                Ok(nb) => nb,
                Err(_e) => {
                    // Break out of the loop if we fail to read from the socket
                    println!("Socket: Error reading from socket.");
                    break
                }
            };

            let input_address: u8 = raw_input[0];
            let request_size: usize = raw_input[1].into();

            let mut input_request: Vec<u8> = Vec::new();
            if request_size > 0 {
                let mut raw_request: Vec<u8> = vec![0; request_size];
                let _num_bytes = stream.read(&mut raw_request)
                    .expect("Socket: Error reading from socket.");
                input_request = raw_request;
            }

            let message = crate::Message {
                address: input_address,
                request: input_request,
                response: Vec::new(),
            };

            sender.send(message)
                .expect("Socket: Could not send data back from worker.");

            let return_message = receiver.recv()
                .expect("Socket: No response from worker");

            match stream.write_all(&return_message.response) {
                Ok(nb) => nb,
                Err(_e) => {
                    // Break out of the loop if we fail to write to the socket
                    println!("Socket: Error reading from socket.");
                    break
                }
            }
        }
    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed)
        .expect("Socket: Error writing to log file.");
}
