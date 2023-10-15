use std::{
    error::Error,
    fs::File,
    io::{prelude::*, Write, BufReader},
    net::TcpListener,
    sync::mpsc::{self, Sender, Receiver},
    thread,
    time::Instant,
};

// Message structure for socket communication
pub struct Message {
    pub address: u8,
    pub request: Vec<u8>,
    pub response: Vec<u8>,
}

pub struct SocketHost {
    // Sender sends data to the SocketHost
    pub sender: Option<mpsc::Sender<Message>>,
    // Receiver gets messages from the SocketHost
    pub receiver: Option<mpsc::Receiver<Message>>,
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
    pub fn new(time_string: &str) -> Result<SocketHost, Box<dyn Error>> {
        // Set up connection to log file
        let log_file_name = format!("/home/ghost/rust-stuff/gpu_logs/{}_sockethost_logfile.txt", time_string);
        let mut log_file = File::create(log_file_name)
            .expect("Unable to create SocketHost logfile.");

        // Both the main and the satellite get a sender and receiver
        let (mosi_sender, mosi_receiver) = mpsc::channel();
        let (miso_sender, miso_receiver) = mpsc::channel();

        let sockethost_thread = thread::spawn(move || receive_connections(&mut log_file, mosi_receiver, miso_sender));

        Ok(SocketHost {
            sender: Some(mosi_sender),
            receiver: Some(miso_receiver),
            thread: Some(sockethost_thread),
        })
    }
}

/// Wait for a socket connection; when one exists, wait for messages and forward those to the GPU
pub fn receive_connections(log_file: &mut File, receiver: Receiver<Message>, sender: Sender<Message>) {
    let start = Instant::now();

    let listener = TcpListener::bind("ip_address:9005").unwrap();

    for stream in listener.incoming() {
        let mut stream = stream.unwrap();

        loop { // Should be able to get out of this loop if the connection dies
            // First two bytes contain the address and number of bytes in the request
            let mut raw_input: [u8; 2] = [8; 2];
            let _num_bytes = stream.read(&mut raw_input)
                .expect("SocketHost: Error reading from socket.");

            let input_address: u8 = raw_input[0];
            let request_size: usize = raw_input[1].into();

            let mut input_request: Vec<u8> = Vec::new();
            if request_size > 0 {
                let mut raw_request: Vec<u8> = vec![0; request_size];
                let _num_bytes = stream.read(&mut raw_request)
                    .expect("SocketHost: Error reading from socket.");
                input_request = raw_request;
            }

            let message = Message {
                address: input_address,
                request: input_request,
                response: Vec::new(),
            };

            sender.send(message)
                .expect("Could not send data back from SocketHost thread.");

            let return_message = receiver.recv()
                .expect("No response from GPU.");

            stream.write_all(&return_message.response)
                .expect("Error writing to socket.");
        }
    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed)
        .expect("Error writing to log file.");
}
