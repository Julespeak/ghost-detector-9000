use std::{
    error::Error,
    fs::File,
    io::{
        prelude::*,
        Write,
        BufReader,
    },
    net::TcpListener,
    sync::{
        mpsc::{
            self,
            Sender,
            Receiver,
        }
    },
    thread,
    time::{
        Duration,
        Instant,
    }
};

// #[repr(u8)]
// #[derive(Copy, Clone)]
// pub enum Mscale {
//     Mfs14bits = 0, // 0.6 mG per LSB
//     Mfs16bits, // 0.15 mG per LSB
// }

// Message structure for socket communication
//  TODO - I'm using Strings to deal with ownership; can this be done more elegantly?
pub struct Message {
    pub address: String,
    pub request: Vec<String>,
    pub response: Vec<String>,
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
        // let log_file_name = format!("/home/ghost/rust-stuff/gpu_logs/{}_sockethost_logfile.txt", time_string);
        // HACK - change log file location temporarily
        let log_file_name = format!("dummy_sockethost_logfile.txt");
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

    let listener = TcpListener::bind("127.0.0.1:9005").unwrap();

    for stream in listener.incoming() {
        let mut stream = stream.unwrap();
        let mut buf_reader = BufReader::new(&mut stream);

        let mut raw_input = String::new();
        let num_bytes = buf_reader.read_line(&mut raw_input)
            .expect("Error reading from socket.");

        let input_vec: Vec<String> = raw_input.split(" ").map(|x| x.to_string()).collect();

        let input_address: String = input_vec.get(0)
            .expect("No address provided.").to_string();

        let input_request: Vec<String> = input_vec.get(1..)
            .expect("No request provided.").to_vec();

        println!("Got an address: {}", input_address);
        println!("Got a request: {:?}", input_request);

        let message = Message {
            address: input_address,
            request: input_request,
            response: Vec::new(),
        };

        sender.send(message)
            .expect("Could not send data back from SocketHost thread.");

        // after this there would be a blocking call to the receiver which would wait for a response from the GPU
    }

    let elapsed = start.elapsed();

    write!(log_file, "DEBUG: Elapsed time: {:?}\n", elapsed)
        .expect("Error writing to log file.");
}
