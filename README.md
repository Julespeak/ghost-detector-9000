# Ghost Detector 9000
The Ghost Detector 9000 is divided into two primary programs:
- The G.P.U. (Ghost Processing Unit): Runs an AHRS algorithm with a IMU, reads from and controls hardware peripherals, responds to commands over a socket connection.
- The Ghost Detection script: Reads the states of the buttons, controls the lights, communicates with the G.P.U. using socket commands.

## Building the G.P.U. ##
If you've already got a working Rust compiler install with the cross module, skip to the end of this section.  Otherwise, you will first need to get a Rust environment set up:
```
curl https://sh.rustup.rs -sSf | sh
```
Log out and log back in again.  Then install Docker:
```
sudo apt update
sudo apt upgrade
sudo apt install docker.io
sudo systemctl enable docker --now
docker
sudo usermod -aG docker $USER
```
Log out and lock back in again.  Then, you will need to install [cross](https://github.com/cross-rs/cross):
```
cargo install cross --git https://github.com/cross-rs/cross
```
Log out and log back in again one last time.  Finally, clone this repo, use cross to build the binary, and then copy it to your RPi:
```
git clone https://github.com/Julespeak/ghost-detector-9000.git
cd ghost-detector-9000
cross build --target arm-unknown-linux-gnueabihf
scp ./target/arm-unknown-linux-gnueabihf/debug/rust_gpu <raspberry pi user>@<raspberry pi ip address>:bin/rust_gpu
```
