# Ghost Detector 9000
This repository serves as a companion to a hardware build with a Raspberry Pi that I have documented in an [Instructable](https://www.instructables.com/The-Ghost-Detector-9000/).  With the Ghost Detector 9000 hardware, this software can be used to play a ghost-detecting game that allows the user to scan objects for ghosts and then instructs the user to point the detector in the direction of a ghost signal.  See the Instructable for more details about the project.  There is also a [video](https://www.youtube.com/watch?v=y_uXFXTJDN4).

The software for the Ghost Detector 9000 is divided into two primary programs:
- The G.P.U. (Ghost Processing Unit): Runs an AHRS algorithm with a IMU, reads from and controls hardware peripherals, responds to commands over a socket connection.
- The Ghost Detection script: Reads the states of the buttons, controls the lights, communicates with the G.P.U. using socket commands.

## Using the G.P.U. ##
Clone this repository onto the Raspberry Pi Zero.  Copy the ```rust_gpu``` program from the latest release on the Raspberry Pi, too.

Currently, the G.P.U. will attempt to create logfiles in ```/home/ghost/rust-stuff/gpu_logs```.  You can either create a new 'ghost' user on the Raspberry Pi to enable this or change this directory in the Rust code and recompile.

To start the G.P.U., do the following:
```
cd <directory_with_rust_program>
export GPU_ADDRESS=127.0.0.1
./rust_gpu
```
Some additional dependencies may be required.  You'll definitely need to enable I2C communication and set up permissions for this.  I have more documentation on how I set things up originally which I will add soon.

When the G.P.U. is running, you can interact with it using Python scripts.  See the scripts/ folder for examples.  The main application is ghost_detection.py.  Start this as:
```
sudo su
export GPU_ADDRESS=127.0.0.1
python scripts/ghost_detection.py
```
All scripts that use the Neopixel library must be run as root AFAICT.

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
scp ./target/arm-unknown-linux-gnueabihf/debug/rust_gpu <raspberry pi user>@<raspberry pi ip address>:rust_gpu
```

## Code Credits ##
- Kris Winer's [MPU9250 repository](https://github.com/kriswiner/MPU9250/tree/master) was used heavily in the AHRS.  This whole thing essentially started as a Rust port of his MPU9250_MS5637_AHRS_t3.ino sketch.
- The quaternion visualizer was copied from this repository: [PyTeapot-Quaternion-Euler-cube-rotation](https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation)

## Sound Effects ##
- ping_short.wav
	- [Button | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/button-124476/)
- ghost_capture.wav
	- [BLASTER 2 | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/blaster-2-81267/)
- ghost_in_area_2.wav
	- [Severe Warning Alarm | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/severe-warning-alarm-98704/)
	- [warning - evacuation in progress - leave the building - Australian female voice | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/warning-evacuation-in-progress-leave-the-building-australian-female-voice-12542/)
- five.wav, four.wav, three.wav, two.wav, one.wav, zero.wav
	- [Robotic Countdown: 10 to 0 | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/robotic-countdown-10-to-0-96511/)
- bad_signal.wav
	- [112_tjerk | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/112-tjerk-79083/)
- capture_success.wav
	- [Rising Funny Game Effect | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/rising-funny-game-effect-132474/)
- scan_pass.wav
	- [Simple Notification | Royalty-free Music - Pixabay](https://pixabay.com/sound-effects/simple-notification-152054/)
