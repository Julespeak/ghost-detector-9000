pub mod ahrs;
pub mod socket;
pub mod emf;

// Addresses of the chips on the board that I have; determined with i2cdetect
pub const ADDR_MPU9265: u16 = 0x68;
pub const ADDR_AK8963:  u16 = 0x0C;
pub const ADDR_ADS1115: u16 = 0x48;

//////////
// GPIO //
//////////

// GPIO pin connected to LED for debug purposes
pub const GPIO_LED: u8 = 17;

// GPIO pin connected to INT for creating hardware interrupt to read new data
pub const MPU_INT: u8 = 5;

// Message structure for channel communication
pub struct Message {
    pub address: u8,
    pub request: Vec<u8>,
    pub response: Vec<u8>,
}
