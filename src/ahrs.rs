// Addresses of the chips on the board that I have; determined with i2cdetect
const ADDR_MPU9265: u16 = 0x68;
const ADDR_AK8963:  u16 = 0x0C;

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

// Set initial input parameters
#[repr(u8)]
#[derive(Copy, Clone)]
enum Ascale {
    Afs2g = 0,
    Afs4g,
    Afs8g,
    Afs16g,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Gscale {
    Gfs250dps = 0,
    Gfs500dps,
    Gfs1000dps,
    Gfs2000dps,
}

#[repr(u8)]
#[derive(Copy, Clone)]
enum Mscale {
    Mfs14bits = 0, // 0.6 mG per LSB
    Mfs16bits, // 0.15 mG per LSB
}

// GPIO pin connected to INT for creating hardware interrupt to read new data
const MPU_INT: u8 = 5;

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
const gyro_meas_error: f64 = std::f64::consts::PI * (40.0 / 180.0); // gryoscope measurement error in rad/s (start at 40 deg/s)
const gyro_meas_drift: f64 = std::f64::consts::PI * (0.0 / 180.0); // gryoscope measurement drift in rad/s/s (start at 0.0 deg/s/s)


pub mod ahrs {
    pub fn madgwick_quaternion_update() {}
}
