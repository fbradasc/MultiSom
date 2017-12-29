// ************************************************************************************************************
// I2C Gyroscope and Accelerometer LSM330
// ************************************************************************************************************
#if !defined(LSM330_ACC_ADDRESS)
#define LSM330_ACC_ADDRESS     0x18	// 30 >> 1 = 18  -> address pin SDO_A low (GND)
//#define LSM330_ACC_ADDRESS     0x19 // 32 >> 1 = 19  -> address pin SDO_A high (VCC)
#endif
    void
ACC_init ()
{

    delay (10);
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x17 ); // 1Hz
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x27 ); // 10Hz
    i2c_writeReg (LSM330_ACC_ADDRESS, 0x20, 0x37);	// 25Hz
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x47 ); // 50Hz
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x20 ,0x57 ); // 100Hz

    delay (5);
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x08 ); // 2G
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x18 ); // 4G
    i2c_writeReg (LSM330_ACC_ADDRESS, 0x23, 0x28);	// 8G
    //i2c_writeReg(LSM330_ACC_ADDRESS ,0x23 ,0x38 ); // 16G

    delay (5);
    i2c_writeReg (LSM330_ACC_ADDRESS, 0x21, 0x00);	// no high-pass filter
}

//#define ACC_DELIMITER 5 // for 2g
#define ACC_DELIMITER 4		// for 4g
//#define ACC_DELIMITER 3 // for 8g
//#define ACC_DELIMITER 2 // for 16g

    void
ACC_getADC ()
{
    i2c_getSixRawADC (LSM330_ACC_ADDRESS, 0x80 | 0x28);	// Start multiple read at reg 0x28

    ACC_ORIENTATION (((rawADC[1] << 8) | rawADC[0]) >> ACC_DELIMITER,
                     ((rawADC[3] << 8) | rawADC[2]) >> ACC_DELIMITER,
                     ((rawADC[5] << 8) | rawADC[4]) >> ACC_DELIMITER);
    ACC_Common ();
}
