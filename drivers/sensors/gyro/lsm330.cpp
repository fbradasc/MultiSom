#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Gyroscope and Accelerometer LSM330
// ************************************************************************************************************
#if !defined(LSM330_GYRO_ADDRESS)
#define LSM330_GYRO_ADDRESS     0x6A	// D4 >> 1 = 6A  -> address pin SDO_G low (GND)
//#define LSM330_GYRO_ADDRESS     0x6B // D6 >> 1 = 6B  -> address pin SDO_G high (VCC)
#endif

    void
Gyro_init ()
{
    delay (100);
    i2c_writeReg (LSM330_GYRO_ADDRESS, 0x20, 0x8F);	// CTRL_REG1   400Hz ODR, 20hz filter, run!
    delay (5);
    i2c_writeReg (LSM330_GYRO_ADDRESS, 0x24, 0x02);	// CTRL_REG5   low pass filter enable
    delay (5);
    i2c_writeReg (LSM330_GYRO_ADDRESS, 0x23, 0x30);	// CTRL_REG4 Select 2000dps
}

    void
Gyro_getADC ()
{
    i2c_getSixRawADC (LSM330_GYRO_ADDRESS, 0x80 | 0x28);

    GYRO_ORIENTATION (((rawADC[1] << 8) | rawADC[0]) >> 2,
                      ((rawADC[3] << 8) | rawADC[2]) >> 2,
                      ((rawADC[5] << 8) | rawADC[4]) >> 2);
    GYRO_Common ();
}
#else // IMPLEMENTATION
// GYRO SCALE: we ignore the last 2 bits and convert it for rad/s
#define GYRO_SCALE ((4.0f * PI * 70.0f)/(1000.0f * 180.0f * 1000000.0f))	// 70 milli deg/s /digit => 1 deg/s = 1000/70 LSB
#endif // IMPLEMENTATION
