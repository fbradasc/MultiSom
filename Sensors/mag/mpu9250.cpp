// ************************************************************************************************************
// I2C Gyroscope/Accelerometer/Compass MPU9250
// ************************************************************************************************************
#if !defined(MPU9250_MAG_ADDRESS)
#define MPU9250_MAG_ADDRESS 0x0C
#define MAG_DATA_REGISTER 0x03
#endif
    void
Mag_init ()
{
    delay (100);
    i2c_writeReg (MPU9250_MAG_ADDRESS, 0x0a, 0x01);	//Start the first conversion
    delay (100);
}

    void
Device_Mag_getADC ()
{
    i2c_getSixRawADC (MPU9250_MAG_ADDRESS, 0x03);
    MAG_ORIENTATION (((rawADC[1] << 8) | rawADC[0]),
                     ((rawADC[3] << 8) | rawADC[2]),
                     ((rawADC[5] << 8) | rawADC[4]));
    //Start another meassurement
    i2c_writeReg (MPU9250_MAG_ADDRESS, 0x0a, 0x01);
}
