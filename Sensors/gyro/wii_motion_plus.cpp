// ************************************************************************************************************
// I2C Wii Motion Plus
// ************************************************************************************************************
// I2C adress 1: 0x53 (7bit)
// I2C adress 2: 0x52 (7bit)
// ************************************************************************************************************
#define WMP_ADDRESS_1 0x53
#define WMP_ADDRESS_2 0x52

    void
Gyro_init ()
{
    delay (250);
    i2c_writeReg (WMP_ADDRESS_1, 0xF0, 0x55);	// Initialize Extension
    delay (250);
    i2c_writeReg (WMP_ADDRESS_1, 0xFE, 0x05);	// Activate Nunchuck pass-through mode
    delay (250);
}

    void
Gyro_getADC ()
{
    uint8_t axis;
    TWBR = ((F_CPU / I2C_SPEED) - 16) / 2;	// change the I2C clock rate
    i2c_getSixRawADC (WMP_ADDRESS_2, 0x00);
    TWBR = ((F_CPU / 400000) - 16) / 2;	// change the I2C clock rate.

    if (micros () < (neutralizeTime + NEUTRALIZE_DELAY))
    {				//we neutralize data in case of blocking+hard reset state
        for (axis = 0; axis < 3; axis++)
        {
            imu.gyroADC[axis] = 0;
            imu.accADC[axis] = 0;
        }
        imu.accADC[YAW] = ACC_1G;
    }

    // Wii Motion Plus Data
    if ((rawADC[5] & 0x03) == 0x02)
    {
        // Assemble 14bit data
        imu.gyroADC[ROLL] = -(((rawADC[5] >> 2) << 8) | rawADC[2]);	//range: +/- 8192
        imu.gyroADC[PITCH] = -(((rawADC[4] >> 2) << 8) | rawADC[1]);
        imu.gyroADC[YAW] = -(((rawADC[3] >> 2) << 8) | rawADC[0]);
        GYRO_Common ();
        // Check if slow bit is set and normalize to fast mode range
        imu.gyroADC[ROLL] = (rawADC[3] & 0x01) ? imu.gyroADC[ROLL] / 5 : imu.gyroADC[ROLL];	//the ratio 1/5 is not exactly the IDG600 or ISZ650 specification
        imu.gyroADC[PITCH] = (rawADC[4] & 0x02) >> 1 ? imu.gyroADC[PITCH] / 5 : imu.gyroADC[PITCH];	//we detect here the slow of fast mode WMP gyros values (see wiibrew for more details)
        imu.gyroADC[YAW] = (rawADC[3] & 0x02) >> 1 ? imu.gyroADC[YAW] / 5 : imu.gyroADC[YAW];	// this step must be done after zero compensation
    }
}
