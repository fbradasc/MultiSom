// ************************************************************************************************************
// I2C Gyroscope ITG3200 / ITG3205 / ITG3050 / MPU3050
// ************************************************************************************************************
// I2C adress: 0xD2 (8bit)   0x69 (7bit)
// I2C adress: 0xD0 (8bit)   0x68 (7bit)
// principle:
// 1) VIO is connected to VDD
// 2) I2C adress is set to 0x69 (AD0 PIN connected to VDD)
// or 2) I2C adress is set to 0x68 (AD0 PIN connected to GND)
// 3) sample rate = 1000Hz ( 1kHz/(div+1) )
// ************************************************************************************************************
#if !defined(GYRO_ADDRESS)
#define GYRO_ADDRESS 0X68
//#define GYRO_ADDRESS 0X69
#endif

    void
Gyro_init ()
{
    i2c_writeReg (GYRO_ADDRESS, 0x3E, 0x80);	//PWR_MGMT_1    -- DEVICE_RESET 1
    delay (5);
    i2c_writeReg (GYRO_ADDRESS, 0x16, 0x18 + GYRO_DLPF_CFG);	//Gyro CONFIG   -- EXT_SYNC_SET 0 (disable input pin for data sync) ; DLPF_CFG = GYRO_DLPF_CFG ; -- FS_SEL = 3: Full scale set to 2000 deg/sec
    delay (5);
    i2c_writeReg (GYRO_ADDRESS, 0x3E, 0x03);	//PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    delay (100);
}

    void
Gyro_getADC ()
{
    i2c_getSixRawADC (GYRO_ADDRESS, 0X1D);
    GYRO_ORIENTATION (((rawADC[0] << 8) | rawADC[1]) >> 2,	// range: +/- 8192; +/- 2000 deg/sec
                      ((rawADC[2] << 8) | rawADC[3]) >> 2,
                      ((rawADC[4] << 8) | rawADC[5]) >> 2);
    GYRO_Common ();
}
