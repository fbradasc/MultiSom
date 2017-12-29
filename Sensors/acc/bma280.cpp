// ************************************************************************************************************
// I2C Accelerometer BMA280
// ************************************************************************************************************
#if !defined(BMA280_ADDRESS)
#define BMA280_ADDRESS 0x18	// SDO PIN on GND
//#define BMA280_ADDRESS 0x19  // SDO PIN on Vddio
#endif

    void
ACC_init ()
{
    delay (10);
    i2c_writeReg (BMA280_ADDRESS, 0x10, 0x09);	//set BW to 15,63Hz
    delay (5);
    i2c_writeReg (BMA280_ADDRESS, 0x0F, 0x08);	//set range to 8G
}

    void
ACC_getADC ()
{
    i2c_getSixRawADC (BMA280_ADDRESS, 0x02);
    //usefull info is on the 14 bits  [2-15] bits  /4 => [0-13] bits  /4 => 12 bit resolution
    ACC_ORIENTATION (((rawADC[1] << 8) | rawADC[0]) >> 4,
                     ((rawADC[3] << 8) | rawADC[2]) >> 4,
                     ((rawADC[5] << 8) | rawADC[4]) >> 4);
    ACC_Common ();
}
