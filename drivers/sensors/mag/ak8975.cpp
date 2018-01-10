#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Compass AK8975
// ************************************************************************************************************
// I2C adress: 0x0C (7bit)
// ************************************************************************************************************
# define MAG_ADDRESS 0x0C
# define MAG_DATA_REGISTER 0x03

void Mag_init()
{
    delay(100);
    i2c_writeReg(MAG_ADDRESS, 0x0a, 0x01);  //Start the first conversion
    delay(100);
}

void Device_Mag_getADC()
{
    i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
    MAG_ORIENTATION( ( (rawADC[1] << 8) | rawADC[0] ),
                     ( (rawADC[3] << 8) | rawADC[2] ),
                     ( (rawADC[5] << 8) | rawADC[4] ) );
    //Start another meassurement
    i2c_writeReg(MAG_ADDRESS, 0x0a, 0x01);
}

#else // IMPLEMENTATION
#endif // IMPLEMENTATION
