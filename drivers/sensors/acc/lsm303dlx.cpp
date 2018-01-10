#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Accelerometer LSM303DLx
// ************************************************************************************************************
void ACC_init()
{
    delay(10);
    i2c_writeReg(0x18, 0x20, 0x27);
    i2c_writeReg(0x18, 0x23, 0x30);
    i2c_writeReg(0x18, 0x21, 0x00);
}

void ACC_getADC()
{
    i2c_getSixRawADC(0x18, 0xA8);
    ACC_ORIENTATION( ( (rawADC[1] << 8) | rawADC[0] ) >> 4,
                     ( (rawADC[3] << 8) | rawADC[2] ) >> 4,
                     ( (rawADC[5] << 8) | rawADC[4] ) >> 4 );
    ACC_Common();
}

#else // IMPLEMENTATION
# define ACC_1G 255
#endif // IMPLEMENTATION
