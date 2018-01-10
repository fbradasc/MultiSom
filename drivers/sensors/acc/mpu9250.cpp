#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Gyroscope/Accelerometer/Compass MPU9250
// ************************************************************************************************************
# if !defined(MPU9250_ADDRESS)
#  define MPU9250_ADDRESS     0x68  // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
//#define MPU9250_ADDRESS     0x69 // address pin AD0 high (VCC)
# endif
void ACC_init()
{
    i2c_writeReg(MPU9250_ADDRESS, 0x1C, 0x10);
}

void ACC_getADC()
{
    i2c_getSixRawADC(MPU9250_ADDRESS, 0x3B);
    ACC_ORIENTATION( ( (rawADC[0] << 8) | rawADC[1] ) / 8,
                     ( (rawADC[2] << 8) | rawADC[3] ) / 8,
                     ( (rawADC[4] << 8) | rawADC[5] ) / 8 );
    ACC_Common();
}

#else // IMPLEMENTATION
# if defined(FREEIMUv04)
#  define ACC_1G 255
# else
#  define ACC_1G 512
# endif
#endif // IMPLEMENTATION
