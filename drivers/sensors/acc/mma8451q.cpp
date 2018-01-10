#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Accelerometer MMA8451Q
// ************************************************************************************************************

# if !defined(MMA8451Q_ADDRESS)
#  define MMA8451Q_ADDRESS 0x1C
//#define MMA8451Q_ADDRESS 0x1D
# endif

void ACC_init()
{
    delay(10);
    i2c_writeReg(MMA8451Q_ADDRESS, 0x2A, 0x05);     // wake up & low noise
    delay(10);
    i2c_writeReg(MMA8451Q_ADDRESS, 0x0E, 0x02);     // full scale range
}

void ACC_getADC()
{
    i2c_getSixRawADC(MMA8451Q_ADDRESS, 0x00);
    ACC_ORIENTATION( ( (rawADC[1] << 8) | rawADC[0] ) / 32,
                     ( (rawADC[3] << 8) | rawADC[2] ) / 32,
                     ( (rawADC[5] << 8) | rawADC[4] ) / 32 );
    ACC_Common();
}

#else // IMPLEMENTATION
# define ACC_1G 512
#endif // IMPLEMENTATION
