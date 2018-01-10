#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Accelerometer MMA7455
// ************************************************************************************************************
# if !defined(MMA7455_ADDRESS)
#  define MMA7455_ADDRESS 0x1D
# endif

void ACC_init()
{
    delay(10);
    i2c_writeReg(MMA7455_ADDRESS, 0x16, 0x21);
}

void ACC_getADC()
{
    i2c_getSixRawADC(MMA7455_ADDRESS, 0x00);
    ACC_ORIENTATION( ( (int8_t (rawADC[1]) << 8) | int8_t (rawADC[0]) ),
                     ( (int8_t (rawADC[3]) << 8) | int8_t (rawADC[2]) ),
                     ( (int8_t (rawADC[5]) << 8) | int8_t (rawADC[4]) ) );
    ACC_Common();
}

#else // IMPLEMENTATION
# define ACC_1G 64
#endif // IMPLEMENTATION
