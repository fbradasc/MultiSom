#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Compass MAG3110
// ************************************************************************************************************
// I2C adress: 0x0E (7bit)
// ************************************************************************************************************
# define MAG_ADDRESS 0x0E
# define MAG_DATA_REGISTER 0x01
# define MAG_CTRL_REG1 0x10
# define MAG_CTRL_REG2 0x11

void Mag_init()
{
    delay(100);
    i2c_writeReg(MAG_ADDRESS, MAG_CTRL_REG2, 0x80);     //Automatic Magnetic Sensor Reset
    delay(100);
    i2c_writeReg(MAG_ADDRESS, MAG_CTRL_REG1, 0x11);     // DR = 20Hz ; OS ratio = 64 ; mode = Active
    delay(100);
}

# if !defined(MPU6050_I2C_AUX_MASTER || MPU9250)
void Device_Mag_getADC()
{
    i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
    MAG_ORIENTATION( ( (rawADC[0] << 8) | rawADC[1] ),
                     ( (rawADC[2] << 8) | rawADC[3] ),
                     ( (rawADC[4] << 8) | rawADC[5] ) );
}

# endif
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
