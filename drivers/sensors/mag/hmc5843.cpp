#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Compass HMC5843
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************
# define MAG_ADDRESS 0x1E
# define MAG_DATA_REGISTER 0x03

void getADC()
{
    i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
    MAG_ORIENTATION( ( (rawADC[0] << 8) | rawADC[1] ),
                     ( (rawADC[2] << 8) | rawADC[3] ),
                     ( (rawADC[4] << 8) | rawADC[5] ) );
}

void Mag_init()
{
    delay(100);
    // force positiveBias
    i2c_writeReg(MAG_ADDRESS, 0x00, 0x71);  //Configuration Register A  -- 0 11 100 01  num samples: 8 ; output rate: 15Hz ; positive bias
    delay(50);
    // set gains for calibration
    i2c_writeReg(MAG_ADDRESS, 0x01, 0x60);  //Configuration Register B  -- 011 00000    configuration gain 2.5Ga
    i2c_writeReg(MAG_ADDRESS, 0x02, 0x01);  //Mode register             -- 000000 01    single Conversion Mode
    // read values from the compass -  self test operation
    // by placing the mode register into single-measurement mode (0x01), two data acquisition cycles will be made on each magnetic vector.
    // The first acquisition values will be subtracted from the second acquisition, and the net measurement will be placed into the data output registers
    delay(100);
    getADC();
    delay(10);
    magGain[ROLL] = 1000.0 / abs(imu.magADC[ROLL]);
    magGain[PITCH] = 1000.0 / abs(imu.magADC[PITCH]);
    magGain[YAW] = 1000.0 / abs(imu.magADC[YAW]);
    // leave test mode
    i2c_writeReg(MAG_ADDRESS, 0x00, 0x70);  //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS, 0x01, 0x20);  //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS, 0x02, 0x00);  //Mode register             -- 000000 00    continuous Conversion Mode
}

# if !defined(MPU6050_I2C_AUX_MASTER)
void Device_Mag_getADC()
{
    getADC();
}

# endif
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
