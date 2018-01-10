#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
# if !defined(MPU6050_ADDRESS)
#  define MPU6050_ADDRESS     0x68  // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
//#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
# endif

//The MAG acquisition function must be replaced because we now talk to the MPU device
# if defined(MPU6050_I2C_AUX_MASTER)
static void Device_Mag_getADC()
{
    i2c_getSixRawADC(MPU6050_ADDRESS, 0x49);    //0x49 is the first memory room for EXT_SENS_DATA
#  if defined(HMC5843)
    MAG_ORIENTATION( ( (rawADC[0] << 8) | rawADC[1] ),
                     ( (rawADC[2] << 8) | rawADC[3] ),
                     ( (rawADC[4] << 8) | rawADC[5] ) );
#  endif
#  if defined(HMC5883)
    MAG_ORIENTATION( ( (rawADC[0] << 8) | rawADC[1] ),
                     ( (rawADC[4] << 8) | rawADC[5] ),
                     ( (rawADC[2] << 8) | rawADC[3] ) );
#  endif
#  if defined(MAG3110)
    MAG_ORIENTATION( ( (rawADC[0] << 8) | rawADC[1] ),
                     ( (rawADC[2] << 8) | rawADC[3] ),
                     ( (rawADC[4] << 8) | rawADC[5] ) );
#  endif
}

# endif
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
