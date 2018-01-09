#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Compass HMC5883
// ************************************************************************************************************
// I2C adress: 0x3C (8bit)   0x1E (7bit)
// ************************************************************************************************************

#define HMC58X3_R_CONFA 0
#define HMC58X3_R_CONFB 1
#define HMC58X3_R_MODE 2
#define HMC58X3_X_SELF_TEST_GAUSS (+1.16)   //!< X axis level when bias current is applied.
#define HMC58X3_Y_SELF_TEST_GAUSS (+1.16)   //!< Y axis level when bias current is applied.
#define HMC58X3_Z_SELF_TEST_GAUSS (+1.08)   //!< Y axis level when bias current is applied.
#define SELF_TEST_LOW_LIMIT  (243.0/390.0)  //!< Low limit when gain is 5.
#define SELF_TEST_HIGH_LIMIT (575.0/390.0)  //!< High limit when gain is 5.
#define HMC_POS_BIAS 1
#define HMC_NEG_BIAS 2

#define MAG_ADDRESS 0x1E
#define MAG_DATA_REGISTER 0x03

static int32_t xyz_total[3] = { 0, 0, 0 };  // 32 bit totals so they won't overflow.

static void
getADC()
{
    i2c_getSixRawADC(MAG_ADDRESS, MAG_DATA_REGISTER);
    MAG_ORIENTATION(((rawADC[0] << 8) | rawADC[1]),
                    ((rawADC[4] << 8) | rawADC[5]),
                    ((rawADC[2] << 8) | rawADC[3]));
}

static uint8_t
bias_collect(uint8_t bias)
{
    int16_t abs_magADC;
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, bias);   // Reg A DOR=0x010 + MS1,MS0 set to pos or negative bias

    for (uint8_t i = 0; i < 10; i++)
    {
        // Collect 10 samples
        i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
        delay(100);
        getADC();       // Get the raw values in case the scales have already been changed.

        for (uint8_t axis = 0; axis < 3; axis++)
        {
            abs_magADC = abs(imu.magADC[axis]);
            xyz_total[axis] += abs_magADC;  // Since the measurements are noisy, they should be averaged rather than taking the max.

            if ((int16_t)(1 << 12) < abs_magADC)
            {
                return false;    // Detect saturation.   if false Breaks out of the for loop.  No sense in continuing if we saturated.
            }
        }
    }

    return true;
}

static void
Mag_init()
{
    bool bret = true;       // Error indicator
    // Note that the  very first measurement after a gain change maintains the same gain as the previous setting.
    // The new gain setting is effective from the second measurement and on.
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 2 << 5);     //Set the Gain
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 1);
    delay(100);
    getADC();           //Get one sample, and discard it

    if (!bias_collect(0x010 + HMC_POS_BIAS))
    {
        bret = false;
    }

    if (!bias_collect(0x010 + HMC_NEG_BIAS))
    {
        bret = false;
    }

    if (bret)           // only if no saturation detected, compute the gain. otherwise, the default 1.0 is used
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            magGain[axis] = 820.0 * HMC58X3_X_SELF_TEST_GAUSS * 2.0 * 10.0 / xyz_total[axis];    // note: xyz_total[axis] is always positive
        }

    // leave test mode
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFA, 0x70);   //Configuration Register A  -- 0 11 100 00  num samples: 8 ; output rate: 15Hz ; normal measurement mode
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_CONFB, 0x20);   //Configuration Register B  -- 001 00000    configuration gain 1.3Ga
    i2c_writeReg(MAG_ADDRESS, HMC58X3_R_MODE, 0x00);    //Mode register             -- 000000 00    continuous Conversion Mode
    delay(100);
}

#if !defined(MPU6050_I2C_AUX_MASTER) && !defined(MPU9250)
static void
Device_Mag_getADC()
{
    getADC();
}
#endif
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
