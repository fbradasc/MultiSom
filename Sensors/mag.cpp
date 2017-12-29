static float magGain[3] = { 1.0, 1.0, 1.0 };	// gain for each axis, populated at sensor init

    uint8_t
Mag_getADC ()
{				// return 1 when news values are available, 0 otherwise
    static uint32_t t, tCal = 0;
    static int16_t magZeroTempMin[3], magZeroTempMax[3];
    uint8_t axis;

    if (currentTime < t)
        return 0;			//each read is spaced by 100ms
    t = currentTime + 100000;
    Device_Mag_getADC ();

    for (axis = 0; axis < 3; axis++)
    {
        imu.magADC[axis] = imu.magADC[axis] * magGain[axis];
        if (!f.CALIBRATE_MAG)
            imu.magADC[axis] -= global_conf.magZero[axis];
    }

    if (f.CALIBRATE_MAG)
    {
        if (tCal == 0)		// init mag calibration
            tCal = t;
        if ((t - tCal) < 30000000)
        {			// 30s: you have 30s to turn the multi in all directions
            LEDPIN_TOGGLE;
            for (axis = 0; axis < 3; axis++)
            {
                if (tCal == t)
                {		// it happens only in the first step, initialize the zero
                    magZeroTempMin[axis] = imu.magADC[axis];
                    magZeroTempMax[axis] = imu.magADC[axis];
                }
                if (imu.magADC[axis] < magZeroTempMin[axis])
                {
                    magZeroTempMin[axis] = imu.magADC[axis];
                    SET_ALARM (ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);
                }
                if (imu.magADC[axis] > magZeroTempMax[axis])
                {
                    magZeroTempMax[axis] = imu.magADC[axis];
                    SET_ALARM (ALRM_FAC_TOGGLE, ALRM_LVL_TOGGLE_1);
                }
                global_conf.magZero[axis] =
                    (magZeroTempMin[axis] + magZeroTempMax[axis]) >> 1;
            }
        }
        else
        {
            f.CALIBRATE_MAG = 0;
            tCal = 0;
            writeGlobalSet (1);
        }
    }

#if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.magADC[PITCH] - imu.magADC[ROLL]) * 7) / 10;
    imu.magADC[ROLL] = ((imu.magADC[ROLL] + imu.magADC[PITCH]) * 7) / 10;
    imu.magADC[PITCH] = temp;
#endif
#if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.magADC[PITCH] + imu.magADC[ROLL]) * 7) / 10;
    imu.magADC[ROLL] = ((imu.magADC[ROLL] - imu.magADC[PITCH]) * 7) / 10;
    imu.magADC[PITCH] = temp;
#endif

    return 1;
}

#if defined(MAG3110)
#include "mag/mag3110.cpp"
#endif
#if defined(HMC5883)
#include "mag/hmc5883.cpp"
#endif
#if defined(HMC5843)
#include "mag/hmc5843.cpp"
#endif
#if defined(AK8975)
#include "mag/ak8975.cpp"
#endif
#if defined(MPU6050)
#include "mag/mpu6050.cpp"
#endif
#if defined(MPU9250)
#include "mag/mpu9250.cpp"
#endif
