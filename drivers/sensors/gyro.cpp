#if defined(IMPLEMENTATION)
// ****************
// GYRO common part
// ****************
void GYRO_Common()
{
    static int16_t previousGyroADC[3] = { 0, 0, 0 };
    static int32_t g[3];
    uint8_t axis /*, tilt=0 */;

# if defined MMGYRO
    // Moving Average Gyros by Magnetron1
    //---------------------------------------------------
    static int16_t mediaMobileGyroADC[3][MMGYROVECTORLENGTH];
    static int32_t mediaMobileGyroADCSum[3];
    static uint8_t mediaMobileGyroIDX;
    //---------------------------------------------------
# endif

    if (calibratingG > 0)
    {
# if defined(FIXEDWING) && defined(SERVO_FIELD_TRIM)
        if ( (imu.accSmooth[2] < -ACC_1G / 2) && (calibratingG == 1) )
        {
            for (int i = PRI_SERVO_FROM - 1; i < PRI_SERVO_TO; i++)
            {
                conf.servoConf[i].middle =
                    constrain(servo[i], MIDRC - 100, MIDRC + 100);
            }

            writeParams(0);
        }
# endif

        for (axis = 0; axis < 3; axis++)
        {
            if (calibratingG == 512)
            {
                // Reset g[axis] at start of calibration
                g[axis] = 0;
# if defined(GYROCALIBRATIONFAILSAFE)
                previousGyroADC[axis] = imu.gyroADC[axis];
# endif
            }

# if defined(GYROCALIBRATIONFAILSAFE)
            if (calibratingG % 10 == 0)
            {
                if (abs(imu.gyroADC[axis] - previousGyroADC[axis]) > 8)
                {
                    tilt = 1;
                }

                previousGyroADC[axis] = imu.gyroADC[axis];
            }
# endif
            g[axis] += imu.gyroADC[axis];   // Sum up 512 readings
            gyroZero[axis] = g[axis] >> 9;

            if (calibratingG == 1)
            {
                SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_ELSE);
            }
        }

# if defined(GYROCALIBRATIONFAILSAFE)
        if (tilt)
        {
            calibratingG = 1000;
            LEDPIN_ON;
        }
        else
        {
            calibratingG--;
            LEDPIN_OFF;
        }

        return;
# else
        calibratingG--;
# endif
    }

# ifdef MMGYRO
    mediaMobileGyroIDX = ++mediaMobileGyroIDX % conf.mmgyro;
# endif

    for (axis = 0; axis < 3; axis++)
    {
        imu.gyroADC[axis] -= gyroZero[axis];
# ifdef MMGYRO
        mediaMobileGyroADCSum[axis] -=
            mediaMobileGyroADC[axis][mediaMobileGyroIDX];
        //anti gyro glitch, limit the variation between two consecutive readings
        mediaMobileGyroADC[axis][mediaMobileGyroIDX] =
            constrain(imu.gyroADC[axis], previousGyroADC[axis] - 800,
                      previousGyroADC[axis] + 800);
        mediaMobileGyroADCSum[axis] +=
            mediaMobileGyroADC[axis][mediaMobileGyroIDX];
        imu.gyroADC[axis] = mediaMobileGyroADCSum[axis] / conf.mmgyro;
# else
        //anti gyro glitch, limit the variation between two consecutive readings
        imu.gyroADC[axis] =
            constrain(imu.gyroADC[axis], previousGyroADC[axis] - 800,
                      previousGyroADC[axis] + 800);
# endif
        previousGyroADC[axis] = imu.gyroADC[axis];
    }

# if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ( (imu.gyroADC[PITCH] - imu.gyroADC[ROLL]) * 7 ) / 10;
    imu.gyroADC[ROLL] = ( (imu.gyroADC[ROLL] + imu.gyroADC[PITCH]) * 7 ) / 10;
    imu.gyroADC[PITCH] = temp;
# endif
# if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ( (imu.gyroADC[PITCH] + imu.gyroADC[ROLL]) * 7 ) / 10;
    imu.gyroADC[ROLL] = ( (imu.gyroADC[ROLL] - imu.gyroADC[PITCH]) * 7 ) / 10;
    imu.gyroADC[PITCH] = temp;
# endif
}

#endif // IMPLEMENTATION

#if defined(L3G4200D)
# include "gyro/l3g4200d.cpp"
#elif defined(ITG3200) || defined(ITG3050) || defined(MPU3050)
# include "gyro/itg3200_itg3205_itg3050_mpu3050.cpp"
#elif defined(LSM330)
# include "gyro/lsm330.cpp"
#elif defined(MPU6050)
# include "gyro/mpu6050.cpp"
#elif defined(MPU9250)
# include "gyro/mpu9250.cpp"
#elif defined(WMP)
# include "gyro/wii_motion_plus.cpp"
#endif
