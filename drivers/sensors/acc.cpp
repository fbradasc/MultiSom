#if defined(IMPLEMENTATION)
// ****************
// ACC common part
// ****************
void
ACC_Common()
{
    static int32_t a[3];

    if (calibratingA > 0)
    {
        calibratingA--;

        for (uint8_t axis = 0; axis < 3; axis++)
        {
            if (calibratingA == 511)
            {
                a[axis] = 0;    // Reset a[axis] at start of calibration
            }

            a[axis] += imu.accADC[axis];    // Sum up 512 readings
            global_conf.accZero[axis] = a[axis] >> 9;   // Calculate average, only the last itteration where (calibratingA == 0) is relevant
        }

        if (calibratingA == 0)
        {
            global_conf.accZero[YAW] -= ACC_1G; // shift Z down by ACC_1G and store values in EEPROM at end of calibration
            conf.angleTrim[ROLL] = 0;
            conf.angleTrim[PITCH] = 0;
            writeGlobalSet(1);  // write accZero in EEPROM
        }
    }

#if defined(INFLIGHT_ACC_CALIBRATION)
    static int32_t b[3];
    static int16_t accZero_saved[3] = { 0, 0, 0 };
    static int16_t angleTrim_saved[2] = { 0, 0 };

    //Saving old zeropoints before measurement
    if (InflightcalibratingA == 50)
    {
        accZero_saved[ROLL] = global_conf.accZero[ROLL];
        accZero_saved[PITCH] = global_conf.accZero[PITCH];
        accZero_saved[YAW] = global_conf.accZero[YAW];
        angleTrim_saved[ROLL] = conf.angleTrim[ROLL];
        angleTrim_saved[PITCH] = conf.angleTrim[PITCH];
    }

    if (InflightcalibratingA > 0)
    {
        for (uint8_t axis = 0; axis < 3; axis++)
        {
            // Reset a[axis] at start of calibration
            if (InflightcalibratingA == 50)
            {
                b[axis] = 0;
            }

            // Sum up 50 readings
            b[axis] += imu.accADC[axis];
            // Clear global variables for next reading
            imu.accADC[axis] = 0;
            global_conf.accZero[axis] = 0;
        }

        //all values are measured
        if (InflightcalibratingA == 1)
        {
            AccInflightCalibrationActive = 0;
            AccInflightCalibrationMeasurementDone = 1;
            SET_ALARM_BUZZER(ALRM_FAC_CONFIRM, ALRM_LVL_CONFIRM_1);     //buzzer for indicatiing the end of calibration
            // recover saved values to maintain current flight behavior until new values are transferred
            global_conf.accZero[ROLL] = accZero_saved[ROLL];
            global_conf.accZero[PITCH] = accZero_saved[PITCH];
            global_conf.accZero[YAW] = accZero_saved[YAW];
            conf.angleTrim[ROLL] = angleTrim_saved[ROLL];
            conf.angleTrim[PITCH] = angleTrim_saved[PITCH];
        }

        InflightcalibratingA--;
    }

    // Calculate average, shift Z down by ACC_1G and store values in EEPROM at end of calibration
    if (AccInflightCalibrationSavetoEEProm == 1)
    {
        //the copter is landed, disarmed and the combo has been done again
        AccInflightCalibrationSavetoEEProm = 0;
        global_conf.accZero[ROLL] = b[ROLL] / 50;
        global_conf.accZero[PITCH] = b[PITCH] / 50;
        global_conf.accZero[YAW] = b[YAW] / 50 - ACC_1G;
        conf.angleTrim[ROLL] = 0;
        conf.angleTrim[PITCH] = 0;
        writeGlobalSet(1);  // write accZero in EEPROM
    }

#endif
    imu.accADC[ROLL] -= global_conf.accZero[ROLL];
    imu.accADC[PITCH] -= global_conf.accZero[PITCH];
    imu.accADC[YAW] -= global_conf.accZero[YAW];
#if defined(SENSORS_TILT_45DEG_LEFT)
    int16_t temp = ((imu.accADC[PITCH] - imu.accADC[ROLL]) * 7) / 10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL] + imu.accADC[PITCH]) * 7) / 10;
    imu.accADC[PITCH] = temp;
#endif
#if defined(SENSORS_TILT_45DEG_RIGHT)
    int16_t temp = ((imu.accADC[PITCH] + imu.accADC[ROLL]) * 7) / 10;
    imu.accADC[ROLL] = ((imu.accADC[ROLL] - imu.accADC[PITCH]) * 7) / 10;
    imu.accADC[PITCH] = temp;
#endif
}
#endif // IMPLEMENTATION

#if defined(MMA7455)
    #include "acc/mma7455.cpp"
#elif defined(MMA8451Q)
    #include "acc/mma8451q.cpp"
#elif defined(ADXL345)
    #include "acc/adxl345.cpp"
#elif defined(BMA180)
    #include "acc/bma180.cpp"
#elif defined(BMA280)
    #include "acc/bma280.cpp"
#elif defined(BMA020)
    #include "acc/bma020.cpp"
#elif defined(LIS3LV02)
    #include "acc/lis3lv02.cpp"
#elif defined(LSM303DLx_ACC)
    #include "acc/lsm303dlx.cpp"
#elif defined(LSM330)
    #include "acc/lsm330.cpp"
#elif defined(ADCACC)
    #include "acc/adc.cpp"
#elif defined(MPU6050)
    #include "acc/mpu6050.cpp"
#elif defined(MPU9250)
    #include "acc/mpu9250.cpp"
#endif

#if !defined(IMPLEMENTATION)
    #if !defined(ACC_1G)
        #define ACC_1G 256
    #endif
#endif // IMPLEMENTATION
