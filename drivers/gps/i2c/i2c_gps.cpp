/**************************************************************************************/
/***************                       I2C GPS                     ********************/
/**************************************************************************************/

#define I2C_GPS_ADDRESS               0x20  //7 bits

#define I2C_GPS_STATUS_00             00    //(Read only)
#define I2C_GPS_STATUS_NEW_DATA       0x01  // New data is available (after every GGA frame)
#define I2C_GPS_STATUS_2DFIX          0x02  // 2dfix achieved
#define I2C_GPS_STATUS_3DFIX          0x04  // 3dfix achieved
#define I2C_GPS_STATUS_NUMSATS        0xF0  // Number of sats in view
#define I2C_GPS_LOCATION              07    // current location 8 byte (lat, lon) int32_t
#define I2C_GPS_GROUND_SPEED          31    // GPS ground speed in m/s*100 (uint16_t)      (Read Only)
#define I2C_GPS_ALTITUDE              33    // GPS altitude in meters (uint16_t)           (Read Only)
#define I2C_GPS_GROUND_COURSE         35    // GPS ground course (uint16_t)
#define I2C_GPS_TIME                  39    // UTC Time from GPS in hhmmss.sss * 100 (uint32_t)(unneccesary precision) (Read Only)
#define I2C_GPS_SONAR_ALT             239   // Sonar Altitude

uint8_t GPS_NewData(void)
{
    uint8_t i2c_gps_status;

    i2c_gps_status = i2c_readReg(I2C_GPS_ADDRESS, I2C_GPS_STATUS_00);                //Get status register
#if defined(I2C_GPS_SONAR)
    i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_SONAR_ALT, (uint8_t *)&sonarAlt, 2);
#endif
    f.GPS_FIX = 0;

    if (i2c_gps_status & I2C_GPS_STATUS_3DFIX)                                       //Check is we have a good 3d fix (numsats>5)
    {
        f.GPS_FIX = 1;

        if (i2c_gps_status & I2C_GPS_STATUS_NEW_DATA)                                  //Check about new data
        {
            GPS_Frame = 1;
            GPS_FAIL_timer = millis();

            if (GPS_update == 1)
            {
                GPS_update = 0;
            }
            else
            {
                GPS_update = 1;    //Blink GPS update
            }

            GPS_numSat = i2c_gps_status >> 4;
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_LOCATION,     (uint8_t *)&GPS_coord[LAT], 4);
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_LOCATION + 4, (uint8_t *)&GPS_coord[LON], 4);
            // note: the following vars are currently not used in nav code -- avoid retrieving it to save time
            // This part is Nessesary for FIXEDWING
#if defined(FIXEDWING)
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_GROUND_SPEED,  (uint8_t *)&GPS_speed,         2);
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_ALTITUDE,      (uint8_t *)&GPS_altitude,      2);
            i2c_read_reg_to_buf(I2C_GPS_ADDRESS, I2C_GPS_GROUND_COURSE, (uint8_t *)&GPS_ground_course, 2);
#endif
            return(1);
        }
    }

    return(0);
}
