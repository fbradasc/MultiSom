// ****************************************************************
// FrSky SmartPort telemetry
// Version: 0.4.0   - by QuadBow
// Changes: V0.4.0: - supports 2.4 with new features added
//                    device specific selection of data to be sent
//                    different ways for displaying
//                    different ways to present data
// Version: 0.3.0
// Changes: V0.3.0: - new structure with cpp/h-files
// Date 20/09/2012
// Changes: V0.2.1: - make it work with 2.1 (shared dev)
// Date: 14/08/2012
// Changes: V0.2: - Byte stuffing added
//                - vBat will be send, if "#define FAS_100" is comment out
//          V0.1: - First release
// ****************************************************************

#if defined(IMPLEMENTATION)

static short _FrSkySport_crc;
static short _currentGPSValue;

void FrSkySport_sendByte(uint8_t byte)
{
    // CRC update
    _FrSkySport_crc += byte; //0-1FF
    _FrSkySport_crc += _FrSkySport_crc >> 8; //0-100
    _FrSkySport_crc &= 0x00ff;
    _FrSkySport_crc += _FrSkySport_crc >> 8; //0-0FF
    _FrSkySport_crc &= 0x00ff;

    if ( (byte == FRSKY_START_STOP) || (byte == FRSKY_BYTESTUFF) )
    {
        SerialWrite(TELEMETRY_SERIAL, FRSKY_BYTESTUFF);
        byte &= ~FRSKY_STUFF_MASK;
    }

    SerialWrite(TELEMETRY_SERIAL, byte);
}

void FrSkySport_sendCrc()
{
    FrSkySport_sendByte(0xFF - _FrSkySport_crc);
}

void FrSkySport_sendValue(uint16_t id, uint32_t value)
{
    _FrSkySport_crc = 0; // Reset CRC
    FrSkySport_sendByte(0x10); // DATA_FRAME
    uint8_t *bytes = (uint8_t*)&id;
    FrSkySport_sendByte(bytes[0]);
    FrSkySport_sendByte(bytes[1]);
    bytes = (uint8_t*)&value;
    FrSkySport_sendByte(bytes[0]);
    FrSkySport_sendByte(bytes[1]);
    FrSkySport_sendByte(bytes[2]);
    FrSkySport_sendByte(bytes[3]);
    FrSkySport_sendCrc();
}

void FrSkySport_sendA2voltage()
{
# ifdef VBAT
    uint32_t opentx_val = (255.0 * (float)(analog.vbat / (float)FRSKY_SPORT_A2_MAX) );
    FrSkySport_sendValue(FRSKY_SPORT_ADC2_ID, (opentx_val) );
# endif
}

uint32_t FrSkySport_EncodeCoordinate(float latLon, bool isLat)
{
# if GPS
    uint32_t otx_coord = 0;

    if (!isLat)
    {
        otx_coord = abs(latLon);  // now we have unsigned value and one bit to spare
        otx_coord = (otx_coord + otx_coord / 2) / 25 | 0x80000000;  // 6/100 = 1.5/25, division by power of 2 is fast

        if (latLon < 0)
        {
            otx_coord |= 0x40000000;
        }
    }
    else
    {
        otx_coord = abs(latLon);  // now we have unsigned value and one bit to spare
        otx_coord = (otx_coord + otx_coord / 2) / 25;  // 6/100 = 1.5/25, division by power of 2 is fast

        if (latLon < 0)
        {
            otx_coord |= 0x40000000;
        }
    }
    return(otx_coord);
# endif
}

void FrSkySport_sendGPSCoordinate()
{
# if GPS
    uint32_t GPSValueToSend = 0;

    if (f.GPS_FIX && (GPS_numSat >= 4) )
    {
        switch (_currentGPSValue)
        {
            case 0:
            {
                GPSValueToSend = FrSkySport_EncodeCoordinate(GPS_coord[LON], false);
                _currentGPSValue = 1;
                break;
            }

            case 1:
            {
                GPSValueToSend = FrSkySport_EncodeCoordinate(GPS_coord[LAT], true);
                _currentGPSValue = 0;
                break;
            }
        }

        FrSkySport_sendValue(FRSKY_SPORT_GPS_LONG_LATI_ID, GPSValueToSend);
    }
# endif
}

void FrSkySport_sendGPSSpeed()
{
# if GPS
    if (f.GPS_FIX && (GPS_numSat >= 4) )
    {
        uint32_t speed = ( (float)GPS_speed * 100 );
        FrSkySport_sendValue(FRSKY_SPORT_GPS_SPEED_ID, speed); // unknown unit, just a guess
    }
# endif
}

void FrSkySport_sendAltitude()
{
# if defined(TELEMETRY_ALT_BARO) and BARO
    FrSkySport_sendValue(FRSKY_SPORT_ALT_ID, (int32_t)(alt.EstAlt) * 100); // cm
# endif
}

void FrSkySport_sendHeading()
{
# if defined(TELEMETRY_COURSE_MAG)or (defined(TELEMETRY_COURSE_GPS) and GPS)
#  if defined(TELEMETRY_COURSE_MAG)
    uint32_t otx_heading = (uint32_t)(att.heading + 360) % 360 * 100;
    FrSkySport_sendValue(FRSKY_SPORT_GPS_COURS_ID, otx_heading); // 1 deg = 100, 0 - 359000
#  elif defined(TELEMETRY_COURSE_GPS) && defined(GPS)
    if (f.GPS_FIX && (GPS_numSat >= 4) )
    {
        uint32_t otx_heading = (uint32_t)(GPS_ground_course + 360) % 360 * 100;
        FrSkySport_sendValue(FRSKY_SPORT_GPS_COURS_ID, otx_heading); // 1 deg = 100, 0 - 359000
    }
#  endif
# endif
}

void FrSkySport_sendACCX()
{
# ifdef ACC
    FrSkySport_sendValue(FRSKY_SPORT_ACCX_ID, imu.accSmooth[0] / 5); // unknown unit
# endif
}

void FrSkySport_sendACCY()
{
# ifdef ACC
    FrSkySport_sendValue(FRSKY_SPORT_ACCY_ID, imu.accSmooth[1] / 5); // unknown unit
# endif
}

void FrSkySport_sendACCZ()
{
# ifdef ACC
    FrSkySport_sendValue(FRSKY_SPORT_ACCZ_ID, imu.accSmooth[2] / 5); // unknown unit
# endif
}

void FrSkySport_sendAltVario()
{
# ifdef VARIOMETER
    FrSkySport_sendValue(FRSKY_SPORT_VARIO_ID, ( (uint32_t)alt.vario ) ); // unknown unit
# endif
}

void init_telemetry(void)
{
    _currentGPSValue = 0;
    SerialOpen(TELEMETRY_SERIAL, TELEMETRY_BAUD);
}

void run_telemetry(void)
{
    static uint8_t lastRx = 0;
    uint8_t c = SerialAvailable(TELEMETRY_SERIAL);

    while (c--)
    {
        int rx = SerialRead(TELEMETRY_SERIAL);

        if (lastRx == FRSKY_START_STOP)
        {
            debug[1] = rx;

            switch (rx)
            {
                case FRSKY_SPORT_DEVICE_4:
                {
                    FrSkySport_sendA2voltage();
                    break;
                }

                case FRSKY_SPORT_DEVICE_8:
                {
                    FrSkySport_sendACCX();
                    break;
                }

                case FRSKY_SPORT_DEVICE_9:
                {
                    FrSkySport_sendACCY();
                    break;
                }

                case FRSKY_SPORT_DEVICE_10:
                {
                    FrSkySport_sendACCZ();
                    break;
                }

                case FRSKY_SPORT_DEVICE_11:
                {
                    FrSkySport_sendAltitude();
                    break;
                }

                case FRSKY_SPORT_DEVICE_12:
                {
                    FrSkySport_sendAltVario();
                    break;
                }

                case FRSKY_SPORT_DEVICE_13:
                {
                    FrSkySport_sendHeading();
                    break;
                }

                case FRSKY_SPORT_DEVICE_14:
                {
                    FrSkySport_sendGPSSpeed();
                    break;
                }

                case FRSKY_SPORT_DEVICE_16:
                {
                    FrSkySport_sendGPSCoordinate();
                    break;
                }
            }
        }

        lastRx = rx;
    }
}

#else // IMPLEMENTATION

# define TELEMETRY_BAUD                57600

# define FRSKY_START_STOP              0x7e
# define FRSKY_BYTESTUFF               0x7d
# define FRSKY_STUFF_MASK              0x20

// FrSky data IDs (2 bytes)
# define FRSKY_SPORT_RSSI_ID           0xf101
# define FRSKY_SPORT_ADC1_ID           0xf102// A1
# define FRSKY_SPORT_ADC2_ID           0xf103// A2
# define FRSKY_SPORT_BATT_ID           0xf104
# define FRSKY_SPORT_SWR_ID            0xf105
# define FRSKY_SPORT_T1_ID             0x0400
# define FRSKY_SPORT_T2_ID             0x0410
# define FRSKY_SPORT_RPM_ID            0x0500
# define FRSKY_SPORT_FUEL_ID           0x0600
# define FRSKY_SPORT_ALT_ID            0x0100
# define FRSKY_SPORT_VARIO_ID          0x0110
# define FRSKY_SPORT_ACCX_ID           0x0700
# define FRSKY_SPORT_ACCY_ID           0x0710
# define FRSKY_SPORT_ACCZ_ID           0x0720
# define FRSKY_SPORT_CURR_ID           0x0200
# define FRSKY_SPORT_VFAS_ID           0x0210
# define FRSKY_SPORT_CELLS_ID          0x0300
# define FRSKY_SPORT_GPS_LONG_LATI_ID  0x0800
# define FRSKY_SPORT_GPS_ALT_ID        0x0820
# define FRSKY_SPORT_GPS_SPEED_ID      0x0830
# define FRSKY_SPORT_GPS_COURS_ID      0x0840
# define FRSKY_SPORT_GPS_TIME_DATE_ID  0x0850

// FrSky sensor IDs (this also happens to be the order in which they're broadcast from an X8R)
// NOTE: As FrSky puts out more sensors let's try to add comments here indicating which is which
# define FRSKY_SPORT_DEVICE_1          0xa1
# define FRSKY_SPORT_DEVICE_2          0x22
# define FRSKY_SPORT_DEVICE_3          0x83
# define FRSKY_SPORT_DEVICE_4          0xe4
# define FRSKY_SPORT_DEVICE_5          0x45
# define FRSKY_SPORT_DEVICE_6          0xc6
# define FRSKY_SPORT_DEVICE_7          0x67
# define FRSKY_SPORT_DEVICE_8          0x48
# define FRSKY_SPORT_DEVICE_9          0xe9
# define FRSKY_SPORT_DEVICE_10         0x6a
# define FRSKY_SPORT_DEVICE_11         0xcb
# define FRSKY_SPORT_DEVICE_12         0xac
# define FRSKY_SPORT_DEVICE_13         0xd
# define FRSKY_SPORT_DEVICE_14         0x8e
# define FRSKY_SPORT_DEVICE_15         0x2f
# define FRSKY_SPORT_DEVICE_16         0xd0
# define FRSKY_SPORT_DEVICE_17         0x71
# define FRSKY_SPORT_DEVICE_18         0xf2
# define FRSKY_SPORT_DEVICE_19         0x53
# define FRSKY_SPORT_DEVICE_20         0x34
# define FRSKY_SPORT_DEVICE_21         0x95
# define FRSKY_SPORT_DEVICE_22         0x16
# define FRSKY_SPORT_DEVICE_23         0xb7
# define FRSKY_SPORT_DEVICE_24         0x98
# define FRSKY_SPORT_DEVICE_25         0x39
# define FRSKY_SPORT_DEVICE_26         0xba
# define FRSKY_SPORT_DEVICE_27         0x1b

#endif // IMPLEMENTATION
