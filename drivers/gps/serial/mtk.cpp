/**************************************************************************************/
/***********************       MTK                           **************************/
/**************************************************************************************/

#define MTK_SET_BINARY          PSTR("$PGCMD,16,0,0,0,0,0*6A\r\n")
#define MTK_SET_NMEA            PSTR("$PGCMD,16,1,1,1,1,1*6B\r\n")
#define MTK_SET_NMEA_SENTENCES  PSTR("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
#define MTK_OUTPUT_4HZ          PSTR("$PMTK220,250*29\r\n")
#define MTK_OUTPUT_5HZ          PSTR("$PMTK220,200*2C\r\n")
#define MTK_OUTPUT_10HZ         PSTR("$PMTK220,100*2F\r\n")
#define MTK_NAVTHRES_OFF        PSTR("$PMTK397,0*23\r\n")   // Set Nav Threshold (the minimum speed the GPS must be moving to update the position) to 0 m/s
#define SBAS_ON                 PSTR("$PMTK313,1*2E\r\n")
#define WAAS_ON                 PSTR("$PMTK301,2*2E\r\n")
#define SBAS_TEST_MODE          PSTR("$PMTK319,0*25\r\n")   //Enable test use of sbas satelite in test mode (usually PRN124 is in test mode)

struct diyd_mtk_msg
{
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    int32_t ground_speed;
    int32_t ground_course;
    uint8_t satellites;
    uint8_t fix_type;
    uint32_t utc_date;
    uint32_t utc_time;
    uint16_t hdop;
};

// #pragma pack(pop)
enum diyd_mtk_fix_type
{
    FIX_NONE = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_2D_SBAS = 6,
    FIX_3D_SBAS = 7
};

#if defined(MTK_BINARY16)
enum diyd_mtk_protocol_bytes
{
    PREAMBLE1 = 0xd0,
    PREAMBLE2 = 0xdd,
};
#endif

#if defined(MTK_BINARY19)
enum diyd_mtk_protocol_bytes
{
    PREAMBLE1 = 0xd1,
    PREAMBLE2 = 0xdd,
};
#endif

// Packet checksum accumulators
uint8_t _ck_a;
uint8_t _ck_b;

// State machine state
uint8_t _step;
uint8_t _payload_counter;

// Time from UNIX Epoch offset
long _time_offset;
bool _offset_calculated;

// Receive buffer
union
{
    diyd_mtk_msg msg;
    uint8_t bytes[];
}
_buffer;

inline long _swapl(const void *bytes)
{
    const uint8_t *b = (const uint8_t *)bytes;

    union
    {
        long v;
        uint8_t b[4];
    }
    u;
    u.b[0] = b[3];
    u.b[1] = b[2];
    u.b[2] = b[1];
    u.b[3] = b[0];
    return (u.v);
}

uint32_t init_speed[5] = {9600, 19200, 38400, 57600, 115200};

void SerialGpsPrint(const char PROGMEM *str)
{
    char b;

    while (str && (b = pgm_read_byte(str++) ) )
    {
        SerialWrite(GPS_SERIAL, b);
    }
}

void GPS_SerialInit(void)
{
    SerialOpen(GPS_SERIAL, GPS_BAUD);
    delay(1000);
#if defined(INIT_MTK_GPS)                              // MTK GPS setup
    for (uint8_t i = 0; i < 5; i++)
    {
        SerialOpen(GPS_SERIAL, init_speed[i]);               // switch UART speed for sending SET BAUDRATE command
# if (GPS_BAUD == 19200)
        SerialGpsPrint(PSTR("$PMTK251,19200*22\r\n") );     // 19200 baud - minimal speed for 5Hz update rate
# endif
# if (GPS_BAUD == 38400)
        SerialGpsPrint(PSTR("$PMTK251,38400*27\r\n") );     // 38400 baud
# endif
# if (GPS_BAUD == 57600)
        SerialGpsPrint(PSTR("$PMTK251,57600*2C\r\n") );     // 57600 baud
# endif
# if (GPS_BAUD == 115200)
        SerialGpsPrint(PSTR("$PMTK251,115200*1F\r\n") );    // 115200 baud
# endif

        while (!SerialTXfree(GPS_SERIAL) )
        {
            delay(80);
        }
    }

    // at this point we have GPS working at selected (via #define GPS_BAUD) baudrate
    // So now we have to set the desired mode and update rate (which depends on the NMEA or MTK_BINARYxx settings)
    SerialOpen(GPS_SERIAL, GPS_BAUD);
    SerialGpsPrint(MTK_NAVTHRES_OFF);

    while (!SerialTXfree(GPS_SERIAL) )
    {
        delay(80);
    }

    SerialGpsPrint(SBAS_ON);

    while (!SerialTXfree(GPS_SERIAL) )
    {
        delay(80);
    }

    SerialGpsPrint(WAAS_ON);

    while (!SerialTXfree(GPS_SERIAL) )
    {
        delay(80);
    }

    SerialGpsPrint(SBAS_TEST_MODE);

    while (!SerialTXfree(GPS_SERIAL) )
    {
        delay(80);
    }

    SerialGpsPrint(MTK_OUTPUT_5HZ);           // 5 Hz update rate
# if defined(NMEA)
    SerialGpsPrint(MTK_SET_NMEA_SENTENCES); // only GGA and RMC sentence
# endif
# if defined(MTK_BINARY19) || defined(MTK_BINARY16)
    SerialGpsPrint(MTK_SET_BINARY);
# endif
#endif // init_mtk_gps
}

bool GPS_newFrame(uint8_t data)
{
    bool parsed = false;

restart:

    switch (_step)
    {
        // Message preamble, class, ID detection
        //
        // If we fail to match any of the expected bytes, we
        // reset the state machine and re-consider the failed
        // byte as the first byte of the preamble.  This
        // improves our chances of recovering from a mismatch
        // and makes it less likely that we will be fooled by
        // the preamble appearing as data in some other message.
        //
        case 0:
        {
            if (PREAMBLE1 == data)
            {
                _step++;
            }

            break;
        }

        case 1:
        {
            if (PREAMBLE2 == data)
            {
                _step++;
                break;
            }

            _step = 0;
            goto restart;
        }

        case 2:
        {
            if (sizeof(_buffer) == data)
            {
                _step++;
                _ck_b = _ck_a = data;                  // reset the checksum accumulators
                _payload_counter = 0;
            }
            else
            {
                _step = 0;                             // reset and wait for a message of the right class
                goto restart;
            }

            break;
        }

        // Receive message data
        case 3:
        {
            _buffer.bytes[_payload_counter++] = data;
            _ck_b += (_ck_a += data);

            if (_payload_counter == sizeof(_buffer) )
            {
                _step++;
            }

            break;
        }

        // Checksum and message processing
        case 4:
        {
            _step++;

            if (_ck_a != data)
            {
                _step = 0;
            }

            break;
        }

        case 5:
            _step = 0;

            if (_ck_b != data)
            {
                break;
            }

            f.GPS_FIX = ( (_buffer.msg.fix_type == FIX_3D) || (_buffer.msg.fix_type == FIX_3D_SBAS) );
#if defined(MTK_BINARY16)
            GPS_coord[LAT] = _buffer.msg.latitude * 10;                 // XXX doc says *10e7 but device says otherwise
            GPS_coord[LON] = _buffer.msg.longitude * 10;                // XXX doc says *10e7 but device says otherwise
#endif
#if defined(MTK_BINARY19)
            GPS_coord[LAT] = _buffer.msg.latitude;                      // With 1.9 now we have real 10e7 precision
            GPS_coord[LON] = _buffer.msg.longitude;
#endif
            GPS_altitude = _buffer.msg.altitude / 100;                  // altitude in meter
            GPS_speed = _buffer.msg.ground_speed;                       // in m/s * 100 == in cm/s
            GPS_ground_course = _buffer.msg.ground_course / 100;           //in degrees
            GPS_numSat = _buffer.msg.satellites;
#if !defined(Ardhat)
            GPS_time = _buffer.msg.utc_time;
#endif
            //GPS_hdop                  = _buffer.msg.hdop;
            parsed = true;
    }

    return(parsed);
}
