/**************************************************************************************/
/***********************       UBLOX                         **************************/
/**************************************************************************************/

#if !defined(GPS_SV_MAXSATS)
# define GPS_SV_MAXSATS 16;
#endif

#if defined(IMPLEMENTATION)

uint8_t GPS_numCh = 0;                      // number of channels
uint8_t GPS_svinfo_chn    [GPS_SV_MAXSATS]; // Channel number
uint8_t GPS_svinfo_svid   [GPS_SV_MAXSATS]; // Satellite ID
uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t GPS_svinfo_cno    [GPS_SV_MAXSATS]; // Carrier to Noise Ratio (Signal Strength)

const char UBLOX_INIT[] PROGMEM =                                                    // PROGMEM array must be outside any function !!!
{
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,                  //disable all default NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,                  //set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,                  //set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,                  //set SOL MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,                  //set VELNED MSG rate
    0xB5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x07, 0x03, 0x00, 0x51, 0x08, 0x00, 0x00, 0x8A, 0x41, //set WAAS to EGNOS
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A //set rate to 5Hz
};

struct ubx_header
{
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
};
struct ubx_nav_posllh
{
    uint32_t time;  // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitude_msl;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
};
struct ubx_nav_solution
{
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
};
struct ubx_nav_velned
{
    uint32_t time;  // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
};
typedef struct
{
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;
typedef struct
{
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[GPS_SV_MAXSATS];         // 16 satellites * 12 byte
} ubx_nav_svinfo;

enum ubs_protocol_bytes
{
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
};
enum ubs_nav_fix_type
{
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
};
enum ubx_nav_status_bits
{
    NAV_STATUS_FIX_VALID = 1
};

// Receive buffer
static union
{
    ubx_nav_posllh posllh;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    ubx_nav_svinfo svinfo;
    uint8_t bytes[];
}
_buffer;

uint32_t init_speed[5] = {9600, 19200, 38400, 57600, 115200};

static void SerialGpsPrint(const char PROGMEM *str)    // http://www.multiwii.com/forum/viewtopic.php?f=8&t=4167
{
    char b;

    while (str && (b = pgm_read_byte(str++) ) )
    {
        SerialWrite(GPS_SERIAL, b);
        delay(5);
    }
}

void GPS_SerialInit(void)
{
    SerialOpen(GPS_SERIAL, GPS_BAUD);
    delay(1000);

    for (uint8_t i = 0; i < 5; i++)
    {
        SerialOpen(GPS_SERIAL, init_speed[i]);         // switch UART speed for sending SET BAUDRATE command (NMEA mode)
# if (GPS_BAUD == 19200)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,19200,0*23\r\n") );     // 19200 baud - minimal speed for 5Hz update rate
# endif
# if (GPS_BAUD == 38400)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,38400,0*26\r\n") );     // 38400 baud
# endif
# if (GPS_BAUD == 57600)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,57600,0*2D\r\n") );     // 57600 baud
# endif
# if (GPS_BAUD == 115200)
        SerialGpsPrint(PSTR("$PUBX,41,1,0003,0001,115200,0*1E\r\n") );    // 115200 baud
# endif

        while (!SerialTXfree(GPS_SERIAL) )
        {
            delay(10);
        }
    }

    delay(200);
    SerialOpen(GPS_SERIAL, GPS_BAUD);

    for (uint8_t i = 0; i < sizeof(UBLOX_INIT); i++)                     // send configuration data in UBX protocol
    {
        SerialWrite(GPS_SERIAL, pgm_read_byte(UBLOX_INIT + i) );
        delay(5); //simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
    }
}

bool GPS_newFrame(uint8_t data)
{
    static uint8_t _step = 0;  // State machine state
    static uint8_t _msg_id;
    static uint16_t _payload_length;
    static uint16_t _payload_counter;
    static uint8_t _ck_a;  // Packet checksum accumulators
    static uint8_t _ck_b;
    uint8_t st = _step + 1;
    bool ret = false;
    uint32_t i;

    if (st == 2)
    {
        if (PREAMBLE2 != data)
        {
            st--;    // in case of faillure of the 2nd header byte, still test the first byte
        }
    }

    if (st == 1)
    {
        if (PREAMBLE1 != data)
        {
            st--;
        }
    }
    else
    if (st == 3)          // CLASS byte, not used, assume it is CLASS_NAV
    {
        _ck_b = _ck_a = data;  // reset the checksum accumulators
    }
    else
    if ( ( st > 3) && ( st < 8) )
    {
        _ck_b += (_ck_a += data);  // checksum byte

        if (st == 4)
        {
            _msg_id = data;
        }
        else
        if (st == 5)
        {
            _payload_length = data; // payload length low byte
        }
        else
        if (st == 6)
        {
            _payload_length += (uint16_t)(data << 8);

            if (_payload_length > 512)
            {
                st = 0;
            }

            _payload_counter = 0;  // prepare to receive payload
        }
        else
        {
            if (_payload_counter + 1 < _payload_length)
            {
                st--;    // stay in the same state while data inside the frame
            }

            if (_payload_counter < sizeof(_buffer) )
            {
                _buffer.bytes[_payload_counter] = data;
            }

            _payload_counter++;
        }
    }
    else
    if (st == 8)
    {
        if (_ck_a != data)
        {
            st = 0;    // bad checksum
        }
    }
    else
    if (st == 9)
    {
        st = 0;

        if (_ck_b == data)   // good checksum
        {
            debug[2] = _msg_id;

            if (_msg_id == MSG_POSLLH)
            {
                if (f.GPS_FIX)
                {
                    GPS_coord[LON] = _buffer.posllh.longitude;
                    GPS_coord[LAT] = _buffer.posllh.latitude;
                    GPS_altitude = _buffer.posllh.altitude_msl / 1000;   //alt in m
# if !defined(Ardhat)
                    GPS_time = _buffer.posllh.time;       //not used for the moment
# endif
                }

                ret = true;       // POSLLH message received, allow blink GUI icon and LED, frame available for nav computation
            }
            else
            if (_msg_id == MSG_SOL)
            {
                f.GPS_FIX = 0;

                if ( (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && ( (_buffer.solution.fix_type == FIX_3D) || (_buffer.solution.fix_type == FIX_2D) ) )
                {
                    f.GPS_FIX = 1;
                }

                GPS_numSat = _buffer.solution.satellites;
            }
            else
            if (_msg_id == MSG_VELNED)
            {
                GPS_speed = _buffer.velned.speed_2d;          // cm/s
                GPS_ground_course = (uint16_t)(_buffer.velned.heading_2d / 10000);  // Heading 2D deg * 100000 rescaled to deg * 10 //not used for the moment
            }
            else
            if (_msg_id == MSG_SVINFO)
            {
                // gpsPacketLogChar = LOG_UBLOX_SVINFO;
                GPS_numCh = _buffer.svinfo.numCh;

                if (GPS_numCh > GPS_SV_MAXSATS)
                {
                    GPS_numCh = GPS_SV_MAXSATS;
                }

                for (i = 0; i < GPS_numCh; i++)
                {
                    GPS_svinfo_chn[i] = _buffer.svinfo.channel[i].chn;
                    GPS_svinfo_svid[i] = _buffer.svinfo.channel[i].svid;
                    GPS_svinfo_quality[i] = _buffer.svinfo.channel[i].quality;
                    GPS_svinfo_cno[i] = _buffer.svinfo.channel[i].cno;
                }

                // GPS_svInfoReceivedCount++;
            }
        }
    }

    _step = st;
    return(ret);
}

#else // IMPLEMENTATION
extern uint8_t GPS_numCh;                          // number of channels
extern uint8_t GPS_svinfo_chn    [GPS_SV_MAXSATS]; // Channel number
extern uint8_t GPS_svinfo_svid   [GPS_SV_MAXSATS]; // Satellite ID
extern uint8_t GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
extern uint8_t GPS_svinfo_cno    [GPS_SV_MAXSATS]; // Carrier to Noise Ratio (Signal Strength)
#endif
