/**************************************************************************************/
/***********************       SkyTraq Venus838FLPx          **************************/
/************************      57600 baud                   ***************************/
/************************      update rate 5Hz              ***************************/
/**************************************************************************************/

// Input System Messages
#define VENUS_CONFIG_SERIAL_PORT          0x05
#define VENUS_CONFIG_OUTPUT_MSG_FORMAT    0x09
#define VENUS_CONFIG_POWER_MODE           0x0C
#define VENUS_CONFIG_GPS_UPDATE_RATE      0x0E

// Input GPS Messages
#define VENUS_CONFIG_GPS_PINNING          0x39
#define VENUS_CONFIG_GPS_PINNING_PARAMS   0x3B
#define VENUS_CONFIG_NAV_MODE             0x3C

// Output GPS Messages
#define VENUS_GPS_LOCATION                0xA8

// command messages
#define VENUS8_EXT2                       0x62
#define VENUS8_EXT3                       0x63
#define VENUS8_EXT4                       0x64
#define VENUS8_CONFIG_SBAS                0x01   // w/EXT2
#define VENUS8_CONFIG_INTERFERENCE_DETECT 0x06    // w/EXT4
#define VENUS8_CONFIG_NAV_MODE            0x17   // w/EXT4
#define VENUS8_CONFIG_SAGPS               0x01   // w/EXT3

typedef struct
{
    int32_t x, y, z;
} xyz32_t;

typedef struct
{
    uint8_t fixmode;
    uint8_t sv_count;   // satellites
    uint16_t gps_week;
    uint32_t gps_tow;
    int32_t latitude;
    int32_t longitude;
    uint32_t ellipsoid_alt;
    uint32_t sealevel_alt;
    uint16_t gdop, pdop, hdop, vdop, tdop;
    xyz32_t ecef, vel;
} venus_location;

typedef struct
{
    uint8_t id;     // message id
    union
    {
        uint8_t body[];
        venus_location location;
    };
} venus_message;

typedef struct
{
    union
    {
        uint8_t payload[];
        venus_message message;
    };
} venus_payload;

static venus_payload venus_ctx;

#define SWAP16(x) ( (x & 0xff) << 8 | (x >> 8) )
#define SWAP32(x) ( (x & 0xff) << 24 | ( (x & 0xff00) << 8 ) | ( (x & 0xff0000) >> 8 ) | ( (x & 0xff000000) >> 24 ) )

void VenusFixLocationEndianess()   // we only do relevant ones to save CPU time
{
    venus_ctx.message.location.latitude = SWAP32(venus_ctx.message.location.latitude);
    venus_ctx.message.location.longitude = SWAP32(venus_ctx.message.location.longitude);
    venus_ctx.message.location.sealevel_alt = SWAP32(venus_ctx.message.location.sealevel_alt);
}

void venusWrite(uint8_t length)
{
    uint8_t pls;
    uint8_t cs = 0;

    SerialWrite(GPS_SERIAL,   0xA0);
    SerialWrite(GPS_SERIAL,   0xA1);
    SerialWrite(GPS_SERIAL,      0); // length is never higher than 8 bits
    SerialWrite(GPS_SERIAL, length);

    for (pls = 0; pls < length; pls++)
    {
        cs = cs ^ venus_ctx.payload[pls];
        SerialWrite(GPS_SERIAL, venus_ctx.payload[pls]);
    }

    SerialWrite(GPS_SERIAL,   cs);
    SerialWrite(GPS_SERIAL, 0x0D);
    SerialWrite(GPS_SERIAL, 0x0A);
    delay(50);
}

void GPS_SerialInit(void)
{
    uint32_t init_speed[5] = {9600, 19200, 38400, 115200, 57600};

    for (uint8_t i = 0; i < 5; i++)
    {
        SerialOpen(GPS_SERIAL, init_speed[i]);
        venus_ctx.message.id = VENUS_CONFIG_OUTPUT_MSG_FORMAT;
        venus_ctx.message.body[0] = 2; // VENUS BINARY
        venus_ctx.message.body[1] = 0; // SRAM
        venusWrite(3);                 // message payload length = 3
        venus_ctx.message.id = VENUS_CONFIG_SERIAL_PORT;
        venus_ctx.message.body[0] = 0; // Venus device's COM1
        venus_ctx.message.body[1] = 4; // 57600
        venus_ctx.message.body[2] = 0; // SRAM
        venusWrite(4);
    }

    delay(200);
    // configure NAV MODE
    venus_ctx.message.id = VENUS8_EXT4;
    venus_ctx.message.body[0] = VENUS8_CONFIG_NAV_MODE;
    venus_ctx.message.body[1] = 5; // NAVMODE AUTO
    venus_ctx.message.body[2] = 0; // SRAM
    venusWrite(4);
    // configure INTERFERENCE DETECTION
    venus_ctx.message.id = VENUS8_EXT4;
    venus_ctx.message.body[0] = VENUS8_CONFIG_INTERFERENCE_DETECT;
    venus_ctx.message.body[1] = 1; // enable
    venus_ctx.message.body[2] = 0; // SRAM
    venusWrite(4);
    // configure INTERFERENCE DETECTION
    venus_ctx.message.id = VENUS8_EXT2;
    venus_ctx.message.body[0] = VENUS8_CONFIG_SBAS;
    venus_ctx.message.body[1] = 1; // SBAS enable
    venus_ctx.message.body[2] = 1; // use SBAS satellite for navigation
    venus_ctx.message.body[3] = 8; // default range
    venus_ctx.message.body[4] = 1; // enable the correction
    venus_ctx.message.body[5] = 3; // number of tracking channels
    venus_ctx.message.body[6] = 7; // Allows WAAS / EGNOS / MSAS
    venus_ctx.message.body[7] = 0; // SRAM
    venusWrite(9);
    // disable POSITION PINNING
    venus_ctx.message.id = VENUS_CONFIG_GPS_PINNING;
    venus_ctx.message.body[0] = 2; // POSPINNING DISABLE
    venus_ctx.message.body[1] = 0; // SRAM
    venusWrite(3);
    // disable POSITION PINNING
    venus_ctx.message.id = VENUS_CONFIG_GPS_PINNING_PARAMS;

    for (uint8_t i = 0; i < 10; i++)
    {
        venus_ctx.message.body[i] = 0;
    }

    venus_ctx.message.body[10] = 0; // SRAM
    venusWrite(12);
    // disable SAGPS
    venus_ctx.message.id = VENUS8_EXT3;
    venus_ctx.message.body[0] = VENUS8_CONFIG_SAGPS;
    venus_ctx.message.body[1] = 2; // SAGPS disable
    venus_ctx.message.body[2] = 0; // SRAM
    venusWrite(4);
    // NORMAL power mode
    venus_ctx.message.id = VENUS_CONFIG_POWER_MODE;
    venus_ctx.message.body[0] = 0; // POWERMODE NORMAL
    venus_ctx.message.body[1] = 0; // SRAM
    venusWrite(3);
    // enable the update rate
    venus_ctx.message.id = VENUS_CONFIG_GPS_UPDATE_RATE;
    venus_ctx.message.body[0] = 5; // VENUS UPDATE RATE
    venus_ctx.message.body[1] = 0; // SRAM
    venusWrite(3);
}

bool GPS_newFrame(uint8_t c)
{
    static uint8_t state = 0;
    static uint8_t n = 0;
    static uint8_t cr = 0;
    static uint8_t length; // payload length
    bool ret = false;

    switch (state)
    {
        case 0:
        {
            if (c == 0xA0)
            {
                state++;
            }

            break;
        }

        case 1:
        {
            if (c == 0xA1)
            {
                state++;
            }
            else
            {
                state = 0;
            }

            break;
        }

        case 2:
        {
            state++;
            break;  // first byte of length is always 0 because message payload is never higher than 255
        }

        case 3:
        {
            length = c;
            state++;
            break; // according to the spec, length is always >1
        }

        case 4:
        {
            venus_ctx.message.id = c;
            cr = c;
            n = 1;
            state++;
            break;
        }

        case 5: // read bytes of the payload
        {
            cr ^= c;  // adjust checksum

            if (n < sizeof(venus_ctx.message) )
            {
                venus_ctx.payload[n] = c;
            }

            n++;

            if (n == length)
            {
                state++;
            }

            break;
        }

        case 6:
        {
            if (c == cr)
            {
                state++;
            }
            else
            {
                state = 0;    // bad cr
            }

            break; // check checksum, abort if not-equal
        }

        case 7:
        {
            if (c == 0x0D)
            {
                state++;
            }
            else
            {
                state = 0;
            }

            break;
        }

        case 8:
            state = 0;

            if (c != 0x0A)
            {
                break;
            }

            if (venus_ctx.message.id == VENUS_GPS_LOCATION)
            {
                GPS_numSat = venus_ctx.message.location.sv_count;
                f.GPS_FIX = venus_ctx.message.location.fixmode >= 2;

                if (f.GPS_FIX)
                {
                    VenusFixLocationEndianess();
                    GPS_coord[LAT] = venus_ctx.message.location.latitude;
                    GPS_coord[LON] = venus_ctx.message.location.longitude;
                    GPS_altitude = venus_ctx.message.location.sealevel_alt / 100;      // altitude in meter
                }

                ret = true;
            }
    }

    return(ret);
}
