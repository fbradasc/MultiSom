#include "../../src/PetitFS/PetitFS.h"
#include "../../src/PetitFS/PetitSerial.h"

#if defined(DEBUG_PETIT_FS)
    #if defined(USE_PETIT_SERIAL)
        PetitSerial ps;
        #define LOG_INIT(p)    ps.print(p)
        #define LOG_PRINT(t)   ps.print(t)
        #define LOG_PRINTLN(t) ps.println(t)
    #else // USE_PETIT_SERIAL
        #define LOG_INIT(p)
        #define LOG_PRINT(t)   LCDprintChar(t)
        #define LOG_PRINTLN(t) LCDprintChar(t); LCDcrlf();
    #endif // USE_PETIT_SERIAL
#else // DEBUG_PETIT_FS
    #define LOG_INIT(p)
    #define LOG_PRINT(t)
    #define LOG_PRINTLN(t)
#endif // DEBUG_PETIT_FS

FATFS fs;

const uint16_t __one__ = 1;

#if defined(CHECK_ENDIANNESS)
    const bool isCpuLittleEndian  = (1 == *(char *)(&__one__));     // CPU endianness
    const bool isFileLittleEndian = false;  // output endianness - you choose :)
#endif // CHECK_ENDIANNESS

void
init_SD()
{
    // LOG_INIT(9600);
    LOG_PRINTLN("     Initializing PetitFS...");

    if (pf_mount(&fs))
    {
        LOG_PRINTLN(" success");
        f.SDCARD = 0;       // If init fails, tell the code not to try to write on it
        debug[1] = 999;
    }
    else
    {
        LOG_PRINTLN(" fail");
        f.SDCARD = 1;
        debug[1] = 000;
    }

    LOG_PRINTLN("done");
}

// type:
//    b: int8_t  (uppercase for unsigned)
//    w: int16_t (uppercase for unsigned)
//    i: int32_t (uppercase for unsigned)
//    l: int64_t (uppercase for unsigned)
//    f: float
//    d: double
//

bool
pf_write_values(const uint8_t *val, uint32_t count = 1)
{
    UINT bw;
    bool fail = false;

    if (NULL != val)
    {
        for (uint32_t i = 0; !fail && i < count; i++)
        {
            fail = !pf_write((const void *) &val[i], sizeof(uint8_t), &bw)
                   && (1 == bw);
        }
    }

    return fail;
}

bool
pf_write_values(const uint16_t *val, uint32_t count = 1)
{
    bool fail = false;

    if (NULL != val)
    {
        for (uint32_t i = 0; !fail && i < count; i++)
        {
            const uint8_t word[2] =
            {
                (uint8_t)((val[i] >> 0) & 0x00ff),
                (uint8_t)((val[i] >> 8) & 0x00ff)
            };
            fail = !pf_write_values(word, 2);
        }
    }

    return fail;
}

bool
pf_write_values(const uint32_t *val, uint32_t count = 1)
{
    bool fail = false;

    if (NULL != val)
    {
        for (uint32_t i = 0; !fail && i < count; i++)
        {
            const uint8_t dword[4] =
            {
                (uint8_t)((val[i] >> 0) & 0x00ff),
                (uint8_t)((val[i] >> 8) & 0x00ff),
                (uint8_t)((val[i] >> 16) & 0x00ff),
                (uint8_t)((val[i] >> 24) & 0x00ff)
            };
            fail = !pf_write_values(dword, 4);
        }
    }

    return fail;
}

bool
pf_read_values(uint8_t *val, const uint32_t count = 1)
{
    UINT bw;
    bool fail = false;

    if (NULL != val)
    {
        for (uint32_t i = 0; !fail && i < count; i++)
        {
            fail = !pf_read((void *) &val[i], sizeof(uint8_t), &bw)
                   && (1 == bw);
        }
    }

    return fail;
}

bool
pf_read_values(uint16_t *val, const uint32_t count = 1)
{
    bool fail = false;

    if (NULL != val)
    {
        for (uint32_t i = 0; !fail && i < count; i++)
        {
            uint8_t word[2];

            if (!(fail = !pf_read_values(word, 2)))
            {
                val[i] = ((uint16_t) word[1] << 8) | ((uint16_t) word[0] << 0);
            }
        }
    }

    return fail;
}

bool
pf_read_values(uint32_t *val, const uint32_t count = 1)
{
    bool fail = false;

    if (NULL != val)
    {
        for (uint32_t i = 0; !fail && i < count; i++)
        {
            uint8_t dword[4];

            if (!(fail = !pf_read_values(dword, 4)))
            {
                val[i] = ((uint32_t) dword[3] << 24) |
                         ((uint32_t) dword[2] << 16) |
                         ((uint32_t) dword[1] << 8) | ((uint32_t) dword[0] << 0);
            }
        }
    }

    return fail;
}

#define __pf_write_tlv__(t,v,c) !pf_write_values(t) && \
    !pf_write_values(&c) && \
    !pf_write_values(v, c)

bool
pf_write_tlv(const uint8_t tag, const uint8_t *value, const uint16_t count = 1)
{
    return __pf_write_tlv__(&tag, value, count);
}

bool
pf_write_tlv(const uint8_t tag, const uint16_t *value, const uint16_t count = 1)
{
    return __pf_write_tlv__(&tag, value, count);
}

bool
pf_write_tlv(const uint8_t tag, const uint32_t *value, const uint16_t count = 1)
{
    return __pf_write_tlv__(&tag, value, count);
}

bool
pf_write_tlv(const uint8_t tag, const uint8_t value)
{
    return __pf_write_tlv__(&tag, &value, __one__);
}

bool
pf_write_tlv(const uint8_t tag, const uint16_t value)
{
    return __pf_write_tlv__(&tag, &value, __one__);
}

bool
pf_write_tlv(const uint8_t tag, const uint32_t value)
{
    return __pf_write_tlv__(&tag, &value, __one__);
}

bool
pf_write_tlv(const uint8_t tag, const int8_t *value, const uint16_t count = 1)
{
    return __pf_write_tlv__(&tag, (const uint8_t *) value, count);
}

bool
pf_write_tlv(const uint8_t tag, const int16_t *value, const uint16_t count = 1)
{
    return __pf_write_tlv__(&tag, (const uint16_t *) value, count);
}

bool
pf_write_tlv(const uint8_t tag, const int32_t *value, const uint16_t count = 1)
{
    return __pf_write_tlv__(&tag, (const uint32_t *) value, count);
}

bool
pf_write_tlv(const uint8_t tag, const int8_t value)
{
    const uint16_t count = 1;
    return __pf_write_tlv__(&tag, (const uint8_t *) &value, count);
}

bool
pf_write_tlv(const uint8_t tag, const int16_t value)
{
    const uint16_t count = 1;
    return __pf_write_tlv__(&tag, (const uint16_t *) &value, count);
}

bool
pf_write_tlv(const uint8_t tag, const int32_t value)
{
    const uint16_t count = 1;
    return __pf_write_tlv__(&tag, (const uint32_t *) &value, count);
}

// time stamp logging might also work for MTK in binary mode if you adjust this ifdef
// as well as the next ifdef UBLOX, but no promises
#ifdef UBLOX
void
writeGPSLog(uint32_t gpstime, int32_t latitude, int32_t longitude, int32_t altitude)
{
    (void) gpstime;
#else
void
writeGPSLog(int32_t latitude, int32_t longitude, int32_t altitude)
{
#endif
    (void) latitude;
    (void) longitude;
    (void) altitude;

    if (f.SDCARD == 0)
    {
        return;
    }

    LOG_PRINT("Appending to: ");
    LOG_PRINTLN(GPS_LOG_FILENAME);

    if (!pf_open(GPS_LOG_FILENAME) && !pf_lseek((DWORD)(-1)))
    {
#ifdef UBLOX
        pf_write_tlv('t', gpstime);
#endif
        pf_write_tlv('x', longitude);
        pf_write_tlv('y', latitude);
        pf_write_tlv('z', altitude);
    }
    else
    {
        return;
    }
}

void
writePLogToSD()
{
    if (f.SDCARD == 0)
    {
        return;
    }

    plog.checksum = calculate_sum((uint8_t *) & plog, sizeof(plog));
    LOG_PRINT("Writing into: ");
    LOG_PRINTLN(PERMANENT_LOG_FILENAME);

    if (!pf_open(PERMANENT_LOG_FILENAME))
    {
        pf_write_tlv('a', plog.arm);
        pf_write_tlv('d', plog.disarm);
        pf_write_tlv('s', plog.start);
        pf_write_tlv('t', plog.armed_time);
        pf_write_tlv('l', plog.lifetime);
        pf_write_tlv('f', plog.failsafe);
        pf_write_tlv('i', plog.i2c);
        pf_write_tlv('r', plog.running);
        pf_write_tlv('c', plog.checksum);
        pf_write_tlv('v', debug, DEBUG_WORDS);
    }
    else
    {
        return;
    }
}

void
readPLogFromSD()
{
    if (f.SDCARD == 0)
    {
        return;
    }

    LOG_PRINT("Reading from: ");
    LOG_PRINTLN(PERMANENT_LOG_FILENAME);

    if (!pf_open(PERMANENT_LOG_FILENAME))
    {
        uint8_t byte;

        if (!pf_read_values(&byte))
        {
            switch (byte)
            {
                case 'a':
                    pf_read_values(&plog.arm);
                    break;

                case 'd':
                    pf_read_values(&plog.disarm);
                    break;

                case 's':
                    pf_read_values(&plog.start);
                    break;

                case 't':
                    pf_read_values(&plog.armed_time);
                    break;

                case 'l':
                    pf_read_values(&plog.lifetime);
                    break;

                case 'f':
                    pf_read_values(&plog.failsafe);
                    break;

                case 'i':
                    pf_read_values(&plog.i2c);
                    break;

                case 'r':
                    pf_read_values(&plog.running);
                    break;

                case 'c':
                    pf_read_values(&plog.checksum);
                    break;

                case 'v':
                    pf_read_values(debug, DEBUG_WORDS);
                    break;
            }
        }
    }
    else
    {
        return;
    }

    if (calculate_sum((uint8_t *) & plog, sizeof(plog)) != plog.checksum)
    {
#if defined(BUZZER)
        alarmArray[7] = 3;
        blinkLED(9, 100, 3);
#endif
        // force load defaults
        plog.arm = plog.disarm = plog.start = plog.failsafe = plog.i2c = 11;
        plog.running = 1;
        plog.lifetime = plog.armed_time = 3;
        writePLogToSD();
    }
}
