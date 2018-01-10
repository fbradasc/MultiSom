// #include <SPI.h>
#include <SdFat.h>
SdFat sd;
SdFile gps_data;
SdFile permanent;

void init_SD()
{
    if (!sd.begin(SDCARD_CSPIN, SPI_HALF_SPEED) )
    {
        f.SDCARD = 0;       // If init fails, tell the code not to try to write on it
        debug[1] = 999;
    }
    else
    {
        f.SDCARD = 1;
        debug[1] = 000;
    }
}

// time stamp logging might also work for MTK in binary mode if you adjust this ifdef
// as well as the next ifdef UBLOX, but no promises
#ifdef UBLOX
void writeGPSLog(uint32_t gpstime, int32_t latitude, int32_t longitude, int32_t altitude)
{
    (void) gpstime;
#else
void writeGPSLog(int32_t latitude, int32_t longitude, int32_t altitude)
{
#endif
    (void) latitude;
    (void) longitude;
    (void) altitude;

    if (f.SDCARD == 0)
    {
        return;
    }

    if (gps_data.open(GPS_LOG_FILENAME, O_WRITE | O_CREAT | O_APPEND) )
    {
#ifdef UBLOX
        gps_data.print(gpstime);
        gps_data.write(',');
#endif
        gps_data.print(latitude);
        gps_data.write(',');
        gps_data.print(longitude);
        gps_data.write(',');
        gps_data.print(altitude);
        gps_data.println();
        gps_data.close();
    }
    else
    {
        return;
    }
}

void writePLogToSD()
{
    if (f.SDCARD == 0)
    {
        return;
    }

    plog.checksum = calculate_sum( (uint8_t *) &plog, sizeof(plog) );

    if (permanent.open(PERMANENT_LOG_FILENAME, O_WRITE | O_CREAT | O_TRUNC) )
    {
        permanent.print(F("arm=") );
        permanent.println(plog.arm);
        permanent.print(F("disarm=") );
        permanent.println(plog.disarm);
        permanent.print(F("start=") );
        permanent.println(plog.start);
        permanent.print(F("armed_time=") );
        permanent.println(plog.armed_time);
        permanent.print(F("lifetime=") );
        permanent.println(plog.lifetime, DEC);
        permanent.print(F("failsafe=") );
        permanent.println(plog.failsafe);
        permanent.print(F("i2c=") );
        permanent.println(plog.i2c);
        permanent.print(F("running=") );
        permanent.println(plog.running, DEC);
        permanent.print(F("checksum=") );
        permanent.println(plog.checksum, DEC);
        permanent.print(F("debug=") );
        permanent.print(debug[0]);
        permanent.print(F(",") );
        permanent.print(debug[1]);
        permanent.print(F(",") );
        permanent.print(debug[2]);
        permanent.print(F(",") );
        permanent.println(debug[3]);
        permanent.println();
        permanent.close();
    }
    else
    {
        return;
    }
}

void fillPlogStruct(char *key, char *value)
{
    if (strcmp(key, "arm") == 0)
    {
        sscanf(value, "%u", &plog.arm);
    }

    if (strcmp(key, "disarm") == 0)
    {
        sscanf(value, "%u", &plog.disarm);
    }

    if (strcmp(key, "start") == 0)
    {
        sscanf(value, "%u", &plog.start);
    }

    if (strcmp(key, "armed_time") == 0)
    {
        sscanf(value, "%lu", &plog.armed_time);
    }

    if (strcmp(key, "lifetime") == 0)
    {
        sscanf(value, "%lu", &plog.lifetime);
    }

    if (strcmp(key, "failsafe") == 0)
    {
        sscanf(value, "%u", &plog.failsafe);
    }

    if (strcmp(key, "i2c") == 0)
    {
        sscanf(value, "%u", &plog.i2c);
    }

    if (strcmp(key, "running") == 0)
    {
        sscanf(value, "%hhu", &plog.running);
    }

    if (strcmp(key, "checksum") == 0)
    {
        sscanf(value, "%hhu", &plog.checksum);
    }
}

void readPLogFromSD()
{
    if (f.SDCARD == 0)
    {
        return;
    }

    char key[12];
    char value[32];
    char *tabPtr = key;
    int c;
    uint8_t i = 0;
    SdFile myfile;

    if (myfile.open(PERMANENT_LOG_FILENAME, O_READ) )
    {
        while (myfile.available() )
        {
            c = myfile.read();

            switch ( (char) c )
            {
                case ' ':
                {
                    break;
                }

                case '=':
                {
                    *tabPtr = '\0';
                    tabPtr = value;
                    break;
                }

                case '\n':
                {
                    *tabPtr = '\0';
                    tabPtr = key;
                    i = 0;
                    fillPlogStruct(key, value);
                    memset(key,   '\0', sizeof(key) );
                    memset(value, '\0', sizeof(value) );
                    break;
                }

                default:
                {
                    i++;

                    if (i <= 12)
                    {
                        *tabPtr = (char) c;
                        tabPtr++;
                    }

                    break;
                }
            }
        }
    }
    else
    {
        return;
    }

    if (calculate_sum( (uint8_t *) &plog, sizeof(plog) ) != plog.checksum)
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
