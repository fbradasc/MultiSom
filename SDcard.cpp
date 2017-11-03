/* 
 *  SDcard logging support
 *  based on wareck's Multiwii_Nav_V2.3 https://github.com/wareck/Multiwii_Nav_V2.3
 *  Updated for PatrikE's Multiwii 2.4/FixedWing fork by Martin Espinoza <martin.espinoza@gmail.com>
 */

/* thanks to Rcnet team for there job !*/

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"

#ifdef MWI_SDCARD

#include "EEPROM.h"
#include "SDcard.h"
#include "MultiSom.h"
#include "Alarms.h"
#include "GPS.h"
#define PERMANENT_LOG_FILENAME "PERM.TXT"
#define GPS_LOG_FILENAME "GPS_DATA.RAW"

#define PETIT_FS

#if defined(PETIT_FS)
#include <PetitFS.h>
#include <PetitSerial.h>

PetitSerial ps;
FATFS fs;

const uint16_t __one__            = 1;
const bool     isCpuLittleEndian  = ( 1 == *(char*)(&__one__) ); // CPU endianness
const bool     isFileLittleEndian = false;  // output endianness - you choose :)

#else
// #include <SPI.h>
#include <SdFat.h>
SdFat sd;
SdFile gps_data;
SdFile permanent;
#endif

void init_SD(){
#if defined(PETIT_FS)
  ps.begin(9600);
  ps.print("Initializing PetitFS...");

  if (pf_mount(&fs))
#else
  if (!sd.begin(SDCARD_CSPIN, SPI_HALF_SPEED))
#endif
  {
#if defined(PETIT_FS)
    ps.println(" success");
#endif

    f.SDCARD = 0; // If init fails, tell the code not to try to write on it
    debug[1] = 999;
  }
  else {
#if defined(PETIT_FS)
    ps.println(" fail");
#endif

    f.SDCARD = 1;
    debug[1] = 000;
  }
}

#if defined(PETIT_FS)
// type:
//    b: int8_t  (uppercase for unsigned)
//    w: int16_t (uppercase for unsigned)
//    i: int32_t (uppercase for unsigned)
//    l: int64_t (uppercase for unsigned)
//    f: float
//    d: double
//

bool pf_write_values(const uint8_t *val, uint32_t count=1)
{
    UINT bw;

    bool fail=false;

    if (NULL != val)
    {
        for (uint32_t i=0; !fail && i<count; i++)
        {
            fail = !pf_write((const void *)&val[i], sizeof(uint8_t), &bw) && (1==bw);
        }
    }

    return fail;
}

bool pf_write_values(const uint16_t *val, uint32_t count=1)
{
    bool fail=false;

    if (NULL != val)
    {
        for (uint32_t i=0; !fail && i<count; i++)
        {
            const uint8_t word[2] =
            {
                (uint8_t)((val[i]>>0) & 0x00ff),
                (uint8_t)((val[i]>>8) & 0x00ff)
            };

            fail = !pf_write_values(word,2);
        }
    }

    return fail;
}

bool pf_write_values(const uint32_t *val, uint32_t count=1)
{
    bool fail=false;

    if (NULL != val)
    {
        for (uint32_t i=0; !fail && i<count; i++)
        {
            const uint8_t dword[4] =
            {
                (uint8_t)((val[i]>> 0) & 0x00ff),
                (uint8_t)((val[i]>> 8) & 0x00ff),
                (uint8_t)((val[i]>>16) & 0x00ff),
                (uint8_t)((val[i]>>24) & 0x00ff)
            };

            fail = !pf_write_values(dword,4);
        }
    }

    return fail;
}

bool pf_read_values(uint8_t *val, const uint32_t count=1)
{
    UINT bw;

    bool fail=false;

    if (NULL != val)
    {
        for (uint32_t i=0; !fail && i<count; i++)
        {
            fail = !pf_read((void *)&val[i], sizeof(uint8_t), &bw) && (1==bw);
        }
    }

    return fail;
}

bool pf_read_values(uint16_t *val, const uint32_t count=1)
{
    bool fail=false;

    if (NULL != val)
    {
        for (uint32_t i=0; !fail && i<count; i++)
        {
            uint8_t word[2];

            if (!(fail = !pf_read_values(word,2)))
            {
                val[i] = ( (uint16_t)word[1] << 8 ) |
                         ( (uint16_t)word[0] << 0 );
            }
        }
    }

    return fail;
}

bool pf_read_values(uint32_t *val, const uint32_t count=1)
{
    bool fail=false;

    if (NULL != val)
    {
        for (uint32_t i=0; !fail && i<count; i++)
        {
            uint8_t dword[4];

            if (!(fail = !pf_read_values(dword,4)))
            {
                val[i] = ( (uint32_t)dword[3] << 24 ) |
                         ( (uint32_t)dword[2] << 16 ) |
                         ( (uint32_t)dword[1] <<  8 ) |
                         ( (uint32_t)dword[0] <<  0 );
            }
        }
    }

    return fail;
}

#define __pf_write_tlv__(t,v,c) !pf_write_values(t) && \
                                !pf_write_values(&c) && \
                                !pf_write_values(v, c)

bool pf_write_tlv(const uint8_t tag, const uint8_t *value, const uint16_t count=1)
{
    return __pf_write_tlv__(&tag,value,count);
}

bool pf_write_tlv(const uint8_t tag, const uint16_t *value, const uint16_t count=1)
{
    return __pf_write_tlv__(&tag,value,count);
}

bool pf_write_tlv(const uint8_t tag, const uint32_t *value, const uint16_t count=1)
{
    return __pf_write_tlv__(&tag,value,count);
}

bool pf_write_tlv(const uint8_t tag, const uint8_t value)
{
    return __pf_write_tlv__(&tag,&value,__one__);
}

bool pf_write_tlv(const uint8_t tag, const uint16_t value)
{
    return __pf_write_tlv__(&tag,&value,__one__);
}

bool pf_write_tlv(const uint8_t tag, const uint32_t value)
{
    return __pf_write_tlv__(&tag,&value,__one__);
}

bool pf_write_tlv(const uint8_t tag, const int8_t *value, const uint16_t count=1)
{
    return __pf_write_tlv__(&tag,(const uint8_t*)value,count);
}

bool pf_write_tlv(const uint8_t tag, const int16_t *value, const uint16_t count=1)
{
    return __pf_write_tlv__(&tag,(const uint16_t*)value,count);
}

bool pf_write_tlv(const uint8_t tag, const int32_t *value, const uint16_t count=1)
{
    return __pf_write_tlv__(&tag,(const uint32_t*)value,count);
}

bool pf_write_tlv(const uint8_t tag, const int8_t value)
{
    const uint16_t count = 1;
    return __pf_write_tlv__(&tag,(const uint8_t*)&value,count);
}

bool pf_write_tlv(const uint8_t tag, const int16_t value)
{
    const uint16_t count = 1;
    return __pf_write_tlv__(&tag,(const uint16_t*)&value,count);
}

bool pf_write_tlv(const uint8_t tag, const int32_t value)
{
    const uint16_t count = 1;
    return __pf_write_tlv__(&tag,(const uint32_t*)&value,count);
}

#endif

// time stamp logging might also work for MTK in binary mode if you adjust this ifdef
// as well as the next ifdef UBLOX, but no promises
#ifdef UBLOX
void writeGPSLog(uint32_t gpstime, int32_t latitude, int32_t longitude, int32_t altitude) {
  (void)gpstime;
#else
void writeGPSLog(int32_t latitude, int32_t longitude, int32_t altitude) {
#endif
  (void)latitude;
  (void)longitude;
  (void)altitude;
if (f.SDCARD == 0) return;
#if defined(PETIT_FS)
  ps.print("Appending to: ");
  ps.println(GPS_LOG_FILENAME);

  if (!pf_open(GPS_LOG_FILENAME) && !pf_lseek((DWORD)(-1))) {
#ifdef UBLOX
    pf_write_tlv('t',gpstime  );
#endif
    pf_write_tlv('x',longitude);
    pf_write_tlv('y',latitude );
    pf_write_tlv('z',altitude );
#else
  if (gps_data.open(GPS_LOG_FILENAME, O_WRITE | O_CREAT | O_APPEND)) {
#ifdef UBLOX
    gps_data.print(gpstime); gps_data.write(',');
#endif
    gps_data.print(latitude); gps_data.write(',');
    gps_data.print(longitude); gps_data.write(',');
    gps_data.print(altitude); gps_data.println();
    gps_data.close();
#endif
  } else {
    return;
  }
}

void writePLogToSD() {
  if (f.SDCARD == 0) return;
  plog.checksum = calculate_sum((uint8_t*)&plog, sizeof(plog));
#if defined(PETIT_FS)
  ps.print("Writing into: ");
  ps.println(PERMANENT_LOG_FILENAME);

  if (!pf_open(PERMANENT_LOG_FILENAME)) {
    pf_write_tlv('a', plog.arm       );
    pf_write_tlv('d', plog.disarm    );
    pf_write_tlv('s', plog.start     );
    pf_write_tlv('t', plog.armed_time);
    pf_write_tlv('l', plog.lifetime  );
    pf_write_tlv('f', plog.failsafe  );
    pf_write_tlv('i', plog.i2c       );
    pf_write_tlv('r', plog.running   );
    pf_write_tlv('c', plog.checksum  );
    pf_write_tlv('v', debug          , 4);
#else
  if (permanent.open(PERMANENT_LOG_FILENAME, O_WRITE | O_CREAT | O_TRUNC)) {
    permanent.print(F("arm=")); permanent.println(plog.arm);
    permanent.print(F("disarm=")); permanent.println(plog.disarm);
    permanent.print(F("start=")); permanent.println(plog.start);
    permanent.print(F("armed_time=")); permanent.println(plog.armed_time);
    permanent.print(F("lifetime=")); permanent.println(plog.lifetime, DEC);
    permanent.print(F("failsafe=")); permanent.println(plog.failsafe);
    permanent.print(F("i2c=")); permanent.println(plog.i2c);
    permanent.print(F("running=")); permanent.println(plog.running, DEC);
    permanent.print(F("checksum=")); permanent.println(plog.checksum, DEC);
    permanent.print(F("debug=")); permanent.print(debug[0]);
    permanent.print(F(",")); permanent.print(debug[1]);
    permanent.print(F(",")); permanent.print(debug[2]);
    permanent.print(F(",")); permanent.println(debug[3]);
    permanent.println();
    permanent.close();
#endif
  } else {
    return;
  }
}

#if defined(PETIT_FS)
void readPLogFromSD() {
  if (f.SDCARD == 0) return;

  ps.print("Reading from: ");
  ps.println(PERMANENT_LOG_FILENAME);

  if (!pf_open(PERMANENT_LOG_FILENAME)) {
      uint8_t  byte;

      if (!pf_read_values(&byte))
      {
          switch (byte)
          {
              case 'a': pf_read_values(&plog.arm       ); break;
              case 'd': pf_read_values(&plog.disarm    ); break;
              case 's': pf_read_values(&plog.start     ); break;
              case 't': pf_read_values(&plog.armed_time); break;
              case 'l': pf_read_values(&plog.lifetime  ); break;
              case 'f': pf_read_values(&plog.failsafe  ); break;
              case 'i': pf_read_values(&plog.i2c       ); break;
              case 'r': pf_read_values(&plog.running   ); break;
              case 'c': pf_read_values(&plog.checksum  ); break;
              case 'v': pf_read_values(debug           ); break;
          }
      }
  } else return;
  if (calculate_sum((uint8_t*)&plog, sizeof(plog)) != plog.checksum) {
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
#else
void fillPlogStruct(char* key, char* value) {
  if (strcmp(key, "arm") == 0) sscanf(value, "%u", &plog.arm);
  if (strcmp(key, "disarm") == 0) sscanf(value, "%u", &plog.disarm);
  if (strcmp(key, "start") == 0) sscanf(value, "%u", &plog.start);
  if (strcmp(key, "armed_time") == 0) sscanf(value, "%lu", &plog.armed_time);
  if (strcmp(key, "lifetime") == 0) sscanf(value, "%lu", &plog.lifetime);
  if (strcmp(key, "failsafe") == 0) sscanf(value, "%u", &plog.failsafe);
  if (strcmp(key, "i2c") == 0) sscanf(value, "%u", &plog.i2c);
  if (strcmp(key, "running") == 0) sscanf(value, "%hhu", &plog.running);
  if (strcmp(key, "checksum") == 0) sscanf(value, "%hhu", &plog.checksum);
}

void readPLogFromSD() {
  if (f.SDCARD == 0) return;
  char key[12];
  char value[32];
  char* tabPtr = key;
  int c;
  uint8_t i = 0;
  SdFile myfile;

  if (myfile.open(PERMANENT_LOG_FILENAME, O_READ)) {
    while (myfile.available()) {
      c = myfile.read();
      switch ((char)c) {
        case ' ':
          break;
        case '=':
          *tabPtr = '\0';
          tabPtr = value;
          break;  
        case '\n':
          *tabPtr = '\0';
          tabPtr = key;
          i = 0;
          fillPlogStruct(key, value);
          memset(key, '\0', sizeof(key));
          memset(value, '\0', sizeof(value));
          break;
        default:
          i++;
          if (i <= 12) {
            *tabPtr = (char)c;
            tabPtr++;
          }
          break;
        }
    }
  } else return;
  if (calculate_sum((uint8_t*)&plog, sizeof(plog)) != plog.checksum) {
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
#endif
#endif
