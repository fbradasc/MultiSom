#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiSom.h"
#include "Alarms.h"
#include "EEPROM.h"
#include "IMU.h"
#include "LCD.h"
#include "Sensors.h"

static void Device_Mag_getADC();
static void ACC_init();
#if defined(MAG)
static void Mag_init();
#endif
#if defined(BARO)
static void Baro_init();
#endif
#if defined(SONAR)
static void Sonar_init();
#endif
#if defined(PITOT)
static void Pitot_init();
#endif

// ************************************************************************************************************
// board orientation and setup
// ************************************************************************************************************
//default board orientation
#if !defined(ACC_ORIENTATION)
# define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL] = X;imu.accADC[PITCH] = Y;imu.accADC[YAW] = Z;}
#endif
#if !defined(GYRO_ORIENTATION)
# define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X;imu.gyroADC[PITCH] = Y;imu.gyroADC[YAW] = Z;}
#endif
#if !defined(MAG_ORIENTATION)
# define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL] = X;imu.magADC[PITCH] = Y;imu.magADC[YAW] = Z;}
#endif

//ITG3200 / ITG3205 / ITG3050 / MPU6050 / MPU3050 Gyro LPF setting
#if defined(GYRO_LPF_256HZ) || defined(GYRO_LPF_188HZ) || defined(GYRO_LPF_98HZ) || defined(GYRO_LPF_42HZ) || defined(GYRO_LPF_20HZ) || defined(GYRO_LPF_10HZ) || defined(GYRO_LPF_5HZ)
# if defined(GYRO_LPF_256HZ)
#  define GYRO_DLPF_CFG   0
# endif
# if defined(GYRO_LPF_188HZ)
#  define GYRO_DLPF_CFG   1
# endif
# if defined(GYRO_LPF_98HZ)
#  define GYRO_DLPF_CFG   2
# endif
# if defined(GYRO_LPF_42HZ)
#  define GYRO_DLPF_CFG   3
# endif
# if defined(GYRO_LPF_20HZ)
#  define GYRO_DLPF_CFG   4
# endif
# if defined(GYRO_LPF_10HZ)
#  define GYRO_DLPF_CFG   5
# endif
# if defined(GYRO_LPF_5HZ)
#  define GYRO_DLPF_CFG   6
# endif
#else
# define GYRO_DLPF_CFG   0  //Default settings LPF 256Hz/8000Hz sample
#endif

static uint8_t rawADC[6];
#if defined(USE_NEUTRALIZE_DELAY)
static uint32_t neutralizeTime = 0;
#endif

// ************************************************************************************************************
// I2C general functions
// ************************************************************************************************************

void i2c_init(void)
{
#if defined(INTERNAL_I2C_PULLUPS)
    I2C_PULLUPS_ENABLE
#else
    I2C_PULLUPS_DISABLE
#endif
    TWSR = 0;           // no prescaler => prescaler = 1
    TWBR = ( (F_CPU / 400000) - 16 ) / 2; // set the I2C clock rate to 400kHz
    TWCR = 1 << TWEN;       // enable twi module, no interrupt
    i2c_errors_count = 0;
}

void __attribute__( (noinline) ) waitTransmissionI2C(uint8_t twcr)
{
    TWCR = twcr;
    uint8_t count = 255;

    while (!(TWCR & (1 << TWINT) ) )
    {
        count--;

        if (count == 0)
        {
            //we are in a blocking state => we don't insist
            TWCR = 0;       //and we force a reset on TWINT register
#if defined(USE_NEUTRALIZE_DELAY)
            neutralizeTime = micros();  //we take a timestamp here to neutralize the value during a short delay
#endif
            i2c_errors_count++;
            break;
        }
    }
}

void i2c_rep_start(uint8_t address)
{
    waitTransmissionI2C( (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) );     // send REPEAT START condition and wait until transmission completed
    TWDR = address;     // send device address
    waitTransmissionI2C( (1 << TWINT) | (1 << TWEN) );    // wail until transmission completed
}

void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    //  while(TWCR & (1<<TWSTO));                // <- can produce a blocking state with some WMP clones
}

void i2c_write(uint8_t data)
{
    TWDR = data;            // send data to the previously addressed device
    waitTransmissionI2C( (1 << TWINT) | (1 << TWEN) );
}

uint8_t i2c_readAck()
{
    waitTransmissionI2C( (1 << TWINT) | (1 << TWEN) | (1 << TWEA) );
    return(TWDR);
}

uint8_t i2c_readNak()
{
    waitTransmissionI2C( (1 << TWINT) | (1 << TWEN) );
    uint8_t r = TWDR;
    i2c_stop();
    return(r);
}

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf, uint8_t size)
{
    i2c_rep_start(add << 1);    // I2C write direction
    i2c_write(reg);         // register selection
    i2c_rep_start( (add << 1) | 1 );  // I2C read direction
    uint8_t *b = buf;

    while (--size)
    {
        *b++ = i2c_readAck();    // acknowledge all but the final byte
    }

    *b = i2c_readNak();
}

void i2c_getSixRawADC(uint8_t add, uint8_t reg)
{
    i2c_read_reg_to_buf(add, reg, rawADC, 6);
}

void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val)
{
    i2c_rep_start(add << 1);    // I2C write direction
    i2c_write(reg);         // register selection
    i2c_write(val);         // value to write in register
    i2c_stop();
}

uint8_t i2c_readReg(uint8_t add, uint8_t reg)
{
    uint8_t val;

    i2c_read_reg_to_buf(add, reg, &val, 1);
    return(val);
}

#define IMPLEMENTATION
#include "drivers/sensors/sensors.hpp"
#undef IMPLEMENTATION

void initS()
{
    i2c_init();

    if (GYRO)
    {
        Gyro_init();
    }

#if defined(BARO)
    if (BARO)
    {
        Baro_init();
    }
#endif
#if defined(MAG)
    if (MAG)
    {
        Mag_init();
    }
#endif

    if (ACC)
    {
        ACC_init();
    }

#if defined(SONAR)
    if (SONAR)
    {
        Sonar_init();
    }
#endif
#if defined(PITOT)
    if (PITOT)
    {
        Pitot_init();
    }
#endif
}

void initSensors()
{
    uint8_t c = 5;

#if !defined(DISABLE_POWER_PIN)
    POWERPIN_ON;
    delay(200);
#endif

    while (c)
    {
        // We try several times to init all sensors without any i2c errors. An I2C error at this stage might results in a wrong sensor settings
        c--;
        initS();

        if (i2c_errors_count == 0)
        {
            break;    // no error during init => init ok
        }
    }
}
