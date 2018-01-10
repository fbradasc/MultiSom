#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Barometer MS561101BA
// ************************************************************************************************************
//
// specs are here: http://www.meas-spec.com/downloads/MS5611-01BA03.pdf
// useful info on pages 7 -> 12
# if !defined(MS561101BA_ADDRESS)
#  define MS561101BA_ADDRESS 0x77 //CBR=0 0xEE I2C address when pin CSB is connected to LOW (GND)
//#define MS561101BA_ADDRESS 0x76 //CBR=1 0xEC I2C address when pin CSB is connected to HIGH (VCC)
# endif

// registers of the device
# define MS561101BA_PRESSURE    0x40
# define MS561101BA_TEMPERATURE 0x50
# define MS561101BA_RESET       0x1E

// OSR (Over Sampling Ratio) constants
# define MS561101BA_OSR_256  0x00
# define MS561101BA_OSR_512  0x02
# define MS561101BA_OSR_1024 0x04
# define MS561101BA_OSR_2048 0x06
# define MS561101BA_OSR_4096 0x08

# define OSR MS561101BA_OSR_4096

static struct
{
    // sensor registers from the MS561101BA datasheet
    uint16_t c[7];
    uint32_t ut;            //uncompensated T
    uint32_t up;            //uncompensated P
    uint8_t state;
    uint16_t deadline;
}
ms561101ba_ctx;

static void Baro_init()
{
    //reset
    i2c_writeReg(MS561101BA_ADDRESS, MS561101BA_RESET, 0);
    delay(100);
    //read calibration data
    union
    {
        uint16_t val;
        uint8_t raw[2];
    }
    data;

    for (uint8_t i = 0; i < 6; i++)
    {
        i2c_rep_start(MS561101BA_ADDRESS << 1);
        i2c_write(0xA2 + 2 * i);
        i2c_rep_start( (MS561101BA_ADDRESS << 1) | 1 );   //I2C read direction => 1
        data.raw[1] = i2c_readAck();    // read a 16 bit register
        data.raw[0] = i2c_readNak();
        ms561101ba_ctx.c[i + 1] = data.val;
    }
}

// read uncompensated temperature or pressure value: send command first
static void i2c_MS561101BA_UT_or_UP_Start(uint8_t reg)
{
    i2c_rep_start(MS561101BA_ADDRESS << 1);     // I2C write direction
    i2c_write(reg);         // register selection
    i2c_stop();
}

static void i2c_MS561101BA_UT_or_UP_Read(uint32_t *val)
{
    union
    {
        uint32_t val;
        uint8_t raw[4];
    }
    data;
    i2c_rep_start(MS561101BA_ADDRESS << 1);
    i2c_write(0);
    i2c_rep_start( (MS561101BA_ADDRESS << 1) | 1 );
    data.raw[2] = i2c_readAck();
    data.raw[1] = i2c_readAck();
    data.raw[0] = i2c_readNak();
    *val = data.val;
}

// use float approximation instead of int64_t intermediate values
// does not use 2nd order compensation under -15 deg
static void i2c_MS561101BA_Calculate()
{
    int32_t delt;
    float dT =
        (int32_t) ms561101ba_ctx.ut -
        (int32_t)( (uint32_t) ms561101ba_ctx.c[5] << 8 );
    float off =
        ( (uint32_t) ms561101ba_ctx.c[2] << 16 ) +
        ( (dT * ms561101ba_ctx.c[4]) / ( (uint32_t) 1 << 7 ) );
    float sens =
        ( (uint32_t) ms561101ba_ctx.c[1] << 15 ) +
        ( (dT * ms561101ba_ctx.c[3]) / ( (uint32_t) 1 << 8 ) );

    delt = (dT * ms561101ba_ctx.c[6]) / ( (uint32_t) 1 << 23 );
    baroTemperature = delt + 2000;

    if (delt < 0)
    {
        // temperature lower than 20st.C
        delt *= 5 * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
    }

    baroPressure =
        ( ( (ms561101ba_ctx.up * sens) / ( (uint32_t) 1 << 21 ) ) -
          off ) / ( (uint32_t) 1 << 15 );
}

//return 0: no data available, no computation ;  1: new value available   or   no new value but computation time
uint8_t Baro_update()
{
    // first UT conversion is started in init procedure
    uint32_t *rawValPointer;
    uint8_t commandRegister;

    if (ms561101ba_ctx.state == 2)
    {
        // a third state is introduced here to isolate calculate() and smooth timecycle spike
        ms561101ba_ctx.state = 0;
        i2c_MS561101BA_Calculate();
        return(1);
    }

    if ( (int16_t)(currentTime - ms561101ba_ctx.deadline) < 0 )
    {
        return(0);    // the initial timer is not initialized, but in any case, no more than 65ms to wait.
    }

    ms561101ba_ctx.deadline = currentTime + 10000;  // UT and UP conversion take 8.5ms so we do next reading after 10ms

    if (ms561101ba_ctx.state == 0)
    {
        Baro_Common();      // moved here for less timecycle spike, goes after i2c_MS561101BA_Calculate
        rawValPointer = &ms561101ba_ctx.ut;
        commandRegister = MS561101BA_PRESSURE + OSR;
    }
    else
    {
        rawValPointer = &ms561101ba_ctx.up;
        commandRegister = MS561101BA_TEMPERATURE + OSR;
    }

    ms561101ba_ctx.state++;
    i2c_MS561101BA_UT_or_UP_Read(rawValPointer);    // get the 24bit resulting from a UP of UT command request. Nothing interresting for the first cycle because we don't initiate a command in Baro_init()
    i2c_MS561101BA_UT_or_UP_Start(commandRegister);     // send the next command to get UP or UT value after at least 8.5ms
    return(1);
}

#else // IMPLEMENTATION
#endif // IMPLEMENTATION
