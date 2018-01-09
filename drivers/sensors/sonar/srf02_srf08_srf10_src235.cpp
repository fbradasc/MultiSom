#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Sonar SRF08
// ************************************************************************************************************
// first contribution from guru_florida (02-25-2012)
//

// the default address for any new sensor found on the bus
// the code will move new sonars to the next available sonar address in range of F0-FE so that another
// sonar sensor can be added again.
// Thus, add only 1 sonar sensor at a time, poweroff, then wire the next, power on, wait for flashing light and repeat
#if !defined(SRF08_DEFAULT_ADDRESS)
    #define SRF08_DEFAULT_ADDRESS (0xE0>>1)
#endif

#if !defined(SRF08_RANGE_WAIT)
    #define SRF08_RANGE_WAIT      70000 // delay between Ping and Range Read commands (65ms is safe in any case)
#endif

#if !defined(SRF08_RANGE_SLEEP)
    #define SRF08_RANGE_SLEEP     5000  // sleep this long before starting another Ping
#endif

#if !defined(SRF08_SENSOR_FIRST)
    #define SRF08_SENSOR_FIRST    (0xF0>>1) // the first sensor i2c address (after it has been moved)
#endif

#if !defined(SRF08_MAX_SENSORS)
    #define SRF08_MAX_SENSORS     4 // maximum number of sensors we'll allow (can go up to 8)
#endif

//#define SONAR_MULTICAST_PING

// registers of the device
#define SRF08_REV_COMMAND    0
#define SRF08_LIGHT_GAIN     1
#define SRF08_ECHO_RANGE     2


static struct
{
    // sensor registers from the MS561101BA datasheet
    int32_t range[SRF08_MAX_SENSORS];
    int8_t sensors;     // the number of sensors present
    int8_t current;     // the current sensor being read
    uint8_t state;
    uint32_t deadline;
} srf08_ctx;


// read uncompensated temperature value: send command first
void
Sonar_init()
{
    memset(&srf08_ctx, 0, sizeof(srf08_ctx));
    srf08_ctx.deadline = 4000000;
}

// this function works like readReg accept a failed read is a normal expectation
// use for testing the existence of sensors on the i2c bus
// a 0xffff code is returned if the read failed
uint16_t
i2c_try_readReg(uint8_t add, uint8_t reg)
{
    uint16_t count = 255;
    i2c_rep_start(add << 1);    // I2C write direction
    i2c_write(reg);         // register selection
    i2c_rep_start((add << 1) | 1);  // I2C read direction
    TWCR = (1 << TWINT) | (1 << TWEN);

    while (!(TWCR & (1 << TWINT)))
    {
        count--;

        if (count == 0)
        {
            //we are in a blocking state => we don't insist
            TWCR = 0;       //and we force a reset on TWINT register
            return 0xffff;  // return failure to read
        }
    }

    uint8_t r = TWDR;
    i2c_stop();
    return r;
}

// read a 16bit unsigned int from the i2c bus
uint16_t
i2c_readReg16(int8_t addr, int8_t reg)
{
    uint8_t b[2];
    i2c_read_reg_to_buf(addr, reg, (uint8_t *) & b, sizeof(b));
    return (b[0] << 8) | b[1];
}

void
i2c_srf08_change_addr(int8_t current, int8_t moveto)
{
    // to change a srf08 address, we must write the following sequence to the command register
    // this sequence must occur as 4 seperate i2c transactions!!   A0 AA A5 [addr]
    i2c_writeReg(current, SRF08_REV_COMMAND, 0xA0);
    delay(30);
    i2c_writeReg(current, SRF08_REV_COMMAND, 0xAA);
    delay(30);
    i2c_writeReg(current, SRF08_REV_COMMAND, 0xA5);
    delay(30);
    i2c_writeReg(current, SRF08_REV_COMMAND, moveto);
    delay(30);          // now change i2c address
}

// discover previously known sensors and any new sensor (move new sensors to assigned area)
void
i2c_srf08_discover()
{
    uint8_t addr;
    uint16_t x;
    srf08_ctx.sensors = 0;  // determine how many sensors are plugged in
    addr = SRF08_SENSOR_FIRST;  // using the I2C address range we choose, starting with first one

    for (uint8_t i = 0; i < SRF08_MAX_SENSORS && x != 0xff; i++)
    {
        // 0xff means a mesure is currently running, so try again
        x = i2c_try_readReg(addr, SRF08_REV_COMMAND);   // read the revision as a way to check if sensor exists at this location

        if (x != 0xffff)
        {
            // detected a sensor at this address
            i2c_writeReg(addr, SRF08_LIGHT_GAIN, 0x15);     // not set to max to avoid bad echo indoor
            i2c_writeReg(addr, SRF08_ECHO_RANGE, 46);   // set to 2m max
            srf08_ctx.sensors++;
            addr += 1;      // 7 bit address => +1 is +2 for 8 bit address
        }
    }

    if (srf08_ctx.sensors < SRF08_MAX_SENSORS)
    {
        // do not add sensors if we are already maxed
        // now determine if any sensor is on the 'new sensor' address (srf08 default address)
        x = i2c_try_readReg(SRF08_DEFAULT_ADDRESS, SRF08_REV_COMMAND);  // we try to read the revision number

        if (x != 0xffff)
        {
            // new sensor detected at SRF08 default address
            i2c_srf08_change_addr(SRF08_DEFAULT_ADDRESS, addr << 1);    // move sensor to the next address (8 bit format expected by the device)
            srf08_ctx.sensors++;
        }
    }
}

void
Sonar_update()
{
    if ((int32_t)(currentTime - srf08_ctx.deadline) < 0)
    {
        return;
    }

    srf08_ctx.deadline = currentTime;

    switch (srf08_ctx.state)
    {
        case 0:
            i2c_srf08_discover();

            if (srf08_ctx.sensors > 0)
            {
                srf08_ctx.state++;
            }
            else
            {
                srf08_ctx.deadline += 5000000;    // wait 5 secs before trying search again
            }

            break;

        case 1:
            srf08_ctx.current = 0;
            srf08_ctx.state++;
            srf08_ctx.deadline += SRF08_RANGE_SLEEP;
            break;
#if defined(SONAR_MULTICAST_PING)

        case 2:
            // send a ping via the general broadcast address
            i2c_writeReg(0, SRF08_REV_COMMAND, 0x51);   // start ranging, result in centimeters
            srf08_ctx.state++;
            srf08_ctx.deadline += SRF08_RANGE_WAIT;
            break;

        case 3:
            srf08_ctx.range[srf08_ctx.current] =
                i2c_readReg16(SRF08_SENSOR_FIRST + srf08_ctx.current,
                              SRF08_ECHO_RANGE);
            srf08_ctx.current++;

            if (srf08_ctx.current >= srf08_ctx.sensors)
            {
                srf08_ctx.state = 1;
            }

            break;
#else

        case 2:
            // send a ping to the current sensor
            i2c_writeReg(SRF08_SENSOR_FIRST + srf08_ctx.current, SRF08_REV_COMMAND, 0x51);  // start ranging, result in centimeters
            srf08_ctx.state++;
            srf08_ctx.deadline += SRF08_RANGE_WAIT;
            break;

        case 3:
            srf08_ctx.range[srf08_ctx.current] =
                i2c_readReg16(SRF08_SENSOR_FIRST + srf08_ctx.current,
                              SRF08_ECHO_RANGE);
            srf08_ctx.current++;

            if (srf08_ctx.current >= srf08_ctx.sensors)
            {
                srf08_ctx.state = 1;
            }
            else
            {
                srf08_ctx.state = 2;
            }

            break;
#endif
    }

    sonarAlt = srf08_ctx.range[0];  // only one sensor considered for the moment
}
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
