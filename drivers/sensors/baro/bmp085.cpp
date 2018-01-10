#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Barometer BOSCH BMP085
// ************************************************************************************************************
// I2C adress: 0x77 (7bit)
// principle:
//  1) read the calibration register (only once at the initialization)
//  2) read uncompensated temperature (not mandatory at every cycle)
//  3) read uncompensated pressure
//  4) raw temp + raw pressure => calculation of the adjusted pressure
//  the following code uses the maximum precision setting (oversampling setting 3)
// ************************************************************************************************************

# define BMP085_ADDRESS 0x77

static struct
{
    // sensor registers from the BOSCH BMP085 datasheet
    int16_t ac1, ac2, ac3;
    uint16_t ac4, ac5, ac6;
    int16_t b1, b2, mb, mc, md;
    union
    {
        uint16_t val;
        uint8_t raw[2];
    }
    ut;                 //uncompensated T
    union
    {
        uint32_t val;
        uint8_t raw[4];
    }
    up;                 //uncompensated P
    uint8_t state;
    uint32_t deadline;
}
bmp085_ctx;
# define OSS 3

/* transform a series of bytes from big endian to little
   endian and vice versa. */
void swap_endianness(void *buf, size_t size)
{
    /* we swap in-place, so we only have to
     * place _one_ element on a temporary tray
     */
    uint8_t tray;
    uint8_t *from;
    uint8_t *to;

    /* keep swapping until the pointers have assed each other */
    for (from = (uint8_t *) buf, to = &from[size - 1]; from < to; from++, to--)
    {
        tray = *from;
        *from = *to;
        *to = tray;
    }
}

void i2c_BMP085_readCalibration()
{
    delay(10);
    //read calibration data in one go
    size_t s_bytes =
        (uint8_t *) &bmp085_ctx.md - (uint8_t *) &bmp085_ctx.ac1 +
        sizeof(bmp085_ctx.ac1);
    i2c_read_reg_to_buf(BMP085_ADDRESS, 0xAA, (uint8_t *) &bmp085_ctx.ac1,
                        s_bytes);
    // now fix endianness
    int16_t *p;

    for (p = &bmp085_ctx.ac1; p <= &bmp085_ctx.md; p++)
    {
        swap_endianness(p, sizeof(*p) );
    }
}

// read uncompensated temperature value: send command first
void i2c_BMP085_UT_Start(void)
{
    i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x2e);
    i2c_rep_start(BMP085_ADDRESS << 1);
    i2c_write(0xF6);
    i2c_stop();
}

// read uncompensated pressure value: send command first
void i2c_BMP085_UP_Start()
{
    i2c_writeReg(BMP085_ADDRESS, 0xf4, 0x34 + (OSS << 6) );  // control register value for oversampling setting 3
    i2c_rep_start(BMP085_ADDRESS << 1);     //I2C write direction => 0
    i2c_write(0xF6);
    i2c_stop();
}

// read uncompensated pressure value: read result bytes
// the datasheet suggests a delay of 25.5 ms (oversampling settings 3) after the send command
void i2c_BMP085_UP_Read()
{
    i2c_rep_start( (BMP085_ADDRESS << 1) | 1 );   //I2C read direction => 1
    bmp085_ctx.up.raw[2] = i2c_readAck();
    bmp085_ctx.up.raw[1] = i2c_readAck();
    bmp085_ctx.up.raw[0] = i2c_readNak();
}

// read uncompensated temperature value: read result bytes
// the datasheet suggests a delay of 4.5 ms after the send command
void i2c_BMP085_UT_Read()
{
    i2c_rep_start( (BMP085_ADDRESS << 1) | 1 );   //I2C read direction => 1
    bmp085_ctx.ut.raw[1] = i2c_readAck();
    bmp085_ctx.ut.raw[0] = i2c_readNak();
}

void i2c_BMP085_Calculate()
{
    int32_t x1, x2, x3, b3, b5, b6, p, tmp;
    uint32_t b4, b7;

    // Temperature calculations
    x1 = ( (int32_t) bmp085_ctx.ut.val - bmp085_ctx.ac6 ) * bmp085_ctx.ac5 >> 15;
    x2 = ( (int32_t) bmp085_ctx.mc << 11 ) / (x1 + bmp085_ctx.md);
    b5 = x1 + x2;
    baroTemperature = (b5 * 10 + 8) >> 4;   // in 0.01 degC (same as MS561101BA temperature)
    // Pressure calculations
    b6 = b5 - 4000;
    x1 = (bmp085_ctx.b2 * (b6 * b6 >> 12) ) >> 11;
    x2 = bmp085_ctx.ac2 * b6 >> 11;
    x3 = x1 + x2;
    tmp = bmp085_ctx.ac1;
    tmp = (tmp * 4 + x3) << OSS;
    b3 = (tmp + 2) / 4;
    x1 = bmp085_ctx.ac3 * b6 >> 13;
    x2 = (bmp085_ctx.b1 * (b6 * b6 >> 12) ) >> 16;
    x3 = ( (x1 + x2) + 2 ) >> 2;
    b4 = (bmp085_ctx.ac4 * (uint32_t)(x3 + 32768) ) >> 15;
    b7 = ( (uint32_t)(bmp085_ctx.up.val >> (8 - OSS) ) - b3 ) * (50000 >> OSS);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    baroPressure = p + ( (x1 + x2 + 3791) >> 4 );
}

void Baro_init()
{
    delay(10);
    i2c_BMP085_readCalibration();
    delay(5);
    i2c_BMP085_UT_Start();
    bmp085_ctx.deadline = currentTime + 5000;
}

//return 0: no data available, no computation ;  1: new value available  ; 2: no new value, but computation time
uint8_t Baro_update()
{
    // first UT conversion is started in init procedure
    if (currentTime < bmp085_ctx.deadline)
    {
        return(0);
    }

    bmp085_ctx.deadline = currentTime + 6000;   // 1.5ms margin according to the spec (4.5ms T convetion time)

    if (bmp085_ctx.state == 0)
    {
        i2c_BMP085_UT_Read();
        i2c_BMP085_UP_Start();
        bmp085_ctx.state = 1;
        Baro_Common();
        bmp085_ctx.deadline += 21000;   // 6000+21000=27000 1.5ms margin according to the spec (25.5ms P convetion time with OSS=3)
        return(1);
    }
    else
    {
        i2c_BMP085_UP_Read();
        i2c_BMP085_UT_Start();
        i2c_BMP085_Calculate();
        bmp085_ctx.state = 0;
        return(2);
    }
}

#else // IMPLEMENTATION
#endif // IMPLEMENTATION
