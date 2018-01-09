#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Accelerometer ADXL345
// ************************************************************************************************************
// I2C adress: 0x3A (8bit)    0x1D (7bit)
// Resolution: 10bit (Full range - 14bit, but this is autoscaling 10bit ADC to the range +- 16g)
// principle:
//  1) CS PIN must be linked to VCC to select the I2C mode
//  2) SD0 PIN must be linked to VCC to select the right I2C adress
//  3) bit  b00000100 must be set on register 0x2D to read data (only once at the initialization)
//  4) bits b00001011 must be set on register 0x31 to select the data format (only once at the initialization)
// ************************************************************************************************************
#if !defined(ADXL345_ADDRESS)
    #define ADXL345_ADDRESS 0x1D
    //#define ADXL345_ADDRESS 0x53   //WARNING: Conflicts with a Wii Motion plus!
#endif

void
ACC_init()
{
    delay(10);
    i2c_writeReg(ADXL345_ADDRESS, 0x2D, 1 << 3);    //  register: Power CTRL  -- value: Set measure bit 3 on
    i2c_writeReg(ADXL345_ADDRESS, 0x31, 0x0B);  //  register: DATA_FORMAT -- value: Set bits 3(full range) and 1 0 on (+/- 16g-range)
    i2c_writeReg(ADXL345_ADDRESS, 0x2C, 0x09);  //  register: BW_RATE     -- value: rate=50hz, bw=20hz
}

void
ACC_getADC()
{
    i2c_getSixRawADC(ADXL345_ADDRESS, 0x32);
    ACC_ORIENTATION(((rawADC[1] << 8) | rawADC[0]),
                    ((rawADC[3] << 8) | rawADC[2]),
                    ((rawADC[5] << 8) | rawADC[4]));
    ACC_Common();
}
#else // IMPLEMENTATION
#define ACC_1G 265
#endif // IMPLEMENTATION
