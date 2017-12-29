#if defined(IMPLEMENTATION)
// ************************************************************************
// LIS3LV02 I2C Accelerometer
// ************************************************************************
#define LIS3A  0x1D

    void
ACC_init ()
{
    i2c_writeReg (LIS3A, 0x20, 0xD7);	// CTRL_REG1   1101 0111 Pwr on, 160Hz
    i2c_writeReg (LIS3A, 0x21, 0x50);	// CTRL_REG2   0100 0000 Littl endian, 12 Bit, Boot
}

    void
ACC_getADC ()
{
    i2c_getSixRawADC (LIS3A, 0x28 + 0x80);
    ACC_ORIENTATION (((rawADC[1] << 8) | rawADC[0]) >> 2,
                     ((rawADC[3] << 8) | rawADC[2]) >> 2,
                     ((rawADC[5] << 8) | rawADC[4]) >> 2);
    ACC_Common ();
}
#else // IMPLEMENTATION
#define ACC_1G 255
#endif // IMPLEMENTATION
