#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// I2C Gyroscope and Accelerometer MPU6050
// ************************************************************************************************************
#if !defined(MPU6050_ADDRESS)
    #define MPU6050_ADDRESS     0x68    // address pin AD0 low (GND), default for FreeIMU v0.4 and InvenSense evaluation board
    //#define MPU6050_ADDRESS     0x69 // address pin AD0 high (VCC)
#endif



static void
Gyro_init()
{
    i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x80);  //PWR_MGMT_1    -- DEVICE_RESET 1
    delay(50);
    i2c_writeReg(MPU6050_ADDRESS, 0x6B, 0x03);  //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
    i2c_writeReg(MPU6050_ADDRESS, 0x1A, GYRO_DLPF_CFG);     //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    i2c_writeReg(MPU6050_ADDRESS, 0x1B, 0x18);  //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
    // enable I2C bypass for AUX I2C
#if defined(MAG)
    i2c_writeReg(MPU6050_ADDRESS, 0x37, 0x02);  //INT_PIN_CFG   -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=1 ; CLKOUT_EN=0
#endif
}

void
Gyro_getADC()
{
    i2c_getSixRawADC(MPU6050_ADDRESS, 0x43);
    GYRO_ORIENTATION(((rawADC[0] << 8) | rawADC[1]) >> 2,   // range: +/- 8192; +/- 2000 deg/sec
                     ((rawADC[2] << 8) | rawADC[3]) >> 2,
                     ((rawADC[4] << 8) | rawADC[5]) >> 2);
    GYRO_Common();
}
#else // IMPLEMENTATION
// GYRO SCALE: we ignore the last 2 bits and convert it for rad/s
#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)  //16.4 LSB = 1 deg/s
#endif // IMPLEMENTATION
