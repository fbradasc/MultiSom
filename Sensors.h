#ifndef SENSORS_H_
#define SENSORS_H_

void ACC_getADC();
void Gyro_getADC();
#if defined(MAG)
    uint8_t Mag_getADC();
#endif
#if defined(BARO)
    uint8_t Baro_update();
#endif
#if defined(SONAR)
    uint8_t Sonar_update();
#endif
#if defined(PITOT)
    void Pitot_update();
#endif

void initSensors();
void i2c_rep_start(uint8_t address);
void i2c_write(uint8_t data);
void i2c_stop(void);
void i2c_write(uint8_t data);
void i2c_writeReg(uint8_t add, uint8_t reg, uint8_t val);
uint8_t i2c_readReg(uint8_t add, uint8_t reg);
uint8_t i2c_readAck();
uint8_t i2c_readNak();

void i2c_read_reg_to_buf(uint8_t add, uint8_t reg, uint8_t *buf,
                         uint8_t size);

#if defined(IMPLEMENTATION)
    #define IMPLEMENTATION_DEFINED
#endif

#undef IMPLEMENTATION
#include "drivers/sensors/sensors.hpp"

#if defined(IMPLEMENTATION_DEFINED)
    #define IMPLEMENTATION
#endif

#define ACCZ_25deg   (int16_t)(ACC_1G * 0.90631)    // 0.90631 = cos(25deg) (cos(theta) of accZ comparison)
#define ACC_VelScale (9.80665f / 10000.0f / ACC_1G)
#endif /* SENSORS_H_ */
