#ifndef RX_H_
#define RX_H_

void configureReceiver();
void computeRC();
uint16_t readRawRC(uint8_t chan);
#if defined(SERIAL_RX)
void readSerial_RX(void);
#endif
#if defined(SPEK_BIND)  // Bind Support
void spekBind(void);
#endif

#endif /* RX_H_ */
