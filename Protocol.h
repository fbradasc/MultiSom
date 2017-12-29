#ifndef PROTOCOL_H_
#define PROTOCOL_H_

//#ifndef (CLEANFLIGHT)
typedef enum {
	FEATURE_RX_PPM_BIT          = 0,
	FEATURE_VBAT_BIT            ,
	FEATURE_INFLIGHT_ACC_CAL_BIT,
	FEATURE_RX_SERIAL_BIT       ,
	FEATURE_MOTOR_STOP_BIT      ,
	FEATURE_SERVO_TILT_BIT      ,
	FEATURE_SOFTSERIAL_BIT      ,
	FEATURE_GPS_BIT             ,
	FEATURE_FAILSAFE_BIT        ,
	FEATURE_SONAR_BIT           ,
	FEATURE_TELEMETRY_BIT       ,
	FEATURE_CURRENT_METER_BIT   ,
	FEATURE_3D_BIT              ,
	FEATURE_RX_PARALLEL_PWM_BIT ,
	FEATURE_RX_MSP_BIT          ,
	FEATURE_RSSI_ADC_BIT        ,
	FEATURE_LED_STRIP_BIT       ,
	FEATURE_ONESHOT125_BIT      ,
	FEATURE_BLACKBOX_BIT        
} features_e;
//#endif

void serialCom();
void debugmsg_append_str(const char *str);
void msp_push(uint8_t uart, uint8_t msp);
void featureSet(uint32_t mask);

#endif /* PROTOCOL_H_ */
