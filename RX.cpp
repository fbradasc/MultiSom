#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "Serial.h"
#include "Protocol.h"
#include "MultiSom.h"
#include "Alarms.h"

/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

#if defined(STANDARD_RX)
    #include "drivers/rx/protocols/standard.cpp"
#elif defined(SERIAL_SUM_PPM)
    #include "drivers/rx/protocols/ppmsum.cpp"
#elif defined(SUMD)
    #include "drivers/rx/protocols/sumd.cpp"
#elif defined(SPEKTRUM) || defined(SPEK_BIND)
    #include "drivers/rx/protocols/spektrum.cpp"
#elif defined(SBUS)
    #include "drivers/rx/protocols/sbus.cpp"
#elif defined(OPENLRSv2MULTI) || defined(OPENLRS_V2)
    #include "drivers/rx/protocols/openlrs.cpp"
#endif
