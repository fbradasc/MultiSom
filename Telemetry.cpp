// ****************************************************************
// FrSky telemetry
// Version: 0.4.0   - by QuadBow
// Changes: V0.4.0: - supports 2.4 with new features added
//                    device specific selection of data to be sent
//                    different ways for displaying
//                    different ways to present data
// Version: 0.3.0
// Changes: V0.3.0: - new structure with cpp/h-files
// Date 20/09/2012
// Changes: V0.2.1: - make it work with 2.1 (shared dev)
// Date: 14/08/2012
// Changes: V0.2: - Byte stuffing added
//                - vBat will be send, if "#define FAS_100" is comment out
//          V0.1: - First release
// ****************************************************************

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiSom.h"
#include "Sensors.h"
#include "Serial.h"
#include "Telemetry.h"

#define IMPLEMENTATION

#if defined(FRSKY_TELEMETRY)
#include "drivers/telemetry/FrSky/DType.cpp"
#elif defined(SPORT_TELEMETRY)
#include "drivers/telemetry/FrSky/SmartPort.cpp"
#endif

#undef IMPLEMENTATION
