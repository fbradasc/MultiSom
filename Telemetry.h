#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#if defined(IMPLEMENTATION)
# define IMPLEMENTATION_DEFINED
#endif

#undef IMPLEMENTATION

#if defined(FRSKY_TELEMETRY)
#include "drivers/telemetry/FrSky/DType.cpp"
#endif

#if defined(SPORT_TELEMETRY)
#include "drivers/telemetry/FrSky/SmartPort.cpp"
#endif

#if defined(IMPLEMENTATION_DEFINED)
# define IMPLEMENTATION
#endif

// Functions
void init_telemetry(void);
void run_telemetry(void);

#endif /* TELEMETRY_H_ */
