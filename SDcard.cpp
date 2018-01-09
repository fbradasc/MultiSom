/*
 *  SDcard logging support
 *  based on wareck's Multiwii_Nav_V2.3 https://github.com/wareck/Multiwii_Nav_V2.3
 *  Updated for PatrikE's Multiwii 2.4/FixedWing fork by Martin Espinoza <martin.espinoza@gmail.com>
 */

/* thanks to Rcnet team for there job !*/

#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"

#ifdef MWI_SDCARD

    #include "EEPROM.h"
    #include "SDcard.h"
    #include "MultiSom.h"
    #include "Alarms.h"
    #include "GPS.h"
    #if !defined(USE_PETIT_SERIAL)
        #include "LCD.h"
    #endif
    #define PERMANENT_LOG_FILENAME "PERM.TXT"
    #define GPS_LOG_FILENAME "GPS_DATA.RAW"

    #if defined(PETIT_FS)
        #include "drivers/sdcard/PetitFs.cpp"
    #else
        #include "drivers/sdcard/SdFat.cpp"
    #endif
#endif
