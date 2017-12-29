#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// BARO section
// ************************************************************************************************************
    static void
Baro_Common ()
{
    static int32_t baroHistTab[BARO_TAB_SIZE];
    static uint8_t baroHistIdx;

    uint8_t indexplus1 = (baroHistIdx + 1);
    if (indexplus1 == BARO_TAB_SIZE)
        indexplus1 = 0;
    baroHistTab[baroHistIdx] = baroPressure;
    baroPressureSum += baroHistTab[baroHistIdx];
    baroPressureSum -= baroHistTab[indexplus1];
    baroHistIdx = indexplus1;
}
#endif // IMPLEMENTATION

#if defined(BMP085)
#include "baro/bmp085.cpp"
#elif defined(MS561101BA)
#include "baro/ms561101ba.cpp"
#endif
