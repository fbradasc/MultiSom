/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

static uint8_t rcChannel[RC_CHANS] = {PITCH, YAW, THROTTLE, ROLL, AUX1, AUX2, AUX3, AUX4};

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
    /******************    Configure each rc pin for PCINT    ***************************/
    SerialOpen(RX_SERIAL_PORT, 115200);
}

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAXCHAN 8
#define SUMD_BUFFSIZE SUMD_MAXCHAN*2 + 5 // 6 channels + 5 -> 17 bytes for 6 channels
static uint8_t sumdIndex = 0;
static uint8_t sumdSize = 0;
static uint8_t sumd[SUMD_BUFFSIZE] = {0};

void readSerial_RX(void)
{
    while (SerialAvailable(RX_SERIAL_PORT))
    {
        int val = SerialRead(RX_SERIAL_PORT);

        if (sumdIndex == 0 && val != SUMD_SYNCBYTE)
        {
            continue;
        }

        if (sumdIndex == 2)
        {
            sumdSize = val;
        }

        if (sumdIndex < SUMD_BUFFSIZE)
        {
            sumd[sumdIndex] = val;
        }

        sumdIndex++;

        if (sumdIndex == sumdSize * 2 + 5)
        {
            sumdIndex = 0;
            spekFrameFlags = 0x00;
            debug[1] = sumd[1];

            if (sumdSize > SUMD_MAXCHAN)
            {
                sumdSize = SUMD_MAXCHAN;
            }

            for (uint8_t b = 0; b < sumdSize; b++)
            {
                rcValue[b] = ((sumd[2 * b + 3] << 8) | sumd[2 * b + 4]) >> 3;
            }

            spekFrameDone = 0x01; // havent checked crc at all
#if defined(FAILSAFE)

            if (sumd[1] == 0x01) // clear FailSafe counter
            {
                if (failsafeCnt > 20)
                {
                    failsafeCnt -= 20;
                }
                else
                {
                    failsafeCnt = 0;
                }
            }

#endif
        }
    }
}

uint16_t readRawRC(uint8_t chan)
{
    uint16_t data;

    if (chan < RC_CHANS)
    {
        data = rcValue[rcChannel[chan]];
    }
    else
    {
        data = 1500;
    }

    return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/
#define AVERAGING_ARRAY_LENGTH 4
void computeRC()
{
    static uint16_t rcData4Values[RC_CHANS][AVERAGING_ARRAY_LENGTH - 1];
    uint16_t rcDataMean, rcDataTmp;
    static uint8_t rc4ValuesIndex = 0;
    uint8_t chan, a;
    uint8_t failsafeGoodCondition = 1;
    rc4ValuesIndex++;

    if (rc4ValuesIndex == AVERAGING_ARRAY_LENGTH - 1)
    {
        rc4ValuesIndex = 0;
    }

    for (chan = 0; chan < RC_CHANS; chan++)
    {
        // Stick Scaling http://mifi.no/blog/?p=95
#if defined(STICK_SCALING_FACTOR)
        if (chan < 4)
        {
            rcDataTmp = ((int16_t)readRawRC(chan) - 1500) * STICK_SCALING_FACTOR + 1500;
        }
        else
#endif
        {
            rcDataTmp = readRawRC(chan);
        }

#if defined(FAILSAFE)
        failsafeGoodCondition = rcDataTmp > FAILSAFE_DETECT_TRESHOLD || chan > 3 || !f.ARMED; // update controls channel only if pulse is above FAILSAFE_DETECT_TRESHOLD
#endif                                                                                // In disarmed state allow always update for easer configuration.

        if (failsafeGoodCondition)
        {
            rcData[chan] = rcDataTmp;
        }

        if (chan < 8 && rcSerialCount > 0) // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
        {
            rcSerialCount --;
#if defined(FAILSAFE)
            failsafeCnt = 0;
#endif

            if (rcSerial[chan] > 900) // only relevant channels are overridden
            {
                rcData[chan] = rcSerial[chan];
            }
        }
    }
}
