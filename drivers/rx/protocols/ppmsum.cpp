/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

static uint8_t rcChannel[RC_CHANS] = {SERIAL_SUM_PPM};

void rxInt(void);

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
    /******************    Configure each rc pin for PCINT    ***************************/
    PPM_PIN_INTERRUPT;
}

/**************************************************************************************/
/***************                PPM SUM RX Pin reading             ********************/
/**************************************************************************************/
// attachInterrupt fix for promicro
#if defined(PROMICRO)
ISR(INT6_vect)
{
    rxInt();
}
#endif

// PPM_SUM at THROTTLE PIN on MEGA boards
#if defined(PPM_ON_THROTTLE) && defined(MEGA)
ISR(PCINT2_vect)
{
    if (PINK & (1 << 0) )
    {
        rxInt();
    }
}
#endif

// Read PPM SUM RX Data
void rxInt(void)
{
    uint16_t now, diff;
    static uint16_t last = 0;
    static uint8_t chan = 0;

#if defined(FAILSAFE)
    static uint8_t GoodPulses;
#endif

    now = micros();
    sei();
    diff = now - last;
    last = now;

    if (diff > 3000)
    {
        chan = 0;
    }
    else
    {
        if ( (900 < diff) && (diff < 2200) && (chan < RC_CHANS) )
        {   //Only if the signal is between these values it is valid, otherwise the failsafe counter should move up
            rcValue[chan] = diff;

#if defined(FAILSAFE)
            if ( (chan < 4) && (diff > FAILSAFE_DETECT_TRESHOLD) )
            {
                GoodPulses |= (1 << chan); // if signal is valid - mark channel as OK
            }

            if (GoodPulses == 0x0F)// If first four chanells have good pulses, clear FailSafe counter
            {
                GoodPulses = 0;

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

        chan++;
    }
}

/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/

uint16_t readRawRC(uint8_t chan)
{
    uint16_t data;
    uint8_t oldSREG;

    oldSREG = SREG;
    cli();                 // Let's disable interrupts
    data = rcValue[rcChannel[chan]]; // Let's copy the data Atomically
    SREG = oldSREG;        // Let's restore interrupt state
    return(data); // We return the value correctly copied when the IRQ's where disabled
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
            rcDataTmp = ( (int16_t)readRawRC(chan) - 1500 ) * STICK_SCALING_FACTOR + 1500;
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
            rcDataMean = rcDataTmp;

            for (a = 0; a < AVERAGING_ARRAY_LENGTH - 1; a++)
            {
                rcDataMean += rcData4Values[chan][a];
            }

            rcDataMean = (rcDataMean + (AVERAGING_ARRAY_LENGTH / 2) ) / AVERAGING_ARRAY_LENGTH;

            if (rcDataMean < (uint16_t) rcData[chan] - 3)
            {
                rcData[chan] = rcDataMean + 2;
            }

            if (rcDataMean > (uint16_t)rcData[chan] + 3)
            {
                rcData[chan] = rcDataMean - 2;
            }

            rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;
        }

#if 0
        if (supress_data_from_rx)
        {
            //if (chan<8 && rcSerialCount > 0) // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
            if (chan < 8)
            {
                // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
                //rcSerialCount--;

# if defined(FAILSAFE)
                failsafeCnt = 0;
# endif

                if (rcSerial[chan] > 900)
                {
                    rcDataMean = rcSerial[chan];

                    for (a = 0; a < AVERAGING_ARRAY_LENGTH - 1; a++)
                    {
                        rcDataMean += rcData4Values[chan][a];
                    }

                    rcDataMean = (rcDataMean + (AVERAGING_ARRAY_LENGTH / 2) ) / AVERAGING_ARRAY_LENGTH;

                    if (rcDataMean < (uint16_t)rcData[chan] - 3)
                    {
                        rcData[chan] = rcDataMean + 2;
                    }

                    if (rcDataMean > (uint16_t)rcData[chan] + 3)
                    {
                        rcData[chan] = rcDataMean - 2;
                    }

                    rcData4Values[chan][rc4ValuesIndex] = rcDataTmp;

# if 0
                    if (rcSerial[chan] > 900) // only relevant channels are overridden
                    {
                        rcData[chan] = rcSerial[chan];
                    }
# endif
                }
            }
        }
#endif

        if ( (chan < 8) && (rcSerialCount > 0) ) // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
        {
            rcSerialCount--;
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
