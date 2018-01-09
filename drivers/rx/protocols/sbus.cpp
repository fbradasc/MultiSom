/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/


//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; // interval [1000;2000]

// for 16 + 2 Channels SBUS. The 10 extra channels 8->17 are not used by MultiSom, but it should be easy to integrate them.
static uint8_t rcChannel[RC_CHANS] = {SBUS};

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
    /******************    Configure each rc pin for PCINT    ***************************/
    SerialOpen(RX_SERIAL_PORT, 100000);
#if defined(MEGA)

    switch (RX_SERIAL_PORT) //parity
    {
        case 0:
            UCSR0C |= (1 << UPM01) | (1 << USBS0);
            break;

        case 1:
            UCSR1C |= (1 << UPM11) | (1 << USBS1);
            break;

        case 2:
            UCSR2C |= (1 << UPM21) | (1 << USBS2);
            break;

        case 3:
            UCSR3C |= (1 << UPM31) | (1 << USBS3);
            break;
    }

#endif
}

/**************************************************************************************/
/***************                   SBUS RX Data                    ********************/
/**************************************************************************************/
#define SBUS_SYNCBYTE 0x0F // Not 100% sure: at the beginning of coding it was 0xF0 !!!
static uint16_t sbusIndex = 0;
static uint16_t sbus[25] = {0};

void readSerial_RX()
{
    while (SerialAvailable(RX_SERIAL_PORT))
    {
        int val = SerialRead(RX_SERIAL_PORT);

        if (sbusIndex == 0 && val != SBUS_SYNCBYTE)
        {
            continue;
        }

        sbus[sbusIndex++] = val;

        if (sbusIndex == 25)
        {
            sbusIndex = 0;
            spekFrameFlags = 0x00;
            rcValue[0]  = ((sbus[1] | sbus[2] << 8) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[1]  = ((sbus[2] >> 3 | sbus[3] << 5) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[2]  = ((sbus[3] >> 6 | sbus[4] << 2 | sbus[5] << 10) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[3]  = ((sbus[5] >> 1 | sbus[6] << 7) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[4]  = ((sbus[6] >> 4 | sbus[7] << 4) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[5]  = ((sbus[7] >> 7 | sbus[8] << 1 | sbus[9] << 9) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[6]  = ((sbus[9] >> 2 | sbus[10] << 6) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[7]  = ((sbus[10] >> 5 | sbus[11] << 3) & 0x07FF) / 2 + SBUS_MID_OFFSET; // & the other 8 + 2 channels if you need them
            //The following lines: If you need more than 8 channels, max 16 analog + 2 digital. Must comment the not needed channels!
            rcValue[8]  = ((sbus[12] | sbus[13] << 8) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[9]  = ((sbus[13] >> 3 | sbus[14] << 5) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[10] = ((sbus[14] >> 6 | sbus[15] << 2 | sbus[16] << 10) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[11] = ((sbus[16] >> 1 | sbus[17] << 7) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[12] = ((sbus[17] >> 4 | sbus[18] << 4) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[13] = ((sbus[18] >> 7 | sbus[19] << 1 | sbus[20] << 9) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[14] = ((sbus[20] >> 2 | sbus[21] << 6) & 0x07FF) / 2 + SBUS_MID_OFFSET;
            rcValue[15] = ((sbus[21] >> 5 | sbus[22] << 3) & 0x07FF) / 2 + SBUS_MID_OFFSET;

            // now the two Digital-Channels
            if ((sbus[23]) & 0x0001)
            {
                rcValue[16] = 2000;
            }
            else
            {
                rcValue[16] = 1000;
            }

            if ((sbus[23] >> 1) & 0x0001)
            {
                rcValue[17] = 2000;
            }
            else
            {
                rcValue[17] = 1000;
            }

            spekFrameDone = 0x01;
            // Failsafe: there is one Bit in the SBUS-protocol (Byte 25, Bit 4) whitch is the failsafe-indicator-bit
#if defined(FAILSAFE)

            if (!((sbus[23] >> 3) & 0x0001)) // clear FailSafe counter
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
            // For some reason the SBUS data provides only about 75% of the actual RX output pulse width
            // Adjust the actual value by +/-25%.  Sign determined by pulse width above or below center
            uint8_t adj_index;

            for (adj_index = 0; adj_index < 16; adj_index++)
            {
                if (rcValue[adj_index] < MIDRC)
                {
                    rcValue[adj_index] -= (MIDRC - rcValue[adj_index]) >> 2;
                }
                else
                {
                    rcValue[adj_index] += (rcValue[adj_index] - MIDRC) >> 2;
                }
            }
        }
    }
}

/**************************************************************************************/
/*************** SUMD ********************/
/**************************************************************************************/


/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/

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
