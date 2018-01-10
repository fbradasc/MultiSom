/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

#include <wiring.c>  //Auto-included by the Arduino core... but we need it sooner.

//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

static uint8_t rcChannel[RC_CHANS] = {PITCH, YAW, THROTTLE, ROLL, AUX1, AUX2, AUX3, AUX4, 8, 9, 10, 11};

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
    /******************    Configure each rc pin for PCINT    ***************************/
    SerialOpen(RX_SERIAL_PORT, 115200);
}

void readSerial_RX(void)
{
    if ( (!f.ARMED) &&
#if defined(FAILSAFE) || (RX_SERIAL_PORT != 0)
         (failsafeCnt > 5) &&
#endif
         (SerialPeek(RX_SERIAL_PORT) == '$') )
    {
        while (SerialAvailable(RX_SERIAL_PORT) )
        {
            serialCom();
            delay(10);
        }

        return;
    } //End of: Is it the GUI?

    while (SerialAvailable(RX_SERIAL_PORT) > SPEK_FRAME_SIZE) // More than a frame?  More bytes implies we weren't called for multiple frame times.  We do not want to process 'old' frames in the buffer.
    {
        for (uint8_t i = 0; i < SPEK_FRAME_SIZE; i++)  //Toss one full frame of bytes.
        {
            SerialRead(RX_SERIAL_PORT);
        }
    }

    if (spekFrameFlags == 0x01)   //The interrupt handler saw at least one valid frame start since we were last here.
    {
        if (SerialAvailable(RX_SERIAL_PORT) == SPEK_FRAME_SIZE)  //A complete frame? If not, we'll catch it next time we are called.
        {
            SerialRead(RX_SERIAL_PORT);
            SerialRead(RX_SERIAL_PORT);        //Eat the header bytes

            for (uint8_t b = 2; b < SPEK_FRAME_SIZE; b += 2)
            {
                uint8_t bh = SerialRead(RX_SERIAL_PORT);
                uint8_t bl = SerialRead(RX_SERIAL_PORT);
                uint8_t spekChannel = 0x0F & (bh >> SPEK_CHAN_SHIFT);

                if (spekChannel < RC_CHANS)
                {
                    rcValue[spekChannel] = 988 + ( ( ( (uint16_t)(bh & SPEK_CHAN_MASK) << 8 ) + bl )SPEK_DATA_SHIFT );
                }
            }

            spekFrameFlags = 0x00;
            spekFrameDone = 0x01;
#if defined(FAILSAFE)
            if (failsafeCnt > 20)   // Valid frame, clear FailSafe counter
            {
                failsafeCnt -= 20;
            }
            else
            {
                failsafeCnt = 0;
            }
#endif
        }
        else //Start flag is on, but not enough bytes means there is an incomplete frame in buffer.  This could be OK, if we happened to be called in the middle of a frame.  Or not, if it has been a while since the start flag was set.
        {
            uint32_t spekInterval = (timer0_overflow_count << 8) * (64 / clockCyclesPerMicrosecond() ) - spekTimeLast;

            if (spekInterval > 2500)  //If it has been a while, make the interrupt handler start over.
            {
                spekFrameFlags = 0;
            }
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
            rcData[chan] = rcDataTmp;
        }

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

#if defined(SPEK_BIND)  // Bind Support
void spekBind()
{
    pinMode(SPEK_BIND_DATA, INPUT);     // Data line from sat
    digitalWrite(SPEK_BIND_DATA, LOW);  // Turn off internal Pull Up resistor
    pinMode(SPEK_BIND_GROUND, INPUT);
    digitalWrite(SPEK_BIND_GROUND, LOW);
    pinMode(SPEK_BIND_GROUND, OUTPUT);
    digitalWrite(SPEK_BIND_GROUND, LOW);
    pinMode(SPEK_BIND_POWER, INPUT);
    digitalWrite(SPEK_BIND_POWER, LOW);
    pinMode(SPEK_BIND_POWER, OUTPUT);

    while (1)   //Do not return.  User presses reset button to return to normal.
    {
        blinkLED(4, 255, 1);
        digitalWrite(SPEK_BIND_POWER, LOW); // Power off sat
        pinMode(SPEK_BIND_DATA, OUTPUT);
        digitalWrite(SPEK_BIND_DATA, LOW);
        delay(1000);
        blinkLED(4, 255, 1);
        digitalWrite(SPEK_BIND_POWER, HIGH); // Power on sat
        delay(10);
        digitalWrite(SPEK_BIND_DATA, HIGH);
        delay(60);                 // Keep data pin steady for 20 to 120ms after power up
        noInterrupts();

        for (byte i = 0; i < SPEK_BIND_PULSES; i++)
        {
            digitalWrite(SPEK_BIND_DATA, LOW);
            delayMicroseconds(118);
            digitalWrite(SPEK_BIND_DATA, HIGH);
            delayMicroseconds(122);
        }

        interrupts();
        delay(60000);         //Allow one full minute to bind, then try again.
    }
}

#endif
