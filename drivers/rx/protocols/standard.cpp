/**************************************************************************************/
/***************             Global RX related variables           ********************/
/**************************************************************************************/

//RAW RC values will be store here
volatile uint16_t rcValue[RC_CHANS] = {1502, 1502, 1502, 1502, 1502, 1502, 1502, 1502}; // interval [1000;2000]

static uint8_t rcChannel[RC_CHANS] = {ROLLPIN, PITCHPIN, YAWPIN, THROTTLEPIN, AUX1PIN, AUX2PIN, AUX3PIN, AUX4PIN};
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS}; // if this slowes the PCINT readings we can switch to a define for each pcint bit

void rxInt(void);

/**************************************************************************************/
/***************                   RX Pin Setup                    ********************/
/**************************************************************************************/
void configureReceiver()
{
    /******************    Configure each rc pin for PCINT    ***************************/
#if defined(MEGA)
    DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
#endif

    // PCINT activation
    for (uint8_t i = 0; i < PCINT_PIN_COUNT; i++) // i think a for loop is ok for the init.
    {
        PCINT_RX_PORT |= PCInt_RX_Pins[i];
        PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }

    PCICR = PCIR_PORT_BIT;
    /*************    atmega328P's Specific Aux2 Pin Setup    *********************/
#if defined(PROMINI)
# if defined(RCAUXPIN)
    PCICR |= (1 << 0);   // PCINT activated also for PINS [D8-D13] on port B
#  if defined(RCAUXPIN8)
    PCMSK0 = (1 << 0);
#  else // defined(RCAUXPIN12)
    PCMSK0 = (1 << 4);
#  endif
# endif
#endif
    /***************   atmega32u4's Specific RX Pin Setup   **********************/
#if defined(PROMICRO)
    //Trottle on pin 7
    DDRE &= ~(1 << 6); // pin 7 to input
    PORTE |= (1 << 6); // enable pullups
    EICRB |= (1 << ISC60);
    EIMSK |= (1 << INT6); // enable interuppt
    // Aux2 pin on PBO (D17/RXLED)
# if defined(RCAUX2PIND17)
    DDRB &= ~(1 << 0); // set D17 to input
# endif
    // Aux2 pin on PD2 (RX0)
# if defined(RCAUX2PINRXO)
    DDRD &= ~(1 << 2); // RX to input
    PORTD |= (1 << 2); // enable pullups
    EICRA |= (1 << ISC20);
    EIMSK |= (1 << INT2); // enable interuppt
# endif
#endif
}

/**************************************************************************************/
/***************               Standard RX Pins reading            ********************/
/**************************************************************************************/

/* stick scaling http://mifi.no/blog/?p=95 */
#if defined(FAILSAFE) && !defined(PROMICRO)
// predefined PC pin block (thanks to lianj)  - Version with failsafe
# define RX_PIN_CHECK(pin_pos, rc_value_pos)                                   \
    if (mask & PCInt_RX_Pins[pin_pos])                                         \
    {                                                                          \
        if (!(pin & PCInt_RX_Pins[pin_pos]) )                                  \
        {                                                                      \
            dTime = cTime - edgeTime[pin_pos];                                 \
            if (900 < dTime && dTime < 2200)                                   \
            {                                                                  \
                rcValue[rc_value_pos] = dTime;                                 \
                if ( (rc_value_pos == THROTTLEPIN || rc_value_pos == YAWPIN || \
                      rc_value_pos == PITCHPIN || rc_value_pos == ROLLPIN)     \
                     && dTime > FAILSAFE_DETECT_TRESHOLD )                     \
                {                                                              \
                    GoodPulses |= (1 << rc_value_pos);                         \
                }                                                              \
            }                                                                  \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            edgeTime[pin_pos] = cTime;                                         \
        }                                                                      \
    }

#else
// predefined PC pin block (thanks to lianj)  - Version without failsafe
# define RX_PIN_CHECK(pin_pos, rc_value_pos)   \
    if (mask & PCInt_RX_Pins[pin_pos])         \
    {                                          \
        if (!(pin & PCInt_RX_Pins[pin_pos]) )  \
        {                                      \
            dTime = cTime - edgeTime[pin_pos]; \
            if (900 < dTime && dTime < 2200)   \
            {                                  \
                rcValue[rc_value_pos] = dTime; \
            }                                  \
        }                                      \
        else                                   \
        {                                      \
            edgeTime[pin_pos] = cTime;         \
        }                                      \
    }

#endif

// port change Interrupt
ISR(RX_PC_INTERRUPT) //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
{
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime, dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;

#if defined(FAILSAFE) && !defined(PROMICRO)
    static uint8_t GoodPulses;
#endif
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
    mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
#if (PCINT_PIN_COUNT > 0)
    RX_PIN_CHECK(0, 2);
#endif
#if (PCINT_PIN_COUNT > 1)
    RX_PIN_CHECK(1, 4);
#endif
#if (PCINT_PIN_COUNT > 2)
    RX_PIN_CHECK(2, 5);
#endif
#if (PCINT_PIN_COUNT > 3)
    RX_PIN_CHECK(3, 6);
#endif
#if (PCINT_PIN_COUNT > 4)
    RX_PIN_CHECK(4, 7);
#endif
#if (PCINT_PIN_COUNT > 5)
    RX_PIN_CHECK(5, 0);
#endif
#if (PCINT_PIN_COUNT > 6)
    RX_PIN_CHECK(6, 1);
#endif
#if (PCINT_PIN_COUNT > 7)
    RX_PIN_CHECK(7, 3);
#endif
#if defined(FAILSAFE) && !defined(PROMICRO)
    if (GoodPulses == (1 << THROTTLEPIN) + (1 << YAWPIN) + (1 << ROLLPIN) + (1 << PITCHPIN) ) // If all main four chanells have good pulses, clear FailSafe counter
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
/*********************      atmega328P's Aux2 Pins      *************************/
#if defined(PROMINI)
# if defined(RCAUXPIN)
/* this ISR is a simplification of the previous one for PROMINI on port D
   it's simplier because we know the interruption deals only with one PIN:
   bit 0 of PORT B, ie Arduino PIN 8
   or bit 4 of PORTB, ie Arduino PIN 12
   => no need to check which PIN has changed */
ISR(PCINT0_vect)
{
    uint8_t pin;
    uint16_t cTime, dTime;
    static uint16_t edgeTime;

    pin = PINB;
    cTime = micros();
    sei();
#  if defined(RCAUXPIN8)
    if (!(pin & 1 << 0) )   //indicates if the bit 0 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
#  else // defined(RCAUXPIN12)
    if (!(pin & 1 << 4) )   //indicates if the bit 4 of the arduino port [B0-B7] is not at a high state (so that we match here only descending PPM pulse)
#  endif
    {
        dTime = cTime - edgeTime;

        if ( (900 < dTime) && (dTime < 2200) ) // just a verification: the value must be in the range [1000;2000] + some margin
        {
            rcValue[0] = dTime;
        }
    }
    else
    {
        edgeTime = cTime;    // if the bit 2 is at a high state (ascending PPM pulse), we memorize the time
    }
}
# endif
#endif

/****************      atmega32u4's Throttle & Aux2 Pin      *******************/
#if defined(PROMICRO)
// throttle
ISR(INT6_vect)
{
    static uint16_t now, diff;
    static uint16_t last = 0;

    now = micros();

    if (!(PINE & (1 << 6) ) )
    {
        diff = now - last;

        if ( (900 < diff) && (diff < 2200) )
        {
            rcValue[3] = diff;
# if defined(FAILSAFE)
            if (diff > FAILSAFE_DETECT_TRESHOLD)      // if Throttle value is higher than FAILSAFE_DETECT_TRESHOLD
            {
                if (failsafeCnt > 20)
                {
                    failsafeCnt -= 20;
                }
                else
                {
                    failsafeCnt = 0;    // If pulse present on THROTTLE pin (independent from ardu version), clear FailSafe counter  - added by MIS
                }
            }
# endif
        }
    }
    else
    {
        last = now;
    }
}
// Aux 2
# if defined(RCAUX2PINRXO)
ISR(INT2_vect)
{
    static uint16_t now, diff;
    static uint16_t last = 0;

    now = micros();

    if (!(PIND & (1 << 2) ) )
    {
        diff = now - last;

        if ( (900 < diff) && (diff < 2200) )
        {
            rcValue[7] = diff;
        }
    }
    else
    {
        last = now;
    }
}
# endif
#endif

/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/

uint16_t readRawRC(uint8_t chan)
{
    uint16_t data;
    uint8_t oldSREG;

    oldSREG = SREG;
    cli(); // Let's disable interrupts
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
        {
            rcDataTmp = readRawRC(chan);
        }
#else
        rcDataTmp = readRawRC(chan);
#endif
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
        if (!supress_data_from_rx)
        {
        }
        else
        if (supress_data_from_rx)
        {
            //if (chan<8 && rcSerialCount > 0) // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
            if (chan < 8) // rcData comes from MSP and overrides RX Data until rcSerialCount reaches 0
            {
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
