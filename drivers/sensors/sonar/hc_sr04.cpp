#if defined(IMPLEMENTATION)
// ************************************************************************************************************
// Sonar HC-SR04 support
// ************************************************************************************************************
volatile unsigned long SONAR_GEP_startTime = 0;
volatile unsigned long SONAR_GEP_echoTime = 0;
volatile static int32_t tempSonarAlt = 0;

void Sonar_init()
{
    SONAR_GEP_EchoPin_PCICR;
    SONAR_GEP_EchoPin_PCMSK;
    SONAR_GEP_EchoPin_PINMODE_IN;
    SONAR_GEP_TriggerPin_PINMODE_OUT;
}

uint8_t Sonar_update()
{
    sonarAlt = 1 + tempSonarAlt;
    SONAR_GEP_TriggerPin_PIN_LOW;
    delayMicroseconds( 2);
    SONAR_GEP_TriggerPin_PIN_HIGH;
    delayMicroseconds(10);
    SONAR_GEP_TriggerPin_PIN_LOW;
    return(sonarAlt);
}

ISR(SONAR_GEP_EchoPin_PCINT_vect)
{
    if (SONAR_GEP_EchoPin_PIN & (1 << SONAR_GEP_EchoPin_PCINT) )
    {
        SONAR_GEP_startTime = micros();
    }
    else
    {
        SONAR_GEP_echoTime = micros() - SONAR_GEP_startTime;

        if (SONAR_GEP_echoTime <= SONAR_GENERIC_MAX_RANGE * SONAR_GENERIC_SCALE)
        {
            tempSonarAlt = SONAR_GEP_echoTime / SONAR_GENERIC_SCALE;
        }
        else
        {
            tempSonarAlt = -1;
        }
    }
}
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
