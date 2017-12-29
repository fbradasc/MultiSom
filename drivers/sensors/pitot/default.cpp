#if defined(IMPLEMENTATION)
/* Inspired by ArduPilot code. Thanks guys! */

    void
Pitot_read (float &whereto)
{
    //  whereto = ((analogRead(PITOT_PIN) / 1024.0f) - 0.5f ) / 2.0f  +  whereto * 0.9f ;  // do not multiply by 5, assume 0.2 ratio for current reading
    whereto = ((analogRead (PITOT_PIN) / 1024.0f) - 0.5f) + whereto * 0.8f;	// faster value update
}				//Pitot_read

    void
Pitot_update ()
{
    Pitot_read (pitotRaw);
    if (pitotRaw <= pitotOffset)
        pitotSpeed = 0;
    else
        pitotSpeed = sqrt ((pitotRaw - pitotOffset) * PITOT_FACTOR) * 100;	// m/s * 100 = cm/s
}				//Pitot_update

    void
Pitot_init ()
{
    /* Reading pitotRaw for some cycles/1s and storing as zero value */
    for (uint8_t pitotI = 0; pitotI < 100; pitotI++)
    {
        delay (10);
        Pitot_read (pitotOffset);
    }
}				// Pitot_init
#else // IMPLEMENTATION
#endif // IMPLEMENTATION
