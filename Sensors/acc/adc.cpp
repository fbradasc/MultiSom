// ************************************************************************************************************
// ADC ACC
// ************************************************************************************************************
    void
ACC_init ()
{
    pinMode (A1, INPUT);
    pinMode (A2, INPUT);
    pinMode (A3, INPUT);
}

    void
ACC_getADC ()
{
    ACC_ORIENTATION (analogRead (A1), analogRead (A2), analogRead (A3));
    ACC_Common ();
}
