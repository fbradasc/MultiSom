/**************************************************************************************/
/***********************       NMEA                          **************************/
/**************************************************************************************/

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output NMEA frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     - GPS altitude
     - GPS speed
 */
#define FRAME_GGA  1
#define FRAME_RMC  2

void GPS_SerialInit(void)
{
    SerialOpen(GPS_SERIAL, GPS_BAUD);
    delay(1000);
}

bool GPS_newFrame(uint8_t c)
{
    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, frame = 0;

    if (c == '$')
    {
        param = 0;
        offset = 0;
        parity = 0;
    }
    else
    if ( ( c == ',') || ( c == '*') )
    {
        string[offset] = 0;

        if (param == 0)   //frame identification
        {
            frame = 0;

            if ( (string[0] == 'G') && (string[1] == 'P') && (string[2] == 'G') && (string[3] == 'G') && (string[4] == 'A') )
            {
                frame = FRAME_GGA;
            }

            if ( (string[0] == 'G') && (string[1] == 'P') && (string[2] == 'R') && (string[3] == 'M') && (string[4] == 'C') )
            {
                frame = FRAME_RMC;
            }
        }
        else
        if (frame == FRAME_GGA)
        {
            if (param == 2)
            {
                GPS_coord[LAT] = GPS_coord_to_degrees(string);
            }
            else
            if ( ( param == 3) && ( string[0] == 'S') )
            {
                GPS_coord[LAT] = -GPS_coord[LAT];
            }
            else
            if (param == 4)
            {
                GPS_coord[LON] = GPS_coord_to_degrees(string);
            }
            else
            if ( ( param == 5) && ( string[0] == 'W') )
            {
                GPS_coord[LON] = -GPS_coord[LON];
            }
            else
            if (param == 6)
            {
                f.GPS_FIX = (string[0] > '0');
            }
            else
            if (param == 7)
            {
                GPS_numSat = grab_fields(string, 0);
            }
            else
            if (param == 9)
            {
                GPS_altitude = grab_fields(string, 0);   // altitude in meters added by Mis
            }
        }
        else
        if (frame == FRAME_RMC)
        {
            if (param == 7)
            {
                GPS_speed = ( (uint32_t)grab_fields(string, 1) * 5144L ) / 1000L;   //gps speed in cm/s will be used for navigation
            }
            else
            if (param == 8)
            {
                GPS_ground_course = grab_fields(string, 1);    //ground course deg*10
            }
        }

        param++;
        offset = 0;

        if (c == '*')
        {
            checksum_param = 1;
        }
        else
        {
            parity ^= c;
        }
    }
    else
    if ( ( c == '\r') || ( c == '\n') )
    {
        if (checksum_param)   //parity checksum
        {
            uint8_t checksum = hex_c(string[0]);
            checksum <<= 4;
            checksum += hex_c(string[1]);

            if (checksum == parity)
            {
                frameOK = 1;
            }
        }

        checksum_param = 0;
    }
    else
    {
        if (offset < 15)
        {
            string[offset++] = c;
        }

        if (!checksum_param)
        {
            parity ^= c;
        }
    }

    return(frameOK && (frame == FRAME_GGA) );
}
