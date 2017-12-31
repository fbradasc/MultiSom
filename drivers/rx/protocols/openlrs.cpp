//RAW RC values will be store here
static uint8_t rcChannel[RC_CHANS] = {ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4};

/**************************************************************************************/
/***************          combine and sort the RX Datas            ********************/
/**************************************************************************************/

uint16_t readRawRC(uint8_t chan)
{
    if (chan < RC_CHANS)
    {
        data = rcData[rcChannel[chan]];
    }
    else
    {
        data = 1500;
    }

    return data; // We return the value correctly copied when the IRQ's where disabled
}

/**************************************************************************************/
/***************                     OPENLRS                       ********************/
/**************************************************************************************/

//note: this dont feels right in RX.pde

// **********************************************************
// ******************   OpenLRS Rx Code   *******************
// ***  OpenLRS Designed by Melih Karakelle on 2010-2012  ***
// **  an Arudino based RC Rx/Tx system with extra futures **
// **       This Source code licensed under GPL            **
// **********************************************************
// Version Number     : 1.11
// Latest Code Update : 2012-03-25
// Supported Hardware : OpenLRS Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************
// # PROJECT DEVELOPERS #
// Melih Karakelle (http://www.flytron.com) (forum nick name: Flytron)
// Jan-Dirk Schuitemaker (http://www.schuitemaker.org/) (forum nick name: CrashingDutchman)
// Etienne Saint-Paul (http://www.gameseed.fr) (forum nick name: Etienne)
//

//######### TRANSMISSION VARIABLES ##########
#define CARRIER_FREQUENCY 435000  // 435Mhz startup frequency
#define FREQUENCY_HOPPING 1 // 1 = Enabled  0 = Disabled

//###### HOPPING CHANNELS #######
//Select the hopping channels between 0-255
// Default values are 13,54 and 23 for all transmitters and receivers, you should change it before your first flight for safety.
//Frequency = CARRIER_FREQUENCY + (StepSize(60khz)* Channel_Number)

static uint8_t hop_list[3]  = HOPLIST;//{13,54,23};
//###### RF DEVICE ID HEADERS #######
// Change this 4 byte values for isolating your transmission, RF module accepts only data with same header
static uint8_t RF_Header[4] = OLRS_HEADER;//{'O','L','R','S'};

//########## Variables #################

static uint32_t last_hopping_time;
static uint8_t RF_Rx_Buffer[17];
static uint16_t temp_int, rx_rssi;
static uint16_t Servo_Buffer[10] = {3000,3000,3000,3000,3000,3000,3000,3000};   //servo position values from RF
static uint8_t hopping_channel = 1;


// **********************************************************
// **      RFM22B/Si4432 control functions for OpenLRS     **
// **       This Source code licensed under GPL            **
// **********************************************************
// Latest Code Update : 2011-09-26
// Supported Hardware : OpenLRS Tx/Rx boards (store.flytron.com)
// Project Forum      : http://forum.flytron.com/viewforum.php?f=7
// Google Code Page   : http://code.google.com/p/openlrs/
// **********************************************************

//*****************************************************************************
//*****************************************************************************
unsigned char ItStatus1, ItStatus2;

//--------------------------------------------------------------
void Write0( void )
{
    SCK_off;
    NOP();
    SDI_off;
    NOP();
    SCK_on;
    NOP();
}

//--------------------------------------------------------------
void Write1( void )
{
    SCK_off;
    NOP();
    SDI_on;
    NOP();
    SCK_on;
    NOP();
}

//--------------------------------------------------------------
void Write8bitcommand(uint8_t command)  // keep sel to low
{
    uint8_t n=8;
    nSEL_on;
    SCK_off;
    nSEL_off;
    while (n--)
    {
        if (command&0x80)
        {
            Write1();
        }
        else
        {
            Write0();
        }
        command = command << 1;
    }
    SCK_off;
}

//--------------------------------------------------------------
void send_read_address(uint8_t i)
{
    i &= 0x7f;
    Write8bitcommand(i);
}

//--------------------------------------------------------------
void send_8bit_data(uint8_t i)
{
    uint8_t n = 8;
    SCK_off;
    while (n--)
    {
        if (i&0x80)
        {
            Write1();
        }
        else
        {
            Write0();
        }
        i = i << 1;
    }
    SCK_off;
}
//--------------------------------------------------------------

uint8_t read_8bit_data(void)
{
    uint8_t Result, i;

    SCK_off;
    Result=0;

    for (i=0;i<8;i++)                    //read fifo data byte
    {
        Result=Result<<1;
        SCK_on;
        NOP();
        if(SDO_1)
        {
            Result|=1;
        }
        SCK_off;
        NOP();
    }
    return(Result);
}

//--------------------------------------------------------------
uint8_t _spi_read(uint8_t address)
{
    uint8_t result;
    send_read_address(address);
    result = read_8bit_data();
    nSEL_on;
    return(result);
}

//--------------------------------------------------------------
void _spi_write(uint8_t address, uint8_t data)
{
    address |= 0x80;
    Write8bitcommand(address);
    send_8bit_data(data);
    nSEL_on;
}

//-------Defaults 38.400 baud----------------------------------------------
void RF22B_init_parameter(void)
{
    ItStatus1 = _spi_read(0x03); // read status, clear interrupt
    ItStatus2 = _spi_read(0x04);
    _spi_write(0x06, 0x00);    // no wakeup up, lbd,
    _spi_write(0x07, RF22B_PWRSTATE_READY);      // disable lbd, wakeup timer, use internal 32768,xton = 1; in ready mode
    _spi_write(0x09, 0x7f);  // c = 12.5p
    _spi_write(0x0a, 0x05);
    _spi_write(0x0b, 0x12);    // gpio0 TX State
    _spi_write(0x0c, 0x15);    // gpio1 RX State
    _spi_write(0x0d, 0xfd);    // gpio 2 micro-controller clk output
    _spi_write(0x0e, 0x00);    // gpio    0, 1,2 NO OTHER FUNCTION.
    _spi_write(0x70, 0x00);    // disable manchest

    // 57.6Kbps data rate
    _spi_write(0x1c, 0x05); // case RATE_57.6K
    _spi_write(0x20, 0x45);//  0x20 calculate from the datasheet= 500*(1+2*down3_bypass)/(2^ndec*RB*(1+enmanch))
    _spi_write(0x21, 0x01); // 0x21 , rxosr[10--8] = 0; stalltr = (default), ccoff[19:16] = 0;
    _spi_write(0x22, 0xD7); // 0x22    ncoff =5033 = 0x13a9
    _spi_write(0x23, 0xDC); // 0x23
    _spi_write(0x24, 0x03); // 0x24
    _spi_write(0x25, 0xB8); // 0x25
    _spi_write(0x2a, 0x1e);

    _spi_write(0x6e, 0x0E); //case RATE_57.6K
    _spi_write(0x6f, 0xBF); //case RATE_57.6K

    _spi_write(0x30, 0x8c);    // enable packet handler, msb first, enable crc,

    _spi_write(0x32, 0xf3);    // 0x32address enable for headere byte 0, 1,2,3, receive header check for byte 0, 1,2,3
    _spi_write(0x33, 0x42);    // header 3, 2, 1,0 used for head length, fixed packet length, synchronize word length 3, 2,
    _spi_write(0x34, 0x07);    // 7 default value or   // 64 nibble = 32byte preamble
    _spi_write(0x36, 0x2d);    // synchronize word
    _spi_write(0x37, 0xd4);
    _spi_write(0x38, 0x00);
    _spi_write(0x39, 0x00);
    _spi_write(0x3a, RF_Header[0]);    // tx header
    _spi_write(0x3b, RF_Header[1]);
    _spi_write(0x3c, RF_Header[2]);
    _spi_write(0x3d, RF_Header[3]);
    _spi_write(0x3e, 17);    // total tx 17 byte

    //RX HEADER
    _spi_write(0x3f, RF_Header[0]);   // check hearder
    _spi_write(0x40, RF_Header[1]);
    _spi_write(0x41, RF_Header[2]);
    _spi_write(0x42, RF_Header[3]);
    _spi_write(0x43, 0xff);    // all the bit to be checked
    _spi_write(0x44, 0xff);    // all the bit to be checked
    _spi_write(0x45, 0xff);    // all the bit to be checked
    _spi_write(0x46, 0xff);    // all the bit to be checked

    _spi_write(0x6d, 0x07); // 7 set power max power
    _spi_write(0x79, 0x00);    // no hopping
    _spi_write(0x7a, 0x06);    // 60khz step size (10khz x value) // no hopping

    _spi_write(0x71, 0x23); // Gfsk, fd[8] =0, no invert for Tx/Rx data, fifo mode, txclk -->gpio
    //_spi_write(0x72, 0x1F); // frequency deviation setting to 19.6khz (for 38.4kbps)
    _spi_write(0x72, 0x2E); // frequency deviation setting to 28.8khz(for 57.6kbps)
    _spi_write(0x73, 0x00);
    _spi_write(0x74, 0x00);    // no offset

    //band 435.000
    _spi_write(0x75, 0x53);
    _spi_write(0x76, 0x7D);
    _spi_write(0x77, 0x00);
}

#if !defined(OPENLRS_V2)
void checkPots()
{
    ////Flytron OpenLRS Multi Pots
    pot_P = analogRead(7);
    pot_I = analogRead(6);

    pot_P = pot_P - 512;
    pot_I = pot_I - 512;

    pot_P = pot_P / 25; //+-20
    pot_I = pot_I / 25; //+-20
}
#endif

void configureReceiver(void)
{
    pinMode(GREEN_LED_pin, OUTPUT);
    pinMode(RED_LED_pin, OUTPUT);

    //RF module pins
    pinMode(SDO_pin, INPUT); //SDO
    pinMode(SDI_pin, OUTPUT); //SDI
    pinMode(SCLK_pin, OUTPUT); //SCLK
    pinMode(IRQ_pin, INPUT); //IRQ
    pinMode(nSel_pin, OUTPUT); //nSEL
#if !defined(OPENLRS_V2)
    checkPots(); // OpenLRS Multi board hardware pot check;
#endif
}

//-----------------------------------------------------------------------
void rx_reset(void)
{
    _spi_write(0x07, RF22B_PWRSTATE_READY);
    _spi_write(0x7e, 36);    // threshold for rx almost full, interrupt when 1 byte received
    _spi_write(0x08, 0x03);    //clear fifo disable multi packet
    _spi_write(0x08, 0x00);    // clear fifo, disable multi packet
    _spi_write(0x07,RF22B_PWRSTATE_RX );  // to rx mode
    _spi_write(0x05, RF22B_Rx_packet_received_interrupt);
    ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register
    ItStatus2 = _spi_read(0x04);
}
//-----------------------------------------------------------------------

//--------------------------------------------------------------
void to_ready_mode(void)
{
    ItStatus1 = _spi_read(0x03);
    ItStatus2 = _spi_read(0x04);
    _spi_write(0x07, RF22B_PWRSTATE_READY);
}

void to_rx_mode(void)
{
    to_ready_mode();
    delay(50);
    rx_reset();
    NOP();
}

//--------------------------------------------------------------
void to_sleep_mode(void)
{
    //  TXEN = RXEN = 0;
    //LED_RED = 0;
    _spi_write(0x07, RF22B_PWRSTATE_READY);

    ItStatus1 = _spi_read(0x03);  //read the Interrupt Status1 register
    ItStatus2 = _spi_read(0x04);
    _spi_write(0x07, RF22B_PWRSTATE_POWERDOWN);
}
//--------------------------------------------------------------

void frequency_configurator(uint32_t frequency)
{
    // frequency formulation from Si4432 chip's datasheet
    // original formulation is working with mHz values and floating numbers, I replaced them with kHz values.
    frequency = frequency / 10;
    frequency = frequency - 24000;
    frequency = frequency - 19000; // 19 for 430-439.9 MHz band from datasheet
    frequency = frequency * 64; // this is the Nominal Carrier Frequency (fc) value for register setting

    uint8_t byte0 = (uint8_t) frequency;
    uint8_t byte1 = (uint8_t) (frequency >> 8);

    _spi_write(0x76, byte1);
    _spi_write(0x77, byte0);
}

//############# FREQUENCY HOPPING FUNCTIONS #################
#if (FREQUENCY_HOPPING==1)
void Hopping(void)
{
    hopping_channel++;
    if (hopping_channel>2)
    {
        hopping_channel = 0;
    }

    _spi_write(0x79, hop_list[hopping_channel]);
}
#endif

void Config_OpenLRS()
{
    RF22B_init_parameter(); // Configure the RFM22B's registers
    frequency_configurator(CARRIER_FREQUENCY); // Calibrate the RFM22B to this frequency, frequency hopping starts from here.
    to_rx_mode();
#if (FREQUENCY_HOPPING==1)
    Hopping(); //Hop to the next frequency
#endif
}

//############ MAIN LOOP ##############
/**************************************************************************************/
/***************          compute and Filter the RX data           ********************/
/**************************************************************************************/

void computeRC()
{
    uint8_t i,tx_data_length;
    uint8_t first_data = 0;
    //rcData[RX_RSSI_CHAN] =0;
    //rx_rssi=0;

    if (_spi_read(0x0C)==0) // detect the locked module and reboot
    {
        RF22B_init_parameter();
        to_rx_mode();
    }

    if ((currentTime-last_hopping_time > 25000)) //automatic hopping for clear channel when rf link down for 25ms.
    {
        Red_LED_ON;
        last_hopping_time = currentTime;
#if (FREQUENCY_HOPPING==1)
        Hopping(); //Hop to the next frequency
#endif
    }

    if(nIRQ_0) // RFM22B INT pin Enabled by received Data
    {
        Red_LED_ON;
        send_read_address(0x7f); // Send the package read command

        for(i = 0; i<17; i++) //read all buffer
        {
            RF_Rx_Buffer[i] = read_8bit_data();
        }

#if defined(RX_RSSI_CHAN)
        rx_rssi =  _spi_read(0x26); // Read the RSSI value
        rcData[RX_RSSI_CHAN] =   map(constrain(rx_rssi,45,120),40,120,0,2000);
#endif

        rx_reset();

        if (RF_Rx_Buffer[0] == 'S') // servo control data
        {
            for (i = 0; i<8; i++) //Write into the Servo Buffer
            {
                temp_int = (256*RF_Rx_Buffer[1+(2*i)]) + RF_Rx_Buffer[2+(2*i)];

                if ((temp_int>1500) && (temp_int<4500))
                {
                    Servo_Buffer[i] = temp_int/2;
                }
            }

            rcData[ROLL] = Servo_Buffer[0];
            rcData[PITCH] = Servo_Buffer[1];
            rcData[THROTTLE] = Servo_Buffer[2];
            rcData[YAW] = Servo_Buffer[3];
            rcData[AUX1] = Servo_Buffer[4];
            rcData[AUX2] = Servo_Buffer[5];
            rcData[AUX3] = Servo_Buffer[6];
            rcData[AUX4] = Servo_Buffer[7];

#if defined(FAILSAFE)
            if (failsafeCnt > 20) // Valid frame, clear FailSafe counter
            {
                failsafeCnt -= 20;
            }
            else
            {
                failsafeCnt = 0;
            }
#endif
        }
#if (FREQUENCY_HOPPING==1)
        Hopping(); //Hop to the next frequency
#endif
        delay(1);
        last_hopping_time = currentTime;
        Red_LED_OFF;
    }

    Red_LED_OFF;
}
