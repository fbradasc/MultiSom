## Ardhat MultiSom UUV Navigation Controller 


is a MultiWii 2.4 firmware derivative for ATmega328 et al, with some additional features:

1. Connects to Cleanflight browser configurator (does not support CLI or servos tab)

2. Supports Ardhat board type

2. Supports Invensys MPU9250 combo IMU

3. Supports generic sonar module such as HC-SR04, SRF04, DYP-ME007
	Check config.h SECTION 9 for PIN assignments
	
4. Inflight PID tuning.
	This requires AUX2(3 way switch) and AUX3(Potentio). 
		AUX2(3 way switch) - Is used to adjust from P, I, or D. 
		- P is the LO switch, I CEnter, and D is the HIghest switch
		AUX3(Potentio) - Is used to adjust the values of P, I, and D.
	This will have anew boxname called "PID Tune"
	Check the sample video here https://www.youtube.com/watch?v=lVwk3JNQx6g
	Check config.h SECTION 9

5. Support for MultiWii Mobile Control for Windows Phone.
	This has new MSP called MSP_SUPRESS_DATA_FROM_RX = 150
 Once enabled in Windows Phone application, it will suppress all data coming in from your receiver so you can use the application in Windows Phone to control the quad with its virtual joystick with 4 AUX channels.
	Here's the sample video https://youtu.be/tptUuW6d01U
	blog: http://jaysonsblog.azurewebsites.net/post/multiwii-mobile-control-for-windows-phone
	discussion: http://www.multiwii.com/forum/viewtopic.php?f=8&t=6322&sid=78edcfaf125c4a9f48736237c399bf0b

6. Can skip GYRO calibration at startup
	Make sure to calibrate the GYRO after you update the firmware.
	Configurable via config.h
	Check config.h SECTION 9
	
7. PID Controller
	PID controller codes are now in separate .cpp file called "PIDControllers.cpp" for future PID controllers
	
----------------------------------------------------------------------------------

This is a development branch merged with multiwii-2.4-fixedwing which is a branch
of MultiWii 2.4 forked by Martin Espinoza <martin.espinoza@gmail.com>

It is forked from a fork by PatrikE for FixedWing RTH Version 2015-04-31
For almost all relevant information, see
http://fotoflygarn.blogspot.com/2014/04/multiwii-gps-airplane.html

This fork includes SD logging based on wareck's Multiwii_Nav_V2.3 fork 
https://github.com/wareck/Multiwii_Nav_V2.3
But it uses the current, modern SdFat interface.

Use with WinGui from EosBandi for setting waypoints.
http://eosbandi.com/downloads/

Since my TX (HK-T6A V2) is impossible to calibrate properly, I have included
PWM scaling factor code from http://mifi.no/blog/?p=95
However I applied one factor to all sticks, and inserted the code into a
different location which I hope will work for all RX except OLRS. Use the
end point values in the TX to make all channels output the same maximums,
then set the STICK_SCALING_FACTOR in config.h. I am using 1.15.

----------------------------------------------------------------------------------

All credits to the MultiWii developers and programmers, and specially Nullstr1ng for the Cleanflight port.
