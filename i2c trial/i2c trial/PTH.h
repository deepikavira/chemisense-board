
/*
	GY-26 digital compass library.
	(c) 13 September 2013 by Vassilis Serasidis
	Home: http://www.serasidis.gr
	e-mail: avrsite@yahoo.gr , info@serasidis.gr
	
	This library is distributed under GPL v3 licence.
*/


#define		I2C_PTH	0x76	// pressure and temp default I2C address.
#include	"i2cmaster.h"


/*=================================================================================

unsigned int degrees;
degrees = pthRead();

The value in 'degrees' variable could be from 0 to 3600.
If the 'degrees' variable has the value 2568 that means 256.8 degrees.
=================================================================================*/
int pthRead (void);
void pth_init (void);