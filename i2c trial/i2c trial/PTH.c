



#include	"PTH.h"

/*=================================================================================
Reads the GY-26 digital compass and returns the degrees value in integer format.
Example:

unsigned int degrees;
degrees = pthRead();

The value in 'degrees' variable could be from 0 to 3650.
If the 'degrees' variable has the value 2568 that means 256.8 degrees.
=================================================================================*/
int pthRead (void)
{
		unsigned int data;
		
		i2c_start(I2C_GY26 + I2C_READ);	// Set device address and read mode		
		data = (i2c_readAck() << 8);	// Read the Most Significant Byte (MSB) from the GY-26 compass. 
		data += i2c_readNak();			// Read the Low Significant Byte (LSB) from the GY-26 compass. 
		i2c_stop();
		
		return (data); //Return the degrees value (16-bit integer from 0-3650).
}

void pth_init (void)
{
	i2c_init();
}	