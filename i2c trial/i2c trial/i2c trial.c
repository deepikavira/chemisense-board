
/*
 * i2c_trial.c
 *
 * Created: 6/30/2015 10:15:16 AM
 *  Author: deepikavira
 */ 
#define F_CPU 3200000UL
#define USART_BAUDRATE 9600s
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
//#define SCL_CLOCK  100000 //100 kHz

//#define TWI_MASTER_PORT PORTC
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>
#include <util/twi.h>
//#include "adc.h"
//#include "board.h"
#include "i2cmaster.h"





static inline void init_oscillator() {
	// enable 32Mhz internal oscillator
	OSC.CTRL |= OSC_RC32MEN_bm;
	// wait for it to be stable
	while (!(OSC.STATUS & OSC_RC32MRDY_bm));
	// tell the processor we want to change a protected register
	CCP=CCP_IOREG_gc;
	// and start using the 32Mhz oscillator
	CLK.CTRL=CLK_SCLKSEL_RC32M_gc;
	// disable the default 2Mhz oscillator
	OSC.CTRL&=(~OSC_RC2MEN_bm);
	// enable 32kHz calibrated internal oscillator
	OSC.CTRL|= OSC_RC32KEN_bm;
	while (!(OSC.STATUS & OSC_RC32KRDY_bm));
	// set bit to 0 to indicate we use the internal 32kHz
	// callibrated oscillator as auto-calibration source
	// for our 32Mhz oscillator
	OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc;
	// enable auto-calibration for the 32Mhz oscillator
	DFLLRC32M.CTRL |= DFLL_ENABLE_bm;
}



static inline void init_interrupts() {
	// Enable PMIC interrupt level low
	PMIC.CTRL |= PMIC_LOLVLEX_bm;
	// enable interrupts
	sei();
}

void sendChar(char c)
{
	
	USARTD0.DATA = c;
	if(!(USARTD0.STATUS & USART_DREIF_bm)) {
		while(!(USARTD0.STATUS & USART_TXCIF_bm)); // wait for TX complete
	}
	USARTD0.STATUS |= USART_TXCIF_bm;
	
}

void sendString(char *text)
{
	
	while(*text)
	{
		sendChar(*text++);
	}
}


static inline void init_usart() {
	
	PORTD.DIRSET = PIN7_bm;         //set means output
	//PORTD.OUTSET = PIN7_bm;
	// remap USARTD0 to PD[7-4]
	PORTD.REMAP |= PORT_USART0_bm;
	// set baud rate 9600: BSEL=12, BSCALE=4
	// as found in table in
	// Atmel-42005-8-and-16-bit-AVR-Microcontrollers-XMEGA-E_Manual.pdf
	USARTD0.BAUDCTRLA = 12; // BSEL
	USARTD0.BAUDCTRLB = 4 << USART_BSCALE_gp; // BSCALE
	// disable 2X
	USARTD0.CTRLB = USARTD0.CTRLB & ~USART_CLK2X_bm;
	// enable RX and TX
	USARTD0.CTRLB = USARTD0.CTRLB | USART_RXEN_bm | USART_TXEN_bm;
	// enable async UART 8N1
	USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	USARTD0.CTRLC &= ~USART_SBMODE_bm;
	USARTD0.CTRLD = 0; // No LUT

	// set interrupt level for RX
	USARTD0.CTRLA = (USARTD0.CTRLA & ~USART_RXCINTLVL_gm) | USART_RXCINTLVL_LO_gc;
}

static inline void init_twi() 
{
	 TWIC_MASTER_BAUD  = 155;
	 PORTC.PIN0CTRL = PORT_OPC_WIREDANDPULL_gc;
	 PORTC.PIN1CTRL = PORT_OPC_WIREDANDPULL_gc;
	 TWIC_MASTER_CTRLA |= TWI_MASTER_INTLVL_MED_gc| TWI_MASTER_RIEN_bm |
	 TWI_MASTER_WIEN_bm | TWI_MASTER_ENABLE_bm; //0x68; //ENABLES THE READ INTERRUPT and also enables the master mode
	 TWIC_MASTER_STATUS  = TWI_MASTER_BUSSTATE_IDLE_gc| 0X04;  // CLEAR THE BUS ERROR	
	 TWIC_MASTER_CTRLB |= 0x01;  
	 TWIC_MASTER_CTRLC =   TWI_MASTER_CMD_RECVTRANS_gc; //TWI_MASTER_CMD_STOP_gc //needs to be changed everytime you begin a new communication
	//PORTCFG.CLKOUT = (PORTCFG.CLKOUT & ~PORTCFG_CLKOUTSEL_gm) | PORTCFG_CLKOUT_PC1_gc; //trying to get clk on pc1
		 
	//TWIC_MASTER_CTRLC = TWI_MASTER_CMD_RECVTRANS_gc;  //read data
	// if((MASTER.STATUS & TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_IDLE_gc) //check if bus is idle
	/* Enable configured PMIC interrupt level. */

//	PMIC.CTRL |= CONF_PMIC_INTLVL;

		
   // PORTC.DIR |= PIN1_bm;	
	//PORTC.DIR |= PIN0_bm;	
	//	PRGEN register 
}

ISR(TWIC_TWIM_vect)
{
char DATA=TWIC_MASTER_DATA;
sendChar(DATA);
}

int main(void)
{
	SREG = 0x80;	/*ENABLE GLOBAL INTERRUPTS*/
	init_oscillator();
	init_usart();
	init_twi();
 //TWI_MASTER_t twi;
 //PMIC_CTRL= //interrupt
 
	PORTC.DIR |= PIN6_bm;
	
	if(( TWIC_MASTER_STATUS & TWI_MASTER_BUSSTATE_gm) == TWI_MASTER_BUSSTATE_IDLE_gc)
{	
	//TWIC_MASTER_ADDR =0x00; 
	TWIC_MASTER_CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	//TWIC_MASTER_CTRLC = TWI_MASTER_CMD_NOACT_gc;   //read data
	TWIC_MASTER_ADDR = 0xEC;  // R/W bit low to write the reg number from where we want to read
	TWIC_MASTER_ADDR = 0xA0;
	TWIC_MASTER_CTRLC = TWI_MASTER_CMD_REPSTART_gc;
	TWIC_MASTER_ADDR = 0xED; // R/W bit high indicating a read operation 
    TWIC_MASTER_CTRLC = TWI_MASTER_CMD_STOP_gc;
	 int DATA=TWIC_MASTER_DATA;
	//sendChar(DATA);
	uint16_t buffer;
	itoa(DATA, buffer, 10);
	sendString(buffer);
	sendString("\n");
	
}

		//while(TWIC_MASTER_STATUS & 0x10)  //ack/NACK recieved from slave
 
//    TWIC_MASTER_ADDR =0x00; 
//    _delay_ms(100);
//    TWIC_MASTER_ADDR = 0xE7;
//      TWIC_MASTER_ADDR = 0xE5;
// 	 _delay_ms(100);
// TWIC_MASTER_ADDR = 0xA0;
    while(1)
	//for(int i=1; i<=5 ; i++)
    {
    
	 PORTC.OUT &= ~PIN6_bm;
  //  sendString("Test\n");
	_delay_ms(1000);
	PORTC.OUT |= PIN6_bm;
	_delay_ms(1000);
 
   //TWIC_MASTER_CTRLC = TWI_MASTER_CMD_REPSTART_gc;

   int DATA=TWIC_MASTER_DATA;
	//sendChar(DATA);
	uint16_t buffer;
	itoa(DATA, buffer, 10);
//	sendString(buffer);
    }
}











