//ADC all four channel reading fine. Ch gain is div by 2  if we keep it to mul by 1 then all the sensors read wired values.

#define F_CPU 32000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>


#define BSCALE -5      // Gives a baudrate of 115200 @ 8 MHz
#define BSEL 107

char channel_no[16];
volatile uint16_t ch_no;  //VARIABLES FOR CH NO

static void usart_init(void)
{
	// Init pins for uart
	PORTD.PIN6CTRL = PORT_OPC_TOTEM_gc;
	PORTD.PIN7CTRL = PORT_OPC_TOTEM_gc;
	PORTD.OUTSET = PIN7_bm;         // TX pin high
	PORTD.DIRSET = PIN7_bm;         // TX pin output
	PORTD.DIRCLR = PIN6_bm;         // RX pin input
	PORTD.REMAP = PORT_USART0_bm;   // Move USART to high bits (XMEGA-E5 Xplained board)

	// Init usart. Setup for 115200 8n1. Does not use interrupts
	USARTD0.BAUDCTRLB = (BSCALE << USART_BSCALE_gp) | ((BSEL >> 8) & ~USART_BSCALE_gm);
	USARTD0.BAUDCTRLA = (BSEL & USART_BSEL_gm);
	USARTD0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	USARTD0.CTRLD = USART_LUTACT_OFF_gc | USART_PECACT_OFF_gc;
	USARTD0.CTRLB = USART_RXEN_bm | USART_TXEN_bm;
	USARTD0.CTRLA = USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
}


static int usart_putchar(char character, FILE *stream)
{
	if (character == '\n')
	{
		usart_putchar('\r', stream);    // Insert CR before LF
	}
	while (!(USARTD0.STATUS & USART_DREIF_bm));     // Wait for USART to be ready for a new char to send
	USARTD0.DATA = character;
	return 0;
}


static void dac_init(void)
{
	DACA.CTRLA = DAC_IDOEN_bm | DAC_ENABLE_bm;      // Enable DAC0 and provide output internally
	DACA.CTRLB = DAC_CHSEL_SINGLE_gc;               // Single channel only (DAC0)
	DACA.CTRLC = DAC_REFSEL_INT1V_gc;               // Use internal 1V reference
}


static void dac_write(uint16_t value)
{
	DACA.CH0DATA = (value <= 0x0fff) ? value : 0x0fff;
}


static void adc_init(void)
{
// 	 ADCA.CH0.CORRCTRL=0x01;
// 	 ADCA.CH0.OFFSETCORR0=0xFF;
// 	ADCA.CH0.OFFSETCORR1=0x02;
// 	ADCA.CH0.CORRCTRL=0x00;
	
	ADCA.CTRLA = ADC_ENABLE_bm;                     // Enable ADC
	ADCA.CTRLB = ADC_CURRLIMIT_NO_gc |ADC_RESOLUTION_12BIT_gc;  // No power saving, signed singleended mode, oversampling resolution
	ADCA.REFCTRL =  ADC_REFSEL_INTVCC_gc;// ADC_REFSEL_INT1V_gc;             // Use internal 1V reference (same as DAC)
	ADCA.EVCTRL =   ADC_EVACT_NONE_gc; // ADC_EVSEL_3_gc;//|ADC_EVSEL_2_gc| ADC_EVSEL_3_gc| ADC_EVSEL_4_gc| ADC_EVACT_SYNCSWEEP_gc ;// ADC_EVACT_NONE_gc;                // Do not use events
	ADCA.PRESCALER = ADC_PRESCALER_DIV8_gc;         // ADC clock = 1 MHz
	ADCA.CH0.CTRL =  ADC_CH_GAIN_DIV2_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;//ADC_CH_INPUTMODE_INTERNAL_gc;       // No gain, use internal input (from DAC)
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN1_gc;// ADC_CH_MUXPOS_PIN1_gc;//|ADC_CH_MUXPOS_PIN2_gc; //ADC_CH_MUXINT_DAC_gc;//  // Use DAC as input to ADC
	ADCA.CH0.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;   // No interrupts
	ADCA.CH0.SCAN = 0x0B;                              // No channel scanning

	/* This is the interesting stuff, select the average/oversampling you want to test */
	ADCA.CH0.AVGCTRL = ADC_SAMPNUM_1X_gc;                                       // No averaging, no oversampling (change the resolution to 12 bit in CTRLB above)
	//ADCA.CH0.AVGCTRL = ADC_SAMPNUM_4X_gc | (1 << ADC_CH_RIGHTSHIFT_gp);         // Oversampling to 13-bit resolution
	//ADCA.CH0.AVGCTRL = ADC_SAMPNUM_16X_gc | (2 << ADC_CH_RIGHTSHIFT_gp);        // Oversampling to 14-bit resolution
	//ADCA.CH0.AVGCTRL = ADC_SAMPNUM_64X_gc | (1 << ADC_CH_RIGHTSHIFT_gp);        // Oversampling to 15-bit resolution
	//ADCA.CH0.AVGCTRL = ADC_SAMPNUM_256X_gc | (0 << ADC_CH_RIGHTSHIFT_gp);       // Oversampling to 16-bit resolution
	//ADCA.CH0.AVGCTRL = ADC_SAMPNUM_32X_gc;                                     // Averaging only
 
}
static uint16_t calibrateADC(uint16_t data){
	
}


static uint16_t adc_read(void)
{
	ADCA.CH0.INTFLAGS = ADC_CH_IF_bm;               // Clear ADC interrupt flag
	ADCA.CH0.CTRL |= ADC_CH_START_bm;               // Start ADC single conversion
	while (!(ADCA.CH0.INTFLAGS & ADC_CH_IF_bm));    // Wait for ADC to complete
	return ADCA.CH0.RES;                            // Return result
}




// Console USART stream
FILE usart_stream = FDEV_SETUP_STREAM(usart_putchar, NULL, _FDEV_SETUP_WRITE);

int main(void)
{
	stdout = &usart_stream; // Used for printf

	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = CLK_PSADIV_4_gc | CLK_PSBCDIV_1_1_gc;  // Set prescale dividers to 4:1:1
	OSC.CTRL |= OSC_RC32KEN_bm | OSC_RC32MEN_bm;        // Enable internal 32 MHZ and 32.768 kHz oscillators
	while (!(OSC.STATUS & OSC_RC32MRDY_bm));            // Wait for 32 MHz to stabilize
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;
	while (!(OSC.STATUS & OSC_RC32KRDY_bm));            // Wait for 32.768 kHz to stabilize
	OSC.DFLLCTRL = OSC_RC32MCREF_RC32K_gc;              // Set DFLL reference to internal 32.768 kHz osc
	DFLLRC32M.CTRL = DFLL_ENABLE_bm;                    // Enable DFLL
	OSC.CTRL &= ~(OSC_RC8MLPM_bm | OSC_RC8MEN_bm | OSC_PLLEN_bm | OSC_XOSCEN_bm | OSC_RC2MEN_bm);   // Disable unused oscillators

	// XMega is now running 8 MHz with DFLL correction
	
	usart_init();
	dac_init();
	adc_init();
	PORTD.DIR |= PIN5_bm;
	//ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN12_gc;
	
	uint16_t res; 
	
	printf_P(PSTR("ADC readings:\n"));
	
	//for (uint16_t k=0; k<100; k++)
	while(1)
	{ 
		
   //  printf_P(PSTR("%d",dustPin);
      PORTD.OUT &= ~PIN5_bm;
		_delay_us(320);
	   res = adc_read();  
	 _delay_us(320);
	  PORTD.OUT |= PIN5_bm;
	  ch_no= ADCA_CH0_SCAN;
	  printf_P(PSTR("the mics adc sensor %u:%u\n"), ch_no, res); 
      }
	
}
