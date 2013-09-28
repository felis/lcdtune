/* HD44780-compatible LCD timing tuning routine. Coded for PIC24F MCU with EPMP peripheral */
/* uses external 12MHz crystal to produce 32MHz clock */
/* Many other oscillators can be used with coresponding fuse change */
/* uses timer interrupt to consume the LCD queue */
/* compiled/tested using Microchip C30 compiler ver.3.31 with no optimization */
/* The LCD pin to MCU pin assignment identical to Microchip Explorer 16 board */

#define FCY 16000000UL	//system clock speed - crystal speed/2

#include <p24FJ256GB206.h>    //MCU-specific header
#include <libpic30.h>					//delays

#define uint8_t unsigned char

/* PIC fuses */
_CONFIG1
(
   JTAGEN_OFF     // JTAG Disabled
   & GCP_OFF      // Code Protect Disabled
   & GWRP_OFF     // Write Protect Disabled
   & FWDTEN_OFF   // Watchdog Timer Disabled
   & WINDIS_OFF   // Windowed Watchdog Timer Disabled
)

_CONFIG2
(
   IESO_ON          // Two Speed Start-up
   & PLL96MHZ_ON   // 96MHz PLL Disabled
   //& PLLDIV_DIV2    //input 8Mhz clock divided by 2 to get 4MHz
   & PLLDIV_DIV3		//input 12Mhz clock divided by 2 to get 4MHz
   & FNOSC_PRIPLL  	//primary clock with PLL
   & POSCMOD_HS 		//High-speed crystal mode
)

//timer period for fast and slow commands
#define BSP_TMR3_PER_SHORT	799     //Timer period for fast commands
//#define BSP_TMR3_PER_SHORT 2000
#define BSP_TMR3_PER_LONG		35000   //Timer3 period for slow commands
//#define BSP_TMR3_PER_LONG 55000

//LCD buffer size - must be power of 2
#define LCD_TX_BUFSIZE 256
#define LCD_TX_BUFMASK ( LCD_TX_BUFSIZE - 1 )

#if ( LCD_TX_BUFSIZE & LCD_TX_BUFMASK )
#error LCD Tx Buffer size is not a power of 2
#endif

//LCD buffer
uint8_t LcdTx_Buf[LCD_TX_BUFSIZE];
uint8_t LcdTx_Head;
volatile uint8_t LcdTx_Tail;

#define CS_BASE 0x20000UL   //start address for LCD registers in EDS
#define CMDFLAG 0x0f        //a character code inserted before a command in LCD queue

//address allocation for LCD registers
__eds__ uint8_t __attribute__((noload, section("epmp_cs1"), address(CS_BASE))) LCDCMD __attribute__((space(eds)));
__eds__ uint8_t __attribute__((noload, section("epmp_cs1"), address(CS_BASE))) LCDALIGN __attribute__((space(eds)));
__eds__ uint8_t __attribute__((noload, section("epmp_cs1"), address(CS_BASE))) LCDDATA __attribute__((space(eds)));

// commands
#define LCD_CLEARDISPLAY        0x01
#define LCD_RETURNHOME          0x02
#define LCD_ENTRYMODESET        0x04
#define LCD_DISPLAYCONTROL      0x08
#define LCD_CURSORSHIFT         0x10
#define LCD_FUNCTIONSET         0x20
#define LCD_SETCGRAMADDR        0x40
#define LCD_SETDDRAMADDR        0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT          0x00
#define LCD_ENTRYLEFT           0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON           0x04
#define LCD_DISPLAYOFF          0x00
#define LCD_CURSORON            0x02
#define LCD_CURSOROFF           0x00
#define LCD_BLINKON             0x01
#define LCD_BLINKOFF            0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE         0x08
#define LCD_CURSORMOVE          0x00
#define LCD_MOVERIGHT           0x04
#define LCD_MOVELEFT            0x00

// flags for function set
#define LCD_8BITMODE            0x10
#define LCD_4BITMODE            0x00
#define LCD_2LINE               0x08
#define LCD_1LINE               0x00
#define LCD_5x10DOTS            0x04
#define LCD_5x8DOTS             0x00

//Timer interrupt
#define TIMER3_ISR_PRIO 1
void  __attribute__((__interrupt__, auto_psv)) _T3Interrupt(void)
{
 static uint8_t state = 0;

    _T3IF = 0;    //clear interrupt flag

    LcdTx_Tail++;

#if LCD_TX_BUFMASK < 255
        LcdTx_Tail &= LCD_TX_BUFMASK;
#endif

	switch( state ) {

  	case 0:    //read byte, send data

    	if( LcdTx_Buf[ LcdTx_Tail ] == CMDFLAG ) {    //next byte is a command

     		TMR3 = PR3 - 20;    //shorter cycle. Must be set longer than the execution time of the rest of the ISR

        state = 1;
      }//if( LcdTx_Buf[ LcdTx_Tail ] == CMDFLAG...
      else {

      	LCDDATA = LcdTx_Buf[ LcdTx_Tail ];    //send data

      	PR3 = BSP_TMR3_PER_SHORT;
      }
      break;

  case 1:    //send command

  	LCDCMD = LcdTx_Buf[ LcdTx_Tail ];    //send command

    	if( LcdTx_Buf[ LcdTx_Tail ] < 4 ) {  //slow command

      	PR3 = BSP_TMR3_PER_LONG;

      }
      else {

      	PR3 = BSP_TMR3_PER_SHORT;

      }

      state = 0;

      break;

    }//switch( state...

    if( LcdTx_Head == LcdTx_Tail ) {    //stop the timer

        T3CONbits.TON = 0;

    }
}

void MCU_init(void) {
	
	//REFOCONbits.ROEN = 1;  //uncomment to observe clock output on RS - breaks LCD interface

	/* All pins output */
 	// PIC24FJ256GB206 doesn't have port A
 	TRISB = 0;
  TRISC = 0;
  TRISD = 0;
  TRISE = 0;
  TRISF = 0;
  TRISG = 0;
  
  /* Turn off analog */
	ANSB = 0;
  ANSC = 0;
  ANSD = 0;

  ANSF = 0;
  ANSG = 0;
  
  
  //reset LCD buffer head and tail
  LcdTx_Head = 0;    
  LcdTx_Tail = 0;

	//Setup timer 3 for LCD
  T3CON  = 0x0000;                     /* Use Internal Osc (Fcy), 16 bit mode, no prescaler */
  PR3    = BSP_TMR3_PER_SHORT;         /* set the period */
  TMR3   = PR3 - 1;                    /* one count before interrupt */
  _T3IP  = TIMER3_ISR_PRIO;            /* set Timer 3 interrupt priority */
  _T3IF  = 0;                          /* clear the interrupt for Timer 2 */
  _T3IE  = 1;                          /* enable interrupt for Timer 2 */
  //we don't want to start this timer
  
  // register bits are set separately for better presentation
  // in case space is needed they can be combined in a single assignment per register
  PMCON1bits.ADRMUX = 0;			// address is not multiplexed
  PMCON1bits.MODE = 3;        // master mode
  PMCON1bits.CSF = 0;         // PMCS1 pin used for chip select 1, PMCS2 pin used for chip select 2
  PMCON1bits.ALMODE = 0;      // "smart" address strobes are not used
  PMCON1bits.BUSKEEP = 0;     // bus keeper is not used
  PMCON1bits.IRQM = 0;        //interrupt at the end of of rd/wr cycle
  /**/
  PMCON3bits.PTWREN = 1;      // enable write(rd/WR) strobe port
  PMCON3bits.PTRDEN = 1;      // enable read(enable) strobe port
  PMCON3bits.AWAITM = 0;      // set address latch pulses width to 1/2 Tcy
  PMCON3bits.AWAITE = 0;      // set address hold time to 1/4 Tcy
  /**/
  PMCON4 = 0x0001;            // PMA0 address line is enabled
  /**/
  PMCS1CFbits.CSDIS = 0;      // enable CS function
  PMCS1CFbits.CSP = 1;     		// CS1 polarity
  PMCS1CFbits.CSPTEN = 0;     // disable CS port
  PMCS1CFbits.BEP = 1;     		// byte enable polarity
  PMCS1CFbits.WRSP = 1;    		// write strobe polarity - enable active high
  PMCS1CFbits.RDSP =1;     		// read strobe polarity, READ high, WRITE low
  PMCS1CFbits.SM = 1;      		// read/write and enable strobes
  PMCS1CFbits.PTSZ = 0;    		// data bus width is 8 bit
  /**/
  PMCS1BS = (CS_BASE>>8);     // CS1 start address
  /**/
  PMCS1MDbits.ACKM = 0;       // PMACK is not used
  // The device timing parameters. Set the proper timing
  // according to the device used (the timing macros are defined in the hardware profile)
  //time in Tcy. One cycle is 62.5ns for 32MHz clock
  PMCS1MDbits.DWAITB = 3;      // time from RS,RW to E
  PMCS1MDbits.DWAITM = 0x08;   //E strobe length - 450ns by spec
  PMCS1MDbits.DWAITE = 3;			 //time from E to valid data
  /**/
	PMCON1bits.PMPEN = 1;        // enable the module
}

/* Places a byte to the LCD queue. Can be used to send data */
void LcdSendByte(uint8_t byte) {
	uint8_t tmphead = LcdTx_Head + 1;

#if LCD_TX_BUFMASK < 255
	tmphead &= LCD_TX_BUFMASK;
#endif

	while( tmphead == LcdTx_Tail );	//this line blocks - keep buffer large enough

  LcdTx_Buf[ tmphead ] = byte;

  LcdTx_Head = tmphead;

  T3CONbits.TON = 1;    //start the timer in case it was stopped
}

/* Places a command flag to the LCD queue followed by a byte */
void LcdSendCmd(uint8_t cmd) {
	
	LcdSendByte( CMDFLAG );   //insert command flag symbol
  LcdSendByte( cmd );    		
  
}

int main( void )
{
//initialization commands for standard 16x2 LCD
#define FUNC_SET  LCD_FUNCTIONSET|LCD_8BITMODE|LCD_2LINE|LCD_5x8DOTS
#define DISP_CTRL LCD_DISPLAYCONTROL|LCD_DISPLAYON|LCD_CURSOROFF|LCD_BLINKOFF
#define ENTRY_MODE LCD_ENTRYMODESET|LCD_ENTRYLEFT

 const uint8_t lcd_init_seq[] = { FUNC_SET, DISP_CTRL, LCD_CLEARDISPLAY, ENTRY_MODE, 0 };	//initialization sequence
 const uint8_t* lcd_init_p = lcd_init_seq;	//pointer to the first element
 const uint8_t rollchar[4] = {'/','-','\\','|'};
 uint8_t roll_idx = 0;
#define ROLL_IDX_MASK 0x03
 
	MCU_init();
	
	while( *lcd_init_p ) {	//power-on display initialization
		
		__delay_ms( 30 );
		
		LCDCMD = *lcd_init_p++;	//place a byte directly on the LCD bus
	
	}
	
	while( 1 ) {	//output rolling characters in the first 4 posirions of the display
		
		uint8_t i;
		
		LcdSendCmd( LCD_RETURNHOME ); //Home the screen - slow command
		
		for( i = 0; i < 4; i++ ) {
		
			LcdSendByte( rollchar[ roll_idx ] ); //fast command
			
		}
		
		roll_idx++;
		roll_idx &= ROLL_IDX_MASK;
		
		__delay_ms( 1000 );
		
	}//while( 1 )
}//main
	