/*
** PAL Video using T3 and Output Compare interrupts
**
** Graphic page 
**
** Modified to produce properly sized and spaced pre-and post equalization pulses as well as vertical sync pulses
*/

#include <p33fj256gp710.h>
#include "PCM_X2.h"

// I/O definitions
#define SYNC    _LATG0  // output 
#define SDO     _RF8    // SPI1 SDO

// timing definitions for PAL video vertical state machine--------------------------------------------------------------------------------------------------
#define V_PAL   625    	// total number of lines composing a frame

#define VRES_1	VRES	// Total number of lines FIRST FIELD, active lines only
#define VRES_2	VRES	// Total number of lines SECOND FIELD, active lines only

#define VSYNC_N1  2     // V sync lines FIRST FIELD
#define VSYNC_N2  2     // V sync lines SECOND FIELD

#define VBLANK_N1  9  	// Number of blank lines FIRST FIELD
#define VBLANK_N2  9  	// Number of blank lines SECOND FIELD

#define PREEQ_N1   2    // pre equalization FIRST FIELD, 
#define PREEQ_N2   3    // pre equalization SECOND FIELD, odd
#define POSTEQ_N1  2    // post equalization FIRST FIELD, 
#define POSTEQ_N2  2    // post equalization SECOND FIELD, 

#define HALF_LINE_N1	1	// valid value is ONE, line 3 of 625
#define HALF_LINE_N2	1	// valid value is ONE, line 313 of 625

// definition of the sync state machine---------------------------------------------------------------------------------------------------------------------
#define SV_SYNC_1     	1
#define HALF_LINE_1		2
#define SV_POSTEQ_1   	3
#define SV_LINE_1     	4
#define SV_BLANKLINE_1	5		
#define SV_PREEQ_1   	6
#define HALF_LINE_2		7
#define SV_SYNC_2     	8
#define SV_POSTEQ_2   	9
#define SV_LINE_2     	10
#define SV_BLANKLINE_2	11
#define SV_PREEQ_2   	0

// timing definitions for PAL video horizontal state machine------------------------------------------------------------------------------------------------
#define H_PAL  			2559    // total number of Tcy in a line (64us), adjusted for latency {{triggers EXACTLY every 64us}}
#define HSYNC_T  		188     // Tcy in a horizontal synch pulse used during each scan line and vertical sync
#define BPORCH_T 		225     // Tcy in a back porch 
#define SHORT_SYNC		95		// Tcy,duration of short sync pulses used during pre-and post equalization
#define FRONT_PORCH_T	66		// Tcy in a front porch, 1.65us
#define PIX_T    		8      	// Tcy per pixel/bit

// timing definitions for external data clock switching pulse ----------------------------------------------------------------------------------------------
#define OC5RISE			(HSYNC_T+BPORCH_T)
#define OC5FALL			(H_PAL-FRONT_PORCH_T)
#define OC5PERIOD		H_PAL					//equal to or greater than value in OC5FALL

//Tcy = 40 MIPS Check configuration bits
//----------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------

#define _FAR __attribute__(( far))

//circular buffers
unsigned _FAR int FIFO[FIFO_SIZE]; 			//buffer holding samples

//FIFO write and read pointers
volatile int FIFO_W,FIFO_R;			 		//both FIFO_LEFT and FIFO_RIGHT will use the same pointers to make sure they remain synchronized
volatile int CHECKSUM;				 		//Checksum value recorded at the end of each line, calculated using the values in that line

//FIFO Housekeeping
volatile int sample_count;					//if sample count is zero, it means there are no available samples to read out.
											// sample_count++ every time you FIFO_W++
											// sample_count-- every time you FIFO_R++
volatile int inside_line_flag;				// 1 --> Busy writing a line
											// 0 --> Not Busy writing a line

//SPI unload slot counter
volatile int SPI_slot_counter;

volatile int *VPtr;
volatile int HCount, VCount, VState, HState;

// next state table
int VS[12] = { SV_SYNC_1,	HALF_LINE_1,	SV_POSTEQ_1,	SV_LINE_1,	SV_BLANKLINE_1,	SV_PREEQ_1,	HALF_LINE_2,	SV_SYNC_2,	SV_POSTEQ_2,	SV_LINE_2,	SV_BLANKLINE_2,	SV_PREEQ_2 	};
// next counter table
int VC[12] = { VSYNC_N1,	HALF_LINE_N1 ,	POSTEQ_N1, 		VRES_1, 	VBLANK_N1,		PREEQ_N1,	HALF_LINE_N2,	VSYNC_N2,	POSTEQ_N2,		VRES_2,		VBLANK_N2,		PREEQ_N2	};

void _ISRFAST _T3Interrupt( void)
{
    // Start a Synch pulse
	// SV_LINE_2 and SV_LINE_1 should be called 294 times per field to set up the SPI loads, 6 DATA, 1 clock, 1 CHECKSUM 
    SYNC = 0;

    // decrement the vertical counter
    VCount--;
    
    // vertical state machine
    switch ( VState) {

		case SV_SYNC_1://FIELD 1 - Lines 1,2
            // vertical sync pulse - OK
            OC3R = (H_PAL/2) - HSYNC_T;
            OC3CON = 0x0009;    // single event 

			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = H_PAL - HSYNC_T;		//
            OC2CON = 0x0009;    		// single event 
            break;

 		case HALF_LINE_1://FIELD 1 - Line 3
           // HALF LINE - OK
            OC3R = (H_PAL/2)- HSYNC_T;
            OC3CON = 0x0009;    // single event 
			
			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = (H_PAL/2) + SHORT_SYNC;	//
            OC2CON = 0x0009;    			// single event 
            break;

 		case SV_POSTEQ_1://FIELD 1 - Lines 4,5
            // Post equalization pulse - OK
            OC3R = SHORT_SYNC;
            OC3CON = 0x0009;    //single event 
			
			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = (H_PAL/2) + SHORT_SYNC;	// Relative to Timer 3
            OC2CON = 0x0009;    			// single event 
         	break;


	 	case SV_LINE_1://FIELD 1 - Lines used for DATA------------------------------------
            // horizontal sync pulse - OK
            OC3R = HSYNC_T;
            OC3CON = 0x0009;    // single event 

            // activate OC4 for the SPI loading
            OC4R = HSYNC_T + BPORCH_T;
            OC4CON = 0x0009;    // single event 
			
            HCount = HRES/16;  // loads 16 bits at a time  
            break; //---------------------------------------------------------------------

		case SV_BLANKLINE_1://FIELD 1 - Lines vary
           // blank line - OK
            OC3R = HSYNC_T;
            OC3CON = 0x0009;    // single event 

			//INSERT CODE HERE FOR Fs CLOCK SWITCHING SETUP
			//OC5 single pulse relative to timer 3, OCxR, OCxRS, PRy
			OC5R = OC5RISE;		//Time to rising edge relative to TMR3
			OC5RS = OC5FALL;	//Time to falling edge relative to TMR3
			PR5 = OC5PERIOD;

			OC5CON = 0x000C;    // single pulse

            break;
     
        case SV_PREEQ_1://FIELD 1 - Lines 311,312
           	// Pre equalization pulse - OK
            OC3R = SHORT_SYNC;
            OC3CON = 0x0009;    //single event 
			
			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = (H_PAL/2) + SHORT_SYNC;	//Relative to Timer 3
            OC2CON = 0x0009;    			// single event 

         	break;
            
        case HALF_LINE_2://FIELD 1 - Line 313
            // Half Line - OK
            OC3R = SHORT_SYNC;
            OC3CON = 0x0009;    // single event 
			
			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = H_PAL - HSYNC_T;	//
            OC2CON = 0x0009;    		// single event 
		
            break;//	

  		case SV_SYNC_2://FIELD 2 - Lines 314,315
           // horizontal sync pulse - OK
           	OC3R = (H_PAL/2) - HSYNC_T;
            OC3CON = 0x0009;    // single event 

			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = H_PAL - HSYNC_T;		//
            OC2CON = 0x0009;    		// single event 
            break;

		case SV_POSTEQ_2://FIELD 2 - Lines 316,317
           	// Post equalization pulse - OK
            OC3R = SHORT_SYNC;
            OC3CON = 0x0009;    //single event 
			
			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = (H_PAL/2) + SHORT_SYNC;	// Relative to Timer 3
            OC2CON = 0x0009;    			// single event 
         	break;

		case SV_BLANKLINE_2://FIELD 2 - Lines vary
			// blank line - OK
            OC3R = HSYNC_T;
            OC3CON = 0x0009;    // single event 

			//INSERT CODE HERE FOR Fs CLOCK SWITCHING SETUP
			//OC5 single pulse relative to timer 3, OCxR, OCxRS, PRy
			OC5R = OC5RISE;		//Time to rising edge relative to TMR3
			OC5RS = OC5FALL;	//Time to falling edge relative to TMR3
			PR5 = OC5PERIOD;

			OC5CON = 0x000C;    // single pulse	

            break;
            
		case SV_PREEQ_2://FIELD 2 - Lines 623-625
           // Pre equalization pulse - OK
            OC3R = SHORT_SYNC;
            OC3CON = 0x0009;    //single event 
			
			OC1R = (H_PAL/2);	//
            OC1CON = 0x0009;    // single event 

			OC2R = (H_PAL/2) + SHORT_SYNC;	// Relative to Timer 3
            OC2CON = 0x0009;    			// single event 

            break;

        default:            
        case SV_LINE_2://FIELD 2 - Lines used for DATA------------------------------------
            // horizontal sync pulse - OK
            OC3R = HSYNC_T;
            OC3CON = 0x0009;    // single event 

            // activate OC4 for the SPI loading
            OC4R = HSYNC_T + BPORCH_T;
            OC4CON = 0x0009;    // single event 
            HCount = HRES/16;  // loads 16 bits at a time 
			//HCount = 7;
            break; //---------------------------------------------------------------------                                       
    } //switch

    // advance the state machine
    if ( VCount == 0)
    {
		VCount = VC[ VState];
        VState = VS[ VState];
     }

    // clear the interrupt flag
    _T3IF = 0;
} // T3Interrupt

//-------------------------------------------INTERRUPT ROUTINES----------------------------------------
void _ISRFAST _OC1Interrupt( void)//modified*
{
    SYNC = 0;   // bring the output down to SYNC level
    _OC1IF = 0; // clear the interrupt flag

} // OC1Interrupt

void _ISRFAST _OC2Interrupt( void)//modified*
{
    SYNC = 1;   // bring the output up to the black level
    _OC2IF = 0; // clear the interrupt flag

} // OC2Interrupt

void _ISRFAST _OC3Interrupt( void)
{
    SYNC = 1;   // bring the output up to the black level
    _OC3IF = 0; // clear the interrupt flag

} // OC3Interrupt

void _ISRFAST _OC4Interrupt( void) // loads SPI FIFO with 1 x 16-bit word = 16 pixels
{    
//	_RA0 = 1;

	if ((sample_count > (FIFO_SIZE/2)) || (inside_line_flag == 1)) //if the buffer has enough data OR you're allready building a line then go ahead
	{//-------------------	
		//if there are not enough samples at the beginning of a line to start construction of a new line then creation of a
		//new line should be locked out untill the next time you start a line

		if (FIFO_R == FIFO_SIZE)
				FIFO_R = 0;

		switch ( SPI_slot_counter ) {
		case 0:
  				SPI1BUF = 0b1010101010101100;
				inside_line_flag = 1;			//started building data line
  		break;
		case 7:
  				SPI1BUF = CHECKSUM;   			// Slot 7 is on far right of TV screen
				CHECKSUM = 0;		  			//Checksum cleared for use in next line
				inside_line_flag = 0;			//finished building data line
  		break;
		default:
 			//load sample into SPI buffer
			SPI1BUF = FIFO[FIFO_R]; 
			sample_count--;		//sample removed from buffer
			//UPDATE CHECKSUM
			CHECKSUM = FIFO[FIFO_R] + CHECKSUM;  //calculate the SUM of CHECKSUM and current sample, store in CHECKSUM
			//increment READ POINTER
			FIFO_R++;
  		break;
	}//switch

		if ( --HCount > 0)
   		{   // activate again in time for the next SPI load , if you NEVER entered this routine at the beginning of a line then it won't activate for the next slot
            OC4R += ( PIX_T *2* 15); //This is a rough fix, SPI is reloaded earlier every sample slot. Seems to work OK
            OC4CON = 0x0009;    // single event                 
		}//NOTE : There was a clock discontinuity between the end of the clock-run-in and the first sample
		SPI_slot_counter++;

		if (SPI_slot_counter == 8)
			SPI_slot_counter=0;      	
		
	}//if------------------------

	// clear the interrupt flag
	_OC4IF = 0;
//	_RA0 = 0;

} // OC4Interrupt

void _ISRFAST _OC5Interrupt( void) //used for switching (Fs x 64)clock from DIR9001 in and out of video signal after data lines
{								   //single pulse relative to Timer 3	
								   //set up OC5 for SINGLE PULSE generation	
    _OC5IF = 0; // clear the interrupt flag

} // OC5Interrupt

void __attribute__((__interrupt__)) _DCIInterrupt(void)//DCI Interrupt handler recieves samples from RXBUF and places them in the FIFO
{													   //confirmed as triggering at 88.2 kHz by measurement
	_RA1 = 1;
		
	FIFO[ FIFO_W]= RXBUF0; 
	sample_count++;					//an extra sample has been added to the buffer			
	
	if ((FIFO_W+1)%FIFO_SIZE != FIFO_R)		//check if buffer is full
		{	
			FIFO_W++;						//else increment the write pointer
			
		}
		FIFO_W %= FIFO_SIZE;				//wrap around
		
		IFS3bits.DCIIF=0;  					//DCIIF: DCI Event Interrupt Flag Status bit, clear the Interrupt flag
											//Check if interrupt priority is set
	_RA1 = 0;

} // DCI Interrupt

//-------------------------------------------END OF INTERRUPT ROUTINES----------------------------------------
//------------------------------------------------------------------------------------------------------------
//---------------------------------------START OF INITIALIZATION ROUTINES-------------------------------------
void initVideo( void)
{
	//Set Oscillator Configuration
	OSCCON = 0x0300;		//
	CLKDIV = 0x0000;		//
	PLLFBD = 0x0037;		// 128 x Fs = 5.6448MHZ --> 40.2192MIPS
	
    // set the priority levels
	_DCIIP = 3;		// priority for DCI module
	_T3IP  = 4; 	// this is the default value anyway
	_OC1IP = 4;		//modified*
    _OC2IP = 4;		//modified*
	_OC3IP = 4;
    _OC4IP = 4;
	_OC5IP = 4;

	TMR3 = 0;		// clear the timer
	PR3 = H_PAL;	// set the period register to PAL line 
	
	// 2.1 configure Timer3 modules
	T3CON = 0x8000;	// enabled, prescaler 1:1, internal clock 	
	
// init Timer1/3/OC3/OC4 Interrupts, clear the flag

	_OC1IF = 0;	_OC1IE = 0;		
    _OC2IF = 0; _OC2IE = 0;		
	_OC3IF = 0;	_OC3IE = 0;
    _OC4IF = 0; _OC4IE = 0;		//OC1,2,3 and 4 interrupts remains disabled on program startup untill FIFO buffer is half full
	_OC5IF = 0; _OC5IE = 0;
    _T3IF  = 0; _T3IE  = 0;		//T3 interrupt remains disabled untill FIFO buffer is half full
	_T2IF  = 0; _T2IE  = 1;
    	
	// 2.3 init the processor priority level
	_IPL = 0;	// this is the default value anyway

    // init the SPI1 
 	
	SPI1CON1 = 0x143D;		// Primary Prescale 16:1,Secondary prescale 1:1,Master Mode ENABLE,16 bit mode, / 16 ; 10000111101

    SPI1CON2 = 0x0000;      // 
    SPI1STAT = 0x8000;      // enable SPI port
    
    // init PORTF for the Sync
    _TRISG0 = 0;            // output the SYNC pin
    	
	// init the vertical sync state machine
	VState = SV_PREEQ_2;
    VCount = PREEQ_N2;

	CHECKSUM = 0;			//Clear the CHECKSUM value
	SPI_slot_counter = 0; 	//Initialize the SPI_slot_counter to ZERO
    
} // initVideo

void haltVideo( void)
{
    T3CONbits.TON = 0;   // turn off the vertical state machine
} //haltVideo

void synchV( void) 
{
    while ( VCount != 1);
} // synchV

void initFIFO( void)
{	//reset circular buffer pointers
	FIFO_W = 0;
	FIFO_R = 0;
	sample_count = 0;

	int i;

	//fill the FIFO buffer with zeros
	for ( i=0; i<FIFO_SIZE; i=i+1)
	{
	FIFO[ FIFO_W] = 0; //write in the sample buffer

	FIFO_W = i;
	}
	
	//reset circular buffer pointers
	FIFO_W = 0;
	FIFO_R = 0;
	sample_count = 0;

} //initFIFO

void i2sInit (void)
{

/*
In this section we will set up the DCI module for I2S operation to interface 
with a DIR9001 SPDIF decoder receiving data at 44.1kHz STEREO The timing diagram is 
provided in Fig 1 below:
                                  FIGURE 1
      _________________________________

      |		 	    	              |	                              |
COFS: |	  	     	                  |_______________________________|
       _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _    
CSCK:_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|
          |<--------Right Channel Data--->|<----Left Channel Data-------->|
          |<---16bits---->|               |<---16bits---->|               |
          |<---RXBUF0---->|               |<---RXBUF0---->|               |
          |<--TimeSlot0-->|               |<--TimeSlot0-->|               |
          |<-----------1 Frame----------->|<-----------1 Frame----------->|
          |<--------------------1/Fs = 22.67 microseconds---------------->|
*/

//DCI Control Register DCICON1 Initialization
	DCICON1bits.CSCKD 	= 1;	// Serial Bit Clock (CSCK pin) is input	DIR9001 controls data transfer
	DCICON1bits.CSCKE 	= 1;	// Data changes on falling edge sampled on rising edge of CSCK, from DIR9001 data sheet
	DCICON1bits.COFSD	= 1;	// Frame Sync Signal is input DIR9001 controls data transfer
	DCICON1bits.UNFM 	= 0;	// Transmit ‘0’s on a transmit underflow
	DCICON1bits.CSDOM 	= 1;	// CSDO pin tristated during disabled transmit time slots, CSDO needs an EXTERNAL pull-down resistor
	DCICON1bits.DJST 	= 0;	// Data TX/RX is begun one serial clock cycle after frame sync pulse
	DCICON1bits.COFSM 	= 1;	// Frame Sync Signal set up for I2S mode 
	DCICON1bits.DLOOP 	= 0;	// No Loopback

// DCI Control Register DCICON2 Initialization
	DCICON2bits.BLEN	= 0;	// One data word will be buffered between interrupts	
	DCICON2bits.COFSG 	= 0;	// Data frame has 1 word
	DCICON2bits.WS 		= 15;	// Data word size is 16 bits


// DCI Control Register DCICON3 Initialization
	DCICON3 = 0;	    // Set up CSCK Bit Clock Frequency (disabled)

// Transmit Slot Control Register Initialization
	TSCONbits.TSE0 = 0;	    // Transmit on Time Slot 0	DISABLED
	
//Receiver Slot Control Register Initialization
	RSCONbits.RSE0 = 1;	    // Receive on Time Slot 0	

// Disable DCI Interrupt and Enable DCI module
	IFS3bits.DCIIF=0;  //DCIIF: DCI Event Interrupt Flag Status bit
    IEC3bits.DCIIE=1;  //DCIIE: DCI Event Interrupt Enable bit

	DCICON1bits.DCIEN = 1; 

} //i2sInit

//---------------------------------------END OF INITIALIZATION ROUTINES---------------------------------------
//------------------------------------------------------------------------------------------------------------
//-------------------------------------------------- MAIN ----------------------------------------------------
int main()
{

    // initializations
    initFIFO();		// initialize circular buffer pointers and clears FIFO
	i2sInit ();		// initialize the I2S interface to recieve samples
    initVideo();    // start the video state machine
	

	TRISA = 0xff80;	// set for debugging purposes   1111111110000000
					// RA0-RA6 Output
					// RA7-RA15 Input -- RA7 is a pushbutton on PIC16 Dev Board
					// 0 - Output
					// 1 - Input

	while(FIFO_W < (FIFO_SIZE/2))
	{
	//Wait for FIFO buffer to become half full
	}

	//Enable interrupts to start loading samples via SPI and generate sync pulses
	_OC1IE = 1;	//Enable sync
	_OC2IE = 1;	//Enable sync
	_OC3IE = 1;	//Enable sync
	_OC4IE = 1;	//Enable SPI 
	_OC5IE = 1;	//Enable OC5 - External switching of Fs x 64 into video signal at blank (non-data) lines
	_T3IE =  1;	//Startup video state machine
 

      // main loop -----------------------------------------    
	while((sample_count > 0) && (sample_count < FIFO_SIZE))
	//while(1)
	{   
   		//continue for as long as there are samples to load

		//sample_count may NEVER be negative or zero, this would indicate a buffer
		//underrun condition, i.e. taking more samples out than you're putting in

		//sample_count may NEVER be more than the actual number of storage locations in the FIFO
        //this would indicate a buffer overrun condition, i.e. putting more samples in than you're taking out

	} // main loop -----------------------------------------


   	//trap to disable system should the above loop exit
	_OC1IE = 0;	//Disable sync
	_OC2IE = 0;	//Disable sync
	_OC3IE = 0;	//Disable sync
	_OC4IE = 0;	//Disable SPI 
	_OC5IE = 0;	//Disable OC5
	_T3IE =  0;	//Shut down video state machine


	while(1)
	{
			__asm__ volatile ("reset"); //an error has occurred an system will be reset
	}

	return 0;
	
} // main
//------------------------------------------------------------------------------------------------------------
