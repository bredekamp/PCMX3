#include <xc.h>
#include "PCM_X2.h"

#define _FAR __attribute__(( far))
#define Nop() {__asm__ volatile ("nop");}

volatile int frame[8]; 		//holds samples for ONE data frame
volatile int slot_counter;	//keep tracks of unload slot currently being read	
volatile int count;			//used by IC1 ISR for delay
volatile int i; 			//general purpose counter

#define _FAR __attribute__(( far))

//--FIFO HOUSEKEEPING-------------------------------
//circular buffers
unsigned int FIFO[FIFO_SIZE] __attribute__((space(dma)));	

//FIFO write and read pointers
volatile int FIFO_W,FIFO_R;	

volatile int sample_count;

//-------------------------------------------INTERRUPT ROUTINES-----------------------------------------------
void _ISRFAST _IC1Interrupt( void)//used to detect sync pulse,capture every rising edge
{	//This ISR produces an array called frame which holds one data frame for processing
	//_RA5 = 1;

	count = 29;	//used for delay to line up SPI clock with first sample ; 
				//little spikes on wave are sample errors, can be fixed by tweaking this delay
				//if this value is way out, then no valid frames will be detected and the sound samples will all be 0x0000
	slot_counter = 0;
	i = 0;

	//delay
	while (count > 0)
		count--;

	Nop();
	Nop();
	Nop();
	Nop();
	
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}

		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample - 0
		slot_counter++;  //1
	
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}
		
		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -1
		slot_counter++; //2
	
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}
	
		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -2
		slot_counter++; //3
		
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}

		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -3
		slot_counter++; //4
		
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}

		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -4
		slot_counter++; //5
		
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}

		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -5
		slot_counter++; //6
		
	//-----------------------------------------------------------------------
		while( SPI1STATbits.SPITBF);
				{   //wait for TRANSMIT buffer to become available	
				}
		
		SPI1BUF = 0x0001; //dummy write to activate SPI unit
		
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}

		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -6
		slot_counter++; //7
	
	//-----------------------------------------------------------------------
		while( !SPI1STATbits.SPIRBF);
				{//wait for RECIEVE to complete
				}

		frame[slot_counter] = SPI1BUF; //read the RECIEVED sample -7
		
	//-----------------------------------------------------------------------
	
	if (frame[i] == 0xAAAA) //is this a valid frame?
	{						//while loop unfurled for speed
		
		FIFO[FIFO_W] = frame[i+1];		//1
		FIFO_W++;						//increment the write pointer
		FIFO_W %= FIFO_SIZE;			//wrap around

	
		FIFO[FIFO_W] = frame[i+2];		//2	
		FIFO_W++;						//increment the write pointer
		FIFO_W %= FIFO_SIZE;			//wrap around

	
		FIFO[FIFO_W] = frame[i+3];		//3	
		FIFO_W++;						//increment the write pointer
		FIFO_W %= FIFO_SIZE;			//wrap around


		FIFO[FIFO_W] = frame[i+4];		//4		
		FIFO_W++;						//increment the write pointer
		FIFO_W %= FIFO_SIZE;			//wrap around
	

		FIFO[FIFO_W] = frame[i+5];		//5	
		FIFO_W++;						//increment the write pointer
		FIFO_W %= FIFO_SIZE;			//wrap around

	
		FIFO[FIFO_W] = frame[i+6];		//6	
		FIFO_W++;						//increment the write pointer
		FIFO_W %= FIFO_SIZE;			//wrap around

		sample_count += 6;
	}

	//-----------------------------------------------------------------------
	
    _IC1IF = 0; // clear the interrupt flag
//	_RA5 = 0;	
}// IC1Interrupt

/*=============================================================================
_DMA0Init(): Initialise DMA0 for DCI Data Transmission 
=============================================================================*/
// DMA0 configuration
// Direction	: Read from DMA RAM and write to peripheral address 0x298 (TXBUF0 register)
// AMODE	: Register Indirect with Post-Increment mode
// MODE		: Continuous, Ping-Pong modes disabled
// IRQ		: DCI
//
void dma0Init(void)
{

	DMA0CON = 0x2000;	//0010000000000000			
	DMA0CNT = FIFO_SIZE-1;	//How many data elements are moved out					
	DMA0REQ = 0x003C;	//IRQ no.60 --> DCI				

	DMA0PAD = (volatile unsigned int) &TXBUF0;
	DMA0STA= __builtin_dmaoffset(FIFO); 	//Start address 0x7800
	
	
	
	IFS0bits.DMA0IF  = 0;			// Clear DMA interrupt
	IEC0bits.DMA0IE  = 0;			// DISABLE DMA interrupt 
	DMA0CONbits.CHEN = 1;			// Enable DMA Channel
}

/*=============================================================================
_DMA0Interrupt(): DCI Transmit Interrupt Handler
=============================================================================*/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
	//Interrupt called every time DCI reaches the end of the buffer
	//NOT USED - Disabled in _DMA0Init()
	
        IFS0bits.DMA0IF = 0;		//Clear the DMA0 Interrupt Flag
}//DMA0Interrupt

//------------------------------------------------------------------------------

//-------------------------------------------END OF INTERRUPT ROUTINES----------------------------------------
//------------------------------------------------------------------------------------------------------------
//---------------------------------------START OF INITIALIZATION ROUTINES-------------------------------------
void initSys( void)
{
	//Set Oscillator Configuration
	OSCCON = 0x0300;		//
	CLKDIV = 0x0000;		//
	PLLFBD = 0x0037;		// 128 x Fs = 5.6448 MHz -- 2.8224 MHz x 57 = 160.8768MHz -- set divider to 57
 							// 40.2192 MIPS / 16 = 2.5137 MHz ==> SPI clock ==> run-in = 1.25685 MHz = 44100Hz x 28.5
	
    // set the priority levels
	_IC1IP = 4;					//4
	_DCIIP = 5;					//5

	//setup IC Modules                                    ___
	IC1CON = 0x0003;			//capture every rising __|    edge

	// init Interrupts, clear the flag
    _IC1IF = 0; _IC1IE = 1;	
    	
	// init the processor priority level
	_IPL = 0;					// this is the default value anyway
    
    // init PORTS
	_TRISD8 = 1;				//make IC1 = RD8 pin an input
	_TRISA5 = 0;				//output for debug purposes

	slot_counter = 0;

	for (i=0;i<8;i=i+1)			//Initialize frame buffer array
		frame[i] = 0;
	
	i = 0;

	//reset circular buffer pointers
	FIFO_W = 0;
	FIFO_R = 0;
	sample_count = 0;

} // initSys

void initSPI( void)
{
	SPI1BUF = 0x0000;		//clear SPI buffer

	_SPI1IF = 0;			//clear SPI interrupt flag
	_SPI1IE = 0;    		//disable interrupt - not used
	_SPI1IP = 4;    		//set SPI priority - not used

	SPI1CON1 = 0b1010100111101;		//SDO disabled, MASTER mode, internal clock
									//
	SPI1CON2 = 0x0000;      		// 

	_SPIROV = 0;			//clear Receive Overflow Flag bit

    //SPI1STAT = 0x8000;      // enable SPI port

}//initSPI

void i2sInit (void)
{

/*
In this section we will set up the DCI module for I2S operation to interface 
with a DIT4096 SPDIF encoder transmitting data at 44.1kHz STEREO The timing diagram is 
provided in Fig 1 below:
                                  FIGURE 1
      _________________________________

      |		 	    	              |	                              |
COFS: |	  	     	                  |_______________________________|
       _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _   _    
CSCK:_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|
          |<--------Right Channel Data--->|<----Left Channel Data-------->|
          |<---16bits---->|               |<---16bits---->|               |
          |<---TXBUF0---->|               |<---TXBUF0---->|               |
          |<--TimeSlot0-->|               |<--TimeSlot0-->|               |
          |<-----------1 Frame----------->|<-----------1 Frame----------->|
          |<--------------------1/Fs = 22.67 microseconds---------------->|
*/

//DCI Control Register DCICON1 Initialization
	DCICON1bits.CSCKD 	= 0;	// Serial Bit Clock (CSCK pin) is output
	DCICON1bits.CSCKE 	= 1;	// Data changes on falling edge sampled on rising edge of CSCK
	DCICON1bits.COFSD	= 0;	// Frame Sync Signal is output
	DCICON1bits.UNFM 	= 0;	// Transmit ‘0’s on a transmit underflow
	DCICON1bits.CSDOM 	= 1;	// CSDO pin tristated during disabled transmit time slots, CSDO needs an EXTERNAL pull-down resistor
	DCICON1bits.DJST 	= 1;	// Data transmission/reception begins during the same serial clock cycle as the frame
								// synchronization pulse -- using TDA1545
	DCICON1bits.COFSM 	= 1;	// Frame Sync Signal set up for I2S mode 
	DCICON1bits.DLOOP 	= 0;	// No Loopback

// DCI Control Register DCICON2 Initialization
	DCICON2bits.BLEN	= 0;	// one data word will be buffered between interrupts	
	DCICON2bits.COFSG 	= 0;	// Data frame has 1 word
	DCICON2bits.WS 		= 15;	// Data word size is 16 bits


// DCI Control Register DCICON3 Initialization
	DCICON3 = 13;	    		// Set up CSCK Bit Clock Frequency 

// Transmit Slot Control Register Initialization
	TSCONbits.TSE0 = 1;	    	// Transmit on Time Slot 0(enabled)
	
//Receiver Slot Control Register Initialization
	RSCONbits.RSE0 = 0;	    	// Receive on Time Slot 0 (disabled)

// Disable DCI Interrupt and Enable DCI module
	IFS3bits.DCIIF=0;  			//DCIIF: DCI Event Interrupt Flag Status bit
    IEC3bits.DCIIE=0;  			//DCIIE: DCI Event Interrupt Enable bit

	
} //i2sInit


//---------------------------------------END OF INITIALIZATION ROUTINES---------------------------------------
//------------------------------------------------------------------------------------------------------------
//-------------------------------------------------- MAIN ----------------------------------------------------
int main()
{
	// initializations
    initSys();    				// initialize basic system
	initSPI();					// initialize SPI
	i2sInit();
	dma0Init();

	DCICON1bits.DCIEN = 1; 		//Activate the DCI
    SPI1STAT = 0x8000;      	//Enable SPI port

	while(FIFO_W < (FIFO_SIZE/2))
	{
	//Wait for FIFO buffer to become half full
	}

	while(1)
	{
		//main loop
	}

	return 0;
	
} // main
//------------------------------------------------------------------------------------------------------------
