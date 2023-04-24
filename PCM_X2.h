void initSys( void);
void initSPI( void);
void i2sInit (void);
void __attribute__((__interrupt__)) _DCIInterrupt(void);

#define CLOCK_PERIOD 8
#define FIFO_SIZE 1024 	//MAXIMUM for DMA RAM space
					   	//1024 x 16 = 2048 x 8










