/*
**  PAL Video 
**  Graphic library
**
**
*/

#define VRES     294    //active lines per field used for data recording
						//44100 samples/s
						//88200 samples/s stereo
						//1764 samples per field
						//1764/294=6 samples per line

#define HRES     128    // desired horizontal resolution (pixel)

#define FIFO_SIZE 2048	// storage locations in FIFO 

void initVideo( void);

void haltVideo( void);

void synchV( void);

void initFIFO( void);

void i2sInit (void);

void __attribute__((__interrupt__)) _DCIInterrupt(void);



