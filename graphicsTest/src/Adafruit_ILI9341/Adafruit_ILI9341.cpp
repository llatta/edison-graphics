/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_ILI9341.h"
#include <limits.h>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>

#define DELAY 0x80

#define HIGH                1
#define LOW                 0

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Adafruit_ILI9341::Adafruit_ILI9341(int8_t cs, int8_t dc, int8_t rst) : Adafruit_GFX(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
	m_cs   = cs;
	m_dc   = dc;
	m_rst  = rst;
}

void Adafruit_ILI9341::spiwrite(uint8_t c) {

	//Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

	mraa_spi_write (m_spi, c);
}


void Adafruit_ILI9341::writecommand(uint8_t c) {
	dcLow();
	csLow();

	spiwrite(c);

	csHigh();
}


void Adafruit_ILI9341::writedata(uint8_t c) {
	dcHigh();
	csLow();

	spiwrite(c);

	csHigh();
}

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
/*void Adafruit_ILI9341::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}*/


void Adafruit_ILI9341::begin(void) {
    m_spi = mraa_spi_init (0);
    mraa_result_t error = mraa_spi_frequency(m_spi, 8 * 1000000);
    if (error != MRAA_SUCCESS) {
        mraa_result_print (error);
    }

	m_csPinCtx = mraa_gpio_init (m_cs);
	if (m_csPinCtx == nullptr) {
		fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", m_cs);
		exit (1);
	}

	error = mraa_gpio_dir (m_csPinCtx, MRAA_GPIO_OUT);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}


	m_dcPinCtx = mraa_gpio_init (m_dc);
	if (m_dcPinCtx == nullptr) {
		fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", m_dc);
		exit (1);
	}

	error = mraa_gpio_dir (m_dcPinCtx, MRAA_GPIO_OUT);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}

	// toggle RST low to reset
	if (m_rst > 0) {
		m_rstPinCtx = mraa_gpio_init (m_rst);
		if (m_rstPinCtx == nullptr) {
			fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", m_rst);
			exit (1);
		}

		error = mraa_gpio_dir (m_rstPinCtx, MRAA_GPIO_OUT);
		if (error != MRAA_SUCCESS) {
			mraa_result_print (error);
		}

		error = mraa_gpio_write (m_rstPinCtx, LOW);
		if (error != MRAA_SUCCESS) {
			mraa_result_print (error);
		}
		mraa_gpio_write (m_rstPinCtx, HIGH);
		usleep(5000);
		mraa_gpio_write (m_rstPinCtx, LOW);
		usleep(20000);
		mraa_gpio_write (m_rstPinCtx, HIGH);
		usleep(150000);
	}

	/*
  uint8_t x = readcommand8(ILI9341_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
	 */
	//if(cmdList) commandList(cmdList);

	writecommand(0xEF);
	writedata(0x03);
	writedata(0x80);
	writedata(0x02);

	writecommand(0xCF);
	writedata(0x00);
	writedata(0XC1);
	writedata(0X30);

	writecommand(0xED);
	writedata(0x64);
	writedata(0x03);
	writedata(0X12);
	writedata(0X81);

	writecommand(0xE8);
	writedata(0x85);
	writedata(0x00);
	writedata(0x78);

	writecommand(0xCB);
	writedata(0x39);
	writedata(0x2C);
	writedata(0x00);
	writedata(0x34);
	writedata(0x02);

	writecommand(0xF7);
	writedata(0x20);

	writecommand(0xEA);
	writedata(0x00);
	writedata(0x00);

	writecommand(ILI9341_PWCTR1);    //Power control
	writedata(0x23);   //VRH[5:0]

	writecommand(ILI9341_PWCTR2);    //Power control
	writedata(0x10);   //SAP[2:0];BT[3:0]

	writecommand(ILI9341_VMCTR1);    //VCM control
	writedata(0x3e); //�Աȶȵ���
	writedata(0x28);

	writecommand(ILI9341_VMCTR2);    //VCM control2
	writedata(0x86);  //--

	writecommand(ILI9341_MADCTL);    // Memory Access Control
	writedata(0x48);

	writecommand(ILI9341_PIXFMT);
	writedata(0x55);

	writecommand(ILI9341_FRMCTR1);
	writedata(0x00);
	writedata(0x18);

	writecommand(ILI9341_DFUNCTR);    // Display Function Control
	writedata(0x08);
	writedata(0x82);
	writedata(0x27);

	writecommand(0xF2);    // 3Gamma Function Disable
	writedata(0x00);

	writecommand(ILI9341_GAMMASET);    //Gamma curve selected
	writedata(0x01);

	writecommand(ILI9341_GMCTRP1);    //Set Gamma
	writedata(0x0F);
	writedata(0x31);
	writedata(0x2B);
	writedata(0x0C);
	writedata(0x0E);
	writedata(0x08);
	writedata(0x4E);
	writedata(0xF1);
	writedata(0x37);
	writedata(0x07);
	writedata(0x10);
	writedata(0x03);
	writedata(0x0E);
	writedata(0x09);
	writedata(0x00);

	writecommand(ILI9341_GMCTRN1);    //Set Gamma
	writedata(0x00);
	writedata(0x0E);
	writedata(0x14);
	writedata(0x03);
	writedata(0x11);
	writedata(0x07);
	writedata(0x31);
	writedata(0xC1);
	writedata(0x48);
	writedata(0x08);
	writedata(0x0F);
	writedata(0x0C);
	writedata(0x31);
	writedata(0x36);
	writedata(0x0F);

	writecommand(ILI9341_SLPOUT);    //Exit Sleep
	usleep(120000);
	writecommand(ILI9341_DISPON);    //Display on
}


void Adafruit_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
		uint16_t y1) {

	writecommand(ILI9341_CASET); // Column addr set
#if 0
	writedata(x0 >> 8);
	writedata(x0 & 0xFF);     // XSTART
	writedata(x1 >> 8);
	writedata(x1 & 0xFF);     // XEND
#else
	dcHigh();
	csLow();
	{
		uint8_t d[] = { uint8_t(x0 >> 8), uint8_t(x0), uint8_t(x1 >> 8), uint8_t(x1) };
		mraa_result_t error = mraa_spi_transfer_buf(m_spi, (uint8_t*)d, (uint8_t*)0, sizeof(d));
		if (error != MRAA_SUCCESS) {
			mraa_result_print (error);
		}
	}
	csHigh();

#endif

	writecommand(ILI9341_PASET); // Row addr set
#if 0
	writedata(y0>>8);
	writedata(y0);     // YSTART
	writedata(y1>>8);
	writedata(y1);     // YEND
#else
	dcHigh();
	csLow();
	{
		uint8_t d[] = { uint8_t(y0 >> 8), uint8_t(y0), uint8_t(y1 >> 8), uint8_t(y1) };
		mraa_result_t error = mraa_spi_transfer_buf(m_spi, (uint8_t*)d, (uint8_t*)0, sizeof(d));
		if (error != MRAA_SUCCESS) {
			mraa_result_print (error);
		}
	}
	csHigh();
#endif

	writecommand(ILI9341_RAMWR); // write to RAM
}


void Adafruit_ILI9341::pushColor(uint16_t color) {
	dcHigh();
	csLow();

	spiwrite(color >> 8);
	spiwrite(color);

	csHigh();
}

void Adafruit_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) {

	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

	setAddrWindow(x,y,x+1,y+1);

	dcHigh();
	csLow();

	spiwrite(color >> 8);
	spiwrite(color);

	csHigh();
}


void Adafruit_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h,
		uint16_t color) {

	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;

	if((y+h-1) >= _height)
		h = _height-y;

	setAddrWindow(x, y, x, y+h-1);

	uint8_t hi = color >> 8, lo = color;

	dcHigh();
	csLow();

	while (h--) {
		spiwrite(hi);
		spiwrite(lo);
	}

	csHigh();
}


void Adafruit_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w,
		uint16_t color) {

	// Rudimentary clipping
	if((x >= _width) || (y >= _height)) return;
	if((x+w-1) >= _width)  w = _width-x;
	setAddrWindow(x, y, x+w-1, y);

	uint8_t hi = color >> 8, lo = color;
	dcHigh();
	csLow();
	while (w--) {
		spiwrite(hi);
		spiwrite(lo);
	}
	csHigh();
}

void Adafruit_ILI9341::fillScreen(uint16_t color) {
	fillRect(0, 0,  _width, _height, color);
}

// fill a rectangle
void Adafruit_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
		uint16_t color) {
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

#if 1
	setAddrWindow(x, y, x+w-1, y+h-1);

	dcHigh();
	csLow();

	uint8_t* p = (uint8_t*)m_frameBuffer;

	for(y=h; y>0; y--) {
		for(x=w; x>0; x--) {
			*p++ = color >> 8;
			*p++ = color & 0xFF;
		}
	}

	const int maxTransferSize = 4 * 1024;
	uint8_t* transfer = (uint8_t*)m_frameBuffer;
	uint8_t* endTransfer = transfer + w * h * sizeof(uint16_t);
	for (; transfer < endTransfer; transfer += maxTransferSize)
	{
		const int size = std::min(maxTransferSize, (int)(endTransfer - transfer));
		mraa_result_t error = mraa_spi_transfer_buf(m_spi, (uint8_t*)transfer, (uint8_t*)0, size);
		if (error != MRAA_SUCCESS) {
			mraa_result_print (error);
		}
	}

	csHigh();


#else
	setAddrWindow(x, y, x+w-1, y+h-1);

	uint8_t hi = color >> 8, lo = color;

	dcHigh();
	csLow();

	for(y=h; y>0; y--) {
		for(x=w; x>0; x--) {
			spiwrite(hi);
			spiwrite(lo);
		}
	}

	csHigh();
#endif
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_ILI9341::color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void Adafruit_ILI9341::setRotation(uint8_t m) {

	writecommand(ILI9341_MADCTL);
	rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		writedata(MADCTL_MX | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 1:
		writedata(MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	case 2:
		writedata(MADCTL_MY | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 3:
		writedata(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	}
}


void Adafruit_ILI9341::invertDisplay(bool i) {
	writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
}


void Adafruit_ILI9341::dcHigh()
{
	mraa_result_t error = mraa_gpio_write(m_dcPinCtx, HIGH);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}
}

void Adafruit_ILI9341::dcLow()
{
	mraa_result_t error = mraa_gpio_write(m_dcPinCtx, LOW);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}
}

void Adafruit_ILI9341::csHigh()
{
	mraa_result_t error = mraa_gpio_write(m_csPinCtx, HIGH);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}
}

void Adafruit_ILI9341::csLow()
{
	mraa_result_t error = mraa_gpio_write(m_csPinCtx, LOW);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}
}

////////// stuff not actively being used, but kept for posterity

uint8_t Adafruit_ILI9341::spiread(void) {
	uint8_t r = 0;
	uint8_t s = 0;

	// Send a dummy byte, to receive a byte
	mraa_result_t error = mraa_spi_transfer_buf(m_spi, &s, &r, 1);
	if (error != MRAA_SUCCESS) {
		mraa_result_print (error);
	}

	//Serial.print("read: 0x"); Serial.print(r, HEX);

	return r;
}

/*uint8_t Adafruit_ILI9341::readdata(void) {
	digitalWrite(_dc, HIGH);
	digitalWrite(_cs, LOW);
	uint8_t r = spiread();
	digitalWrite(_cs, HIGH);

	return r;
}

*/
uint8_t Adafruit_ILI9341::readcommand8(uint8_t c, uint8_t index) {
	dcLow(); // command
	csLow();
	spiwrite(0xD9);  // woo sekret command?
	dcHigh(); // data
	spiwrite(0x10 + index);
	csHigh();

	dcLow();
	csLow();
	spiwrite(c);

	dcHigh();
	uint8_t r = spiread();
	csHigh();

	return r;
}
/*
 uint16_t Adafruit_ILI9341::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);

 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 uint32_t Adafruit_ILI9341::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!

 dummyclock();
 dummyclock();

 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);

 pinMode(_sid, OUTPUT); // back to output
 return r;
 }

 */
