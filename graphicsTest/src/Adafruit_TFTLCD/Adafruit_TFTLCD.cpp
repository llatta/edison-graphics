// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#include "Adafruit_TFTLCD.h"
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>

#if SIMPLE_PINS
	static const int TFT_CS = 8;
	static const int TFT_CD = 9;
	static const int TFT_WR = 10;
	static const int TFT_RD = 11;
	static const int TFT_DATA[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
#else
	static const int TFT_CS = 9;
	static const int TFT_CD = 8;
	static const int TFT_WR = 7;
	static const int TFT_RD = 6;
	static const int TFT_DATA[8] = { 13, 10, 12, 11, 14, 15, 16, 17 };

	// First hardware pin number of the data bits, ie 40-47, and WR flag assumed in the same % 32 range
	static const int TFT_EDISON_DATA0 = 40;
	static const int TFT_EDISON_WR = 48;
#endif

#define TFTWIDTH   240
#define TFTHEIGHT  320

#define HIGH                1
#define LOW                 0

// When using the TFT breakout board, control pins are configurable.
#define RD_ACTIVE  mraa_gpio_write(m_rdPinCtx, LOW)
#define RD_IDLE    mraa_gpio_write(m_rdPinCtx, HIGH)
#define WR_ACTIVE  mraa_gpio_write(m_wrPinCtx, LOW)
#define WR_IDLE    mraa_gpio_write(m_wrPinCtx, HIGH)
#define CD_COMMAND mraa_gpio_write(m_cdPinCtx, LOW)
#define CD_DATA    mraa_gpio_write(m_cdPinCtx, HIGH)
#define CS_ACTIVE  mraa_gpio_write(m_csPinCtx, LOW)
#define CS_IDLE    mraa_gpio_write(m_csPinCtx, HIGH)

// Data write strobe, ~2 instructions and always inline
//#define WR_STROBE { WR_ACTIVE; WR_IDLE; }
#define WR_STROBE { uint32_t bit = (uint32_t)((uint32_t)1 << (TFT_EDISON_WR % 32)); \
	*(volatile uint32_t*)mmap_reg_clear = bit; \
	*(volatile uint32_t*)mmap_reg_set = bit; }

static void delay(int milliseconds)
{
	usleep(milliseconds * 1000);
}

static void delayMicroseconds(int microseconds)
{
	usleep(microseconds);
}

// LCD controller chip identifiers
#define ID_932X    0
#define ID_7575    1
#define ID_9341    2
#define ID_HX8357D    3
#define ID_UNKNOWN 0xFF

#include "registers.h"

/**
 * A structure representing a gpio pin.
 */
struct _gpio {
    /*@{*/
    int pin; /**< the pin number, as known to the os. */
    int phy_pin; /**< pin passed to clean init. -1 none and raw*/
    int value_fp; /**< the file pointer to the value of the gpio */
    void (* isr)(void *); /**< the interupt service request */
    void *isr_args; /**< args return when interupt service request triggered */
    pthread_t thread_id; /**< the isr handler thread id */
    int isr_value_fp; /**< the isr file pointer on the value */
    mraa_boolean_t owner; /**< If this context originally exported the pin */
    mraa_result_t (*mmap_write) (mraa_gpio_context dev, int value);
    int (*mmap_read) (mraa_gpio_context dev);
    /*@}*/
};

// This is an absolute path to a resource file found within sysfs.
// Might not always be correct. First thing to check if mmap stops
// working. Check the device for 0x1199 and Intel Vendor (0x8086)
#define MMAP_PATH "/sys/devices/pci0000:00/0000:00:0c.0/resource0"

//MMAP
static uint8_t *mmap_reg = NULL;
static uint8_t *mmap_reg_set = NULL;
static uint8_t *mmap_reg_clear = NULL;
static int mmap_fd = 0;
static int mmap_size;

mraa_result_t mraa_intel_edison_mmap_writeX(mraa_gpio_context dev, int value)
{
    uint8_t offset = ((dev->pin / 32) * sizeof(uint32_t));
    uint8_t valoff;

    if (value) {
        valoff = 0x34;
    } else {
        valoff = 0x4c;
    }

    *(volatile uint32_t*) (mmap_reg + offset + valoff) =
        (uint32_t)(1 << (dev->pin % 32));

    return MRAA_SUCCESS;
}

// Requirements: dev points to the first of 8 consecutive pins. All pins need to be in the same % 32 range.
mraa_result_t mraa_intel_edison_mmap_write8(mraa_gpio_context dev, uint8_t value)
{
    uint8_t offset = ((dev->pin / 32) * sizeof(uint32_t));

    uint32_t set   = (uint32_t)((uint32_t)value << (dev->pin % 32));
    uint32_t clear = (uint32_t)((uint32_t)((uint8_t)~value) << (dev->pin % 32));
    *(volatile uint32_t*) (mmap_reg + offset + 0x34) = set;
    *(volatile uint32_t*) (mmap_reg + offset + 0x4c) = clear;

    return MRAA_SUCCESS;
}

// Constructor for breakout board (configurable LCD control lines).
Adafruit_TFTLCD::Adafruit_TFTLCD() :
		Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {

	m_csPinCtx = mraa_gpio_init(TFT_CS);
	if (m_csPinCtx == nullptr) {
		fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", TFT_CS);
		exit (1);
	}
	m_cdPinCtx = mraa_gpio_init(TFT_CD);
	if (m_cdPinCtx == nullptr) {
		fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", TFT_CD);
		exit (1);
	}
	m_wrPinCtx = mraa_gpio_init(TFT_WR);
	if (m_wrPinCtx == nullptr) {
		fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", TFT_WR);
		exit (1);
	}
	m_rdPinCtx = mraa_gpio_init(TFT_RD);
	if (m_rdPinCtx == nullptr) {
		fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", TFT_RD);
		exit (1);
	}
	for (int i = 0; i < 8; ++i)
	{
		m_dataPinCtx[i] = mraa_gpio_init(TFT_DATA[i]);
		if (m_dataPinCtx[i] == nullptr) {
			fprintf (stderr, "Are you sure that pin%d you requested is valid on your platform?", TFT_DATA[i]);
			exit (1);
		}

		mraa_gpio_use_mmaped(m_dataPinCtx[i], 1);
	}

	mraa_gpio_dir(m_csPinCtx, MRAA_GPIO_OUT);
	mraa_gpio_dir(m_cdPinCtx, MRAA_GPIO_OUT);
	mraa_gpio_dir(m_wrPinCtx, MRAA_GPIO_OUT);
	mraa_gpio_dir(m_rdPinCtx, MRAA_GPIO_OUT);

	mraa_gpio_use_mmaped(m_csPinCtx, 1);
	mraa_gpio_use_mmaped(m_cdPinCtx, 1);
	mraa_gpio_use_mmaped(m_wrPinCtx, 1);
	mraa_gpio_use_mmaped(m_rdPinCtx, 1);

    if (mmap_reg == NULL) {
        if ((mmap_fd = open(MMAP_PATH, O_RDWR)) < 0) {
            printf("edison map: unable to open resource0 file\n");
        }

        struct stat fd_stat;
        fstat(mmap_fd, &fd_stat);
        mmap_size = fd_stat.st_size;

        mmap_reg = (uint8_t*) mmap(NULL, fd_stat.st_size,
                                   PROT_READ | PROT_WRITE,
                                   MAP_FILE | MAP_SHARED,
                                   mmap_fd, 0);
        if (mmap_reg == MAP_FAILED) {
            printf("edison mmap: failed to mmap\n");
            mmap_reg = NULL;
            close(mmap_fd);
        }

        uint8_t offset = ((TFT_EDISON_DATA0 / 32) * sizeof(uint32_t));
        mmap_reg_set = mmap_reg + offset + 0x34;
        mmap_reg_clear = mmap_reg + offset + 0x4c;
    }



#if 1//def USE_ADAFRUIT_SHIELD_PINOUT
	CS_IDLE; // Set all control bits to idle state
	WR_IDLE;
	RD_IDLE;
	CD_DATA;
#endif

	setWriteDir(); // Set up LCD data port(s) for WRITE operations

	rotation  = 0;
	cursor_y  = cursor_x = 0;
	textsize  = 1;
	textcolor = 0xFFFF;
	_width    = TFTWIDTH;
	_height   = TFTHEIGHT;


	m_floodPinCtx = mraa_gpio_init(19);
	mraa_gpio_dir(m_floodPinCtx, MRAA_GPIO_OUT);
}

// Initialization command tables for different LCD controllers
#define TFTLCD_DELAY 0xFF

void Adafruit_TFTLCD::begin(uint16_t id) {
	reset();

	delay(200);

	if (id == 0x9341) {
		driver = ID_9341;
		CS_ACTIVE;
		writeRegister8(ILI9341_SOFTRESET, 0);
		delay(50);
		writeRegister8(ILI9341_DISPLAYOFF, 0);

		writeRegister8(ILI9341_POWERCONTROL1, 0x23);
		writeRegister8(ILI9341_POWERCONTROL2, 0x10);
		writeRegister16(ILI9341_VCOMCONTROL1, 0x2B2B);
		writeRegister8(ILI9341_VCOMCONTROL2, 0xC0);
		writeRegister8(ILI9341_MEMCONTROL, ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR);
		writeRegister8(ILI9341_PIXELFORMAT, 0x55);
		writeRegister16(ILI9341_FRAMECONTROL, 0x001B);

		writeRegister8(ILI9341_ENTRYMODE, 0x07);
		/* writeRegister32(ILI9341_DISPLAYFUNC, 0x0A822700);*/

		writeRegister8(ILI9341_SLEEPOUT, 0);
		delay(150);
		writeRegister8(ILI9341_DISPLAYON, 0);
		delay(500);
		setAddrWindow(0, 0, TFTWIDTH-1, TFTHEIGHT-1);
		return;
	} else {
		driver = ID_UNKNOWN;
		return;
	}
}


void Adafruit_TFTLCD::reset(void) {

	CS_IDLE;
	//  CD_DATA;
	WR_IDLE;
	RD_IDLE;

#if 0 // TESTCODE
	setWriteDir();
	write8(0xff);
	setWriteDir();
	CS_ACTIVE;
	CD_COMMAND;
	write8(5);
	CD_DATA;
	write8(10);
	CS_IDLE;
#endif


#if 0
	if(_reset) {
		digitalWrite(_reset, LOW);
		delay(2);
		digitalWrite(_reset, HIGH);
	}
#endif

	// Data transfer sync
	CS_ACTIVE;
	CD_COMMAND;
	write8(0x00);
	for(uint8_t i=0; i<3; i++) WR_STROBE; // Three extra 0x00s
	CS_IDLE;
}

// Sets the LCD address window (and address counter, on 932X).
// Relevant to rect/screen fills and H/V lines.  Input coordinates are
// assumed pre-sorted (e.g. x2 >= x1).
void Adafruit_TFTLCD::setAddrWindow(int x1, int y1, int x2, int y2) {
	CS_ACTIVE;
	if(driver == ID_932X) {

		// Values passed are in current (possibly rotated) coordinate
		// system.  932X requires hardware-native coords regardless of
		// MADCTL, so rotate inputs as needed.  The address counter is
		// set to the top-left corner -- although fill operations can be
		// done in any direction, the current screen rotation is applied
		// because some users find it disconcerting when a fill does not
		// occur top-to-bottom.
		int x, y, t;
		switch(rotation) {
		default:
			x  = x1;
			y  = y1;
			break;
		case 1:
			t  = y1;
			y1 = x1;
			x1 = TFTWIDTH  - 1 - y2;
			y2 = x2;
			x2 = TFTWIDTH  - 1 - t;
			x  = x2;
			y  = y1;
			break;
		case 2:
			t  = x1;
			x1 = TFTWIDTH  - 1 - x2;
			x2 = TFTWIDTH  - 1 - t;
			t  = y1;
			y1 = TFTHEIGHT - 1 - y2;
			y2 = TFTHEIGHT - 1 - t;
			x  = x2;
			y  = y2;
			break;
		case 3:
			t  = x1;
			x1 = y1;
			y1 = TFTHEIGHT - 1 - x2;
			x2 = y2;
			y2 = TFTHEIGHT - 1 - t;
			x  = x1;
			y  = y2;
			break;
		}
		writeRegister16(0x0050, x1); // Set address window
		writeRegister16(0x0051, x2);
		writeRegister16(0x0052, y1);
		writeRegister16(0x0053, y2);
		writeRegister16(0x0020, x ); // Set address counter to top left
		writeRegister16(0x0021, y );

	} else if(driver == ID_7575) {

		writeRegisterPair(HX8347G_COLADDRSTART_HI, HX8347G_COLADDRSTART_LO, x1);
		writeRegisterPair(HX8347G_ROWADDRSTART_HI, HX8347G_ROWADDRSTART_LO, y1);
		writeRegisterPair(HX8347G_COLADDREND_HI  , HX8347G_COLADDREND_LO  , x2);
		writeRegisterPair(HX8347G_ROWADDREND_HI  , HX8347G_ROWADDREND_LO  , y2);

	} else if ((driver == ID_9341) || (driver == ID_HX8357D)){
		uint32_t t;

		t = x1;
		t <<= 16;
		t |= x2;
		writeRegister32(ILI9341_COLADDRSET, t);  // HX8357D uses same registers!
		t = y1;
		t <<= 16;
		t |= y2;
		writeRegister32(ILI9341_PAGEADDRSET, t); // HX8357D uses same registers!

	}
	CS_IDLE;
}

// Unlike the 932X drivers that set the address window to the full screen
// by default (using the address counter for drawPixel operations), the
// 7575 needs the address window set on all graphics operations.  In order
// to save a few register writes on each pixel drawn, the lower-right
// corner of the address window is reset after most fill operations, so
// that drawPixel only needs to change the upper left each time.
void Adafruit_TFTLCD::setLR(void) {
	CS_ACTIVE;
	writeRegisterPair(HX8347G_COLADDREND_HI, HX8347G_COLADDREND_LO, _width  - 1);
	writeRegisterPair(HX8347G_ROWADDREND_HI, HX8347G_ROWADDREND_LO, _height - 1);
	CS_IDLE;
}

// Fast block fill operation for fillScreen, fillRect, H/V line, etc.
// Requires setAddrWindow() has previously been called to set the fill
// bounds.  'len' is inclusive, MUST be >= 1.
void Adafruit_TFTLCD::flood(uint16_t color, uint32_t len) {
	int blocks;
	int i;
	uint8_t hi = color >> 8,
			lo = color;

	CS_ACTIVE;
	CD_COMMAND;
	if (driver == ID_9341) {
		write8(0x2C);
	} else if (driver == ID_932X) {
		write8(0x00); // High byte of GRAM register...
		write8(0x22); // Write data to GRAM
	} else if (driver == ID_HX8357D) {
		write8(HX8357_RAMWR);
	} else {
		write8(0x22); // Write data to GRAM
	}

	CD_DATA;

	if(hi == lo) {
		// Write first pixel normally, decrement counter by 1
		write8(hi);
		write8(lo);
		len--;

		blocks = (uint16_t)(len / 64); // 64 pixels/block

		// High and low bytes are identical.  Leave prior data
		// on the port(s) and just toggle the write strobe.
		while(blocks--) {
			i = 16; // 64 pixels/block / 4 pixels/pass
			do {
				WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // 2 bytes/pixel
				WR_STROBE; WR_STROBE; WR_STROBE; WR_STROBE; // x 4 pixels
			} while(--i);
		}
		// Fill any remaining pixels (1 to 64)
		for(i = (uint8_t)len & 63; i--; ) {
			WR_STROBE;
			WR_STROBE;
		}
	}
	else if (color == 0x07E0 && len == (long)TFTWIDTH * (long)TFTHEIGHT)
	{
		mraa_gpio_write(m_floodPinCtx, 1);
		mraa_gpio_write(m_floodPinCtx, 0);
	}
	else
	{
#if 0
		blocks = (uint16_t)(len / 64); // 64 pixels/block

		while(blocks--) {
			i = 16; // 64 pixels/block / 4 pixels/pass
			do {
				write8(hi); write8(lo); write8(hi); write8(lo);
				write8(hi); write8(lo); write8(hi); write8(lo);
			} while(--i);
		}
		for(i = (uint8_t)len & 63; i--; ) {
			write8(hi);
			write8(lo);
		}
#elif 0
		for(int l = len; l; --l) {
			write8(hi);
			write8(lo);
		}
#else
		blocks = (uint16_t)(len / 64); // 64 pixels/block

		while(blocks--) {
			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);

			write8(hi); write8(lo); write8(hi); write8(lo);
			write8(hi); write8(lo); write8(hi); write8(lo);
		}
		for(i = (uint8_t)len & 63; i--; ) {
			write8(hi);
			write8(lo);
		}

#endif
	}
	CS_IDLE;
}

void Adafruit_TFTLCD::drawFastHLine(int16_t x, int16_t y, int16_t length,
		uint16_t color)
{
	int16_t x2;

	// Initial off-screen clipping
	if((length <= 0     ) ||
			(y      <  0     ) || ( y                  >= _height) ||
			(x      >= _width) || ((x2 = (x+length-1)) <  0      )) return;

	if(x < 0) {        // Clip left
		length += x;
		x       = 0;
	}
	if(x2 >= _width) { // Clip right
		x2      = _width - 1;
		length  = x2 - x + 1;
	}

	setAddrWindow(x, y, x2, y);
	flood(color, length);
	if(driver == ID_932X) setAddrWindow(0, 0, _width - 1, _height - 1);
	else                  setLR();
}

void Adafruit_TFTLCD::drawFastVLine(int16_t x, int16_t y, int16_t length,
		uint16_t color)
{
	int16_t y2;

	// Initial off-screen clipping
	if((length <= 0      ) ||
			(x      <  0      ) || ( x                  >= _width) ||
			(y      >= _height) || ((y2 = (y+length-1)) <  0     )) return;
	if(y < 0) {         // Clip top
		length += y;
		y       = 0;
	}
	if(y2 >= _height) { // Clip bottom
		y2      = _height - 1;
		length  = y2 - y + 1;
	}

	setAddrWindow(x, y, x, y2);
	flood(color, length);
	if(driver == ID_932X) setAddrWindow(0, 0, _width - 1, _height - 1);
	else                  setLR();
}

void Adafruit_TFTLCD::fillRect(int16_t x1, int16_t y1, int16_t w, int16_t h, 
		uint16_t fillcolor) {
	int16_t  x2, y2;

	// Initial off-screen clipping
	if( (w            <= 0     ) ||  (h             <= 0      ) ||
			(x1           >= _width) ||  (y1            >= _height) ||
			((x2 = x1+w-1) <  0     ) || ((y2  = y1+h-1) <  0      )) return;
	if(x1 < 0) { // Clip left
		w += x1;
		x1 = 0;
	}
	if(y1 < 0) { // Clip top
		h += y1;
		y1 = 0;
	}
	if(x2 >= _width) { // Clip right
		x2 = _width - 1;
		w  = x2 - x1 + 1;
	}
	if(y2 >= _height) { // Clip bottom
		y2 = _height - 1;
		h  = y2 - y1 + 1;
	}

	setAddrWindow(x1, y1, x2, y2);
	flood(fillcolor, (uint32_t)w * (uint32_t)h);
	if(driver == ID_932X) setAddrWindow(0, 0, _width - 1, _height - 1);
	else                  setLR();
}

void Adafruit_TFTLCD::fillScreen(uint16_t color) {

	if(driver == ID_932X) {

		// For the 932X, a full-screen address window is already the default
		// state, just need to set the address pointer to the top-left corner.
		// Although we could fill in any direction, the code uses the current
		// screen rotation because some users find it disconcerting when a
		// fill does not occur top-to-bottom.
		uint16_t x, y;
		switch(rotation) {
		default: x = 0            ; y = 0            ; break;
		case 1 : x = TFTWIDTH  - 1; y = 0            ; break;
		case 2 : x = TFTWIDTH  - 1; y = TFTHEIGHT - 1; break;
		case 3 : x = 0            ; y = TFTHEIGHT - 1; break;
		}
		CS_ACTIVE;
		writeRegister16(0x0020, x);
		writeRegister16(0x0021, y);

	} else if ((driver == ID_9341) || (driver == ID_7575) || (driver == ID_HX8357D)) {
		// For these, there is no settable address pointer, instead the
		// address window must be set for each drawing operation.  However,
		// this display takes rotation into account for the parameters, no
		// need to do extra rotation math here.
		setAddrWindow(0, 0, _width - 1, _height - 1);

	}
	flood(color, (long)TFTWIDTH * (long)TFTHEIGHT);
}

void Adafruit_TFTLCD::drawPixel(int16_t x, int16_t y, uint16_t color) {

	// Clip
	if((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) return;

	CS_ACTIVE;
	if(driver == ID_932X) {
		int16_t t;
		switch(rotation) {
		case 1:
			t = x;
			x = TFTWIDTH  - 1 - y;
			y = t;
			break;
		case 2:
			x = TFTWIDTH  - 1 - x;
			y = TFTHEIGHT - 1 - y;
			break;
		case 3:
			t = x;
			x = y;
			y = TFTHEIGHT - 1 - t;
			break;
		}
		writeRegister16(0x0020, x);
		writeRegister16(0x0021, y);
		writeRegister16(0x0022, color);

	} else if(driver == ID_7575) {

		uint8_t hi, lo;
		switch(rotation) {
		default: lo = 0   ; break;
		case 1 : lo = 0x60; break;
		case 2 : lo = 0xc0; break;
		case 3 : lo = 0xa0; break;
		}
		writeRegister8(   HX8347G_MEMACCESS      , lo);
		// Only upper-left is set -- bottom-right is full screen default
		writeRegisterPair(HX8347G_COLADDRSTART_HI, HX8347G_COLADDRSTART_LO, x);
		writeRegisterPair(HX8347G_ROWADDRSTART_HI, HX8347G_ROWADDRSTART_LO, y);
		hi = color >> 8; lo = color;
		CD_COMMAND; write8(0x22); CD_DATA; write8(hi); write8(lo);

	} else if ((driver == ID_9341) || (driver == ID_HX8357D)) {
		setAddrWindow(x, y, _width-1, _height-1);
		CS_ACTIVE;
		CD_COMMAND;
		write8(0x2C);
		CD_DATA;
		write8(color >> 8); write8(color);
	}

	CS_IDLE;
}

// Issues 'raw' an array of 16-bit color values to the LCD; used
// externally by BMP examples.  Assumes that setWindowAddr() has
// previously been set to define the bounds.  Max 255 pixels at
// a time (BMP examples read in small chunks due to limited RAM).
void Adafruit_TFTLCD::pushColors(uint16_t *data, uint8_t len, bool first) {
	uint16_t color;
	uint8_t  hi, lo;
	CS_ACTIVE;
	if(first == true) { // Issue GRAM write command only on first call
		CD_COMMAND;
		if(driver == ID_932X) write8(0x00);
		if ((driver == ID_9341) || (driver == ID_HX8357D)){
			write8(0x2C);
		}  else {
			write8(0x22);
		}
	}
	CD_DATA;
	while(len--) {
		color = *data++;
		hi    = color >> 8; // Don't simplify or merge these
		lo    = color;      // lines, there's macro shenanigans
		write8(hi);         // going on.
		write8(lo);
	}
	CS_IDLE;
}

void Adafruit_TFTLCD::setRotation(uint8_t x) {

	// Call parent rotation func first -- sets up rotation flags, etc.
	Adafruit_GFX::setRotation(x);
	// Then perform hardware-specific rotation operations...

	CS_ACTIVE;
	if (driver == ID_9341) {
		// MEME, HX8357D uses same registers as 9341 but different values
		uint16_t t = 0;

		switch (rotation) {
		case 2:
			t = ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR;
			break;
		case 3:
			t = ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
			break;
		case 0:
			t = ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR;
			break;
		case 1:
			t = ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR;
			break;
		}
		writeRegister8(ILI9341_MADCTL, t ); // MADCTL
		// For 9341, init default full-screen address window:
		setAddrWindow(0, 0, _width - 1, _height - 1); // CS_IDLE happens here
	}
}


// Because this function is used infrequently, it configures the ports for
// the read operation, reads the data, then restores the ports to the write
// configuration.  Write operations happen a LOT, so it's advantageous to
// leave the ports in that state as a default.
uint16_t Adafruit_TFTLCD::readPixel(int16_t x, int16_t y) {

	if((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) return 0;

	CS_ACTIVE;
	if(driver == ID_932X) {

		uint8_t hi, lo;
		int16_t t;
		switch(rotation) {
		case 1:
			t = x;
			x = TFTWIDTH  - 1 - y;
			y = t;
			break;
		case 2:
			x = TFTWIDTH  - 1 - x;
			y = TFTHEIGHT - 1 - y;
			break;
		case 3:
			t = x;
			x = y;
			y = TFTHEIGHT - 1 - t;
			break;
		}
		writeRegister16(0x0020, x);
		writeRegister16(0x0021, y);
		// Inexplicable thing: sometimes pixel read has high/low bytes
		// reversed.  A second read fixes this.  Unsure of reason.  Have
		// tried adjusting timing in read8() etc. to no avail.
		for(uint8_t pass=0; pass<2; pass++) {
			CD_COMMAND; write8(0x00); write8(0x22); // Read data from GRAM
			CD_DATA;
			setReadDir();  // Set up LCD data port(s) for READ operations
			read8(hi);     // First 2 bytes back are a dummy read
			read8(hi);
			read8(hi);     // Bytes 3, 4 are actual pixel value
			read8(lo);
			setWriteDir(); // Restore LCD data port(s) to WRITE configuration
		}
		CS_IDLE;
		return ((uint16_t)hi << 8) | lo;

	} else if(driver == ID_7575) {

		uint8_t r, g, b;
		writeRegisterPair(HX8347G_COLADDRSTART_HI, HX8347G_COLADDRSTART_LO, x);
		writeRegisterPair(HX8347G_ROWADDRSTART_HI, HX8347G_ROWADDRSTART_LO, y);
		CD_COMMAND; write8(0x22); // Read data from GRAM
		setReadDir();  // Set up LCD data port(s) for READ operations
		CD_DATA;
		read8(r);      // First byte back is a dummy read
		read8(r);
		read8(g);
		read8(b);
		setWriteDir(); // Restore LCD data port(s) to WRITE configuration
		CS_IDLE;
		return (((uint16_t)r & 0xF8) << 8) |
				(((uint16_t)g & 0XFC) << 3) |
				(           b              >> 3);
	} else return 0;
}

// Ditto with the read/write port directions, as above.
uint16_t Adafruit_TFTLCD::readID(void) {

	uint8_t hi, lo;

	/*
  for (uint8_t i=0; i<128; i++) {
    Serial.print("$"); Serial.print(i, HEX);
    Serial.print(" = 0x"); Serial.println(readReg(i), HEX);
  }
	 */

/*	if (readReg(0x04) == 0x8000) { // eh close enough
		// setc!
		writeRegister24(HX8357D_SETC, 0xFF8357);
		delay(300);
		//Serial.println(readReg(0xD0), HEX);
		if (readReg(0xD0) == 0x990000) {
			return 0x8357;
		}
	}
*/
	uint16_t id = readReg(0xD3);
	if (id == 0x9341) {
		return id;
	}

	CS_ACTIVE;
	CD_COMMAND;
	write8(0x00);
	WR_STROBE;     // Repeat prior byte (0x00)
	setReadDir();  // Set up LCD data port(s) for READ operations
	CD_DATA;
	read8(hi);
	read8(lo);
	setWriteDir();  // Restore LCD data port(s) to WRITE configuration
	CS_IDLE;

	id = hi; id <<= 8; id |= lo;
	return id;
}

uint32_t Adafruit_TFTLCD::readReg(uint8_t r) {
	uint32_t id;
	uint8_t x;

	// try reading register #4
	CS_ACTIVE;
	CD_COMMAND;
	write8(r);
	setReadDir();  // Set up LCD data port(s) for READ operations
	CD_DATA;
	delayMicroseconds(50);
	read8(x);
	id = x;          // Do not merge or otherwise simplify
	id <<= 8;              // these lines.  It's an unfortunate
	read8(x);
	id  |= x;        // shenanigans that are going on.
	id <<= 8;              // these lines.  It's an unfortunate
	read8(x);
	id  |= x;        // shenanigans that are going on.
	id <<= 8;              // these lines.  It's an unfortunate
	read8(x);
	id  |= x;        // shenanigans that are going on.
	CS_IDLE;
	setWriteDir();  // Restore LCD data port(s) to WRITE configuration

	//Serial.print("Read $"); Serial.print(r, HEX);
	//Serial.print(":\t0x"); Serial.println(id, HEX);
	return id;
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Adafruit_TFTLCD::color565(uint8_t r, uint8_t g, uint8_t b) {
	return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// For I/O macros that were left undefined, declare function
// versions that reference the inline macros just once:

#ifndef write8
void Adafruit_TFTLCD::write8(uint8_t value) {
#if 0
	for (int i = 0; i < 8; ++i)
	{
		mraa_intel_edison_mmap_writeX(m_dataPinCtx[i], (value >> i) & 1);
	}

	WR_STROBE;
#elif 0
	mraa_intel_edison_mmap_write8(m_dataPinCtx[0], value);

	WR_STROBE;
#elif 0
    uint8_t offset = ((TFT_EDISON_DATA0 / 32) * sizeof(uint32_t));

    uint32_t set   = (uint32_t)((uint32_t)value << (TFT_EDISON_DATA0 % 32));
    uint32_t clear = (uint32_t)((uint32_t)((uint8_t)~value) << (TFT_EDISON_DATA0 % 32))
    		| (uint32_t)1 << (TFT_EDISON_WR % 32);
    uint32_t set2   = (uint32_t)((uint32_t)1 << (TFT_EDISON_WR % 32));
    *(volatile uint32_t*) (mmap_reg + offset + 0x34) = set;
    *(volatile uint32_t*) (mmap_reg + offset + 0x4c) = clear;
    *(volatile uint32_t*) (mmap_reg + offset + 0x34) = set2;
#else
    uint32_t set   = (uint32_t)((uint32_t)value << (TFT_EDISON_DATA0 % 32));
    uint32_t clear = (uint32_t)((uint32_t)((uint8_t)~value) << (TFT_EDISON_DATA0 % 32))
    		| (uint32_t)1 << (TFT_EDISON_WR % 32);
    uint32_t set2   = (uint32_t)((uint32_t)1 << (TFT_EDISON_WR % 32));
    *(volatile uint32_t*)mmap_reg_set = set;
    *(volatile uint32_t*)mmap_reg_clear = clear;
    *(volatile uint32_t*)mmap_reg_set = set2;
#endif
}
#endif

#ifndef read8
void Adafruit_TFTLCD::read8(uint8_t& result) {
	RD_ACTIVE;

	// Sleep minimum of 400 ns
	timespec step;
	step.tv_sec = 0;
	step.tv_nsec = 400;
	nanosleep(&step, nullptr);

	result = 0;
	for (int i = 0; i < 8; ++i)
	{
		result |= mraa_gpio_read(m_dataPinCtx[i]) >> i;
	}

	RD_IDLE;
}
#endif




#define SYSFS_CLASS_GPIO "/sys/class/gpio"
#define MAX_SIZE 64

static unsigned int outputen[] = {248,249,250,251,252,253,254,255,256,257,258,259,260,261,232,233,234,235,236,237};
static mraa_gpio_context outputPins[sizeof(outputen) / sizeof(outputen[0])] = {0};
static mraa_gpio_context tristate;

mraa_result_t
mraa_intel_edison_gpio_dir_preX(mraa_gpio_context dev, gpio_dir_t dir)
{
    if (dev->phy_pin >= 0) {
        int pin = dev->phy_pin;

        if (outputPins[pin] == 0)
        {
        	outputPins[pin] = mraa_gpio_init_raw(outputen[pin]);
			if (mraa_gpio_dir(outputPins[pin], MRAA_GPIO_OUT) != MRAA_SUCCESS)
				return MRAA_ERROR_INVALID_RESOURCE;
        }

        int output_val = 0;
        if (dir == MRAA_GPIO_OUT)
            output_val = 1;
        if (mraa_gpio_write(outputPins[pin], output_val) != MRAA_SUCCESS)
            return MRAA_ERROR_INVALID_RESOURCE;
    }
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_intel_edison_gpio_dir_postX(mraa_gpio_context dev, gpio_dir_t dir)
{
    return MRAA_SUCCESS;
}

mraa_result_t
mraa_gpio_dirX(mraa_gpio_context dev, gpio_dir_t dir)
{
	mraa_result_t pre_ret = mraa_intel_edison_gpio_dir_preX(dev,dir);
	if(pre_ret != MRAA_SUCCESS)
		return pre_ret;

    if (dev == NULL) {
        return MRAA_ERROR_INVALID_HANDLE;
    }
    if (dev->value_fp != -1) {
         close(dev->value_fp);
         dev->value_fp = -1;
    }
    char filepath[MAX_SIZE];
    snprintf(filepath, MAX_SIZE, SYSFS_CLASS_GPIO "/gpio%d/direction", dev->pin);

    int direction = open(filepath, O_RDWR);

    if (direction == -1) {
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    char bu[MAX_SIZE];
    int length;
    switch(dir) {
        case MRAA_GPIO_OUT:
            length = snprintf(bu, sizeof(bu), "out");
            break;
        case MRAA_GPIO_IN:
            length = snprintf(bu, sizeof(bu), "in");
            break;
        default:
            close(direction);
            return MRAA_ERROR_FEATURE_NOT_IMPLEMENTED;
    }

    if (write(direction, bu, length*sizeof(char)) == -1) {
        close(direction);
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    close(direction);
    return mraa_intel_edison_gpio_dir_postX(dev,dir);
}

#ifndef setWriteDir
void Adafruit_TFTLCD::setWriteDir(void) {
	if(tristate == nullptr)
	{
	    tristate = mraa_gpio_init_raw(214);
	    if (tristate == nullptr) {
	        printf("edison: Failed to initialise Arduino board TriState\n");
	    }
	    mraa_gpio_dir(tristate, MRAA_GPIO_OUT);
	}

    mraa_gpio_write(tristate, 0);

    for (int i = 0; i < 8; ++i)
	{
		mraa_gpio_dirX(m_dataPinCtx[i], MRAA_GPIO_OUT);
	}

    mraa_gpio_write(tristate, 1);
}
#endif

#ifndef setReadDir
void Adafruit_TFTLCD::setReadDir(void) {
	if(tristate == nullptr)
	{
	    tristate = mraa_gpio_init_raw(214);
	    if (tristate == nullptr) {
	        printf("edison: Failed to initialise Arduino board TriState\n");
	    }
	    mraa_gpio_dir(tristate, MRAA_GPIO_OUT);
	}

    mraa_gpio_write(tristate, 0);

    for (int i = 0; i < 8; ++i)
	{
		mraa_gpio_dirX(m_dataPinCtx[i], MRAA_GPIO_IN);
	}

    mraa_gpio_write(tristate, 1);
}
#endif

// Set value of TFT register: 8-bit address, 8-bit value
#define writeRegister8inline(a, d) { \
  CD_COMMAND; write8(a); CD_DATA; write8(d); }

// Set value of TFT register: 16-bit address, 16-bit value
// See notes at top about macro expansion, hence hi & lo temp vars
#define writeRegister16inline(a, d) { \
  uint8_t hi, lo; \
  hi = (a) >> 8; lo = (a); CD_COMMAND; write8(hi); write8(lo); \
  hi = (d) >> 8; lo = (d); CD_DATA   ; write8(hi); write8(lo); }

// Set value of 2 TFT registers: Two 8-bit addresses (hi & lo), 16-bit value
#define writeRegisterPairInline(aH, aL, d) { \
  uint8_t hi = (d) >> 8, lo = (d); \
  CD_COMMAND; write8(aH); CD_DATA; write8(hi); \
  CD_COMMAND; write8(aL); CD_DATA; write8(lo); }


#ifndef writeRegister8
void Adafruit_TFTLCD::writeRegister8(uint8_t a, uint8_t d) {
	writeRegister8inline(a, d);
}
#endif

#ifndef writeRegister16
void Adafruit_TFTLCD::writeRegister16(uint16_t a, uint16_t d) {
	writeRegister16inline(a, d);
}
#endif

#ifndef writeRegisterPair
void Adafruit_TFTLCD::writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d) {
	writeRegisterPairInline(aH, aL, d);
}
#endif


void Adafruit_TFTLCD::writeRegister24(uint8_t r, uint32_t d) {
	CS_ACTIVE;
	CD_COMMAND;
	write8(r);
	CD_DATA;
	delayMicroseconds(10);
	write8(d >> 16);
	delayMicroseconds(10);
	write8(d >> 8);
	delayMicroseconds(10);
	write8(d);
	CS_IDLE;
}


void Adafruit_TFTLCD::writeRegister32(uint8_t r, uint32_t d) {
	CS_ACTIVE;
	CD_COMMAND;
	write8(r);
	CD_DATA;
	delayMicroseconds(10);
	write8(d >> 24);
	delayMicroseconds(10);
	write8(d >> 16);
	delayMicroseconds(10);
	write8(d >> 8);
	delayMicroseconds(10);
	write8(d);
	CS_IDLE;
}
