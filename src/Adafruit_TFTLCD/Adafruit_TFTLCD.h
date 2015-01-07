// Graphics library by ladyada/adafruit with init code from Rossum
// MIT license

#ifndef _ADAFRUIT_TFTLCD_H_
#define _ADAFRUIT_TFTLCD_H_

#include "../Adafruit_GFX/Adafruit_GFX.h"
#include <mraa/gpio.h>


class Adafruit_TFTLCD : public Adafruit_GFX {
public:

	Adafruit_TFTLCD(void);

	void begin(uint16_t id);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawFastHLine(int16_t x0, int16_t y0, int16_t w, uint16_t color);
	void drawFastVLine(int16_t x0, int16_t y0, int16_t h, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t c);
	void fillScreen(uint16_t color);
	void reset(void);
	void setRegisters8(uint8_t *ptr, uint8_t n);
	void setRegisters16(uint16_t *ptr, uint8_t n);
	void setRotation(uint8_t x);
	void setAddrWindow(int x1, int y1, int x2, int y2);
	void pushColors(uint16_t *data, uint32_t len, bool first);

	uint16_t color565(uint8_t r, uint8_t g, uint8_t b);
	uint16_t readPixel(int16_t x, int16_t y);
	uint16_t readID(void);
	uint32_t readReg(uint8_t r);

private:

	inline void write8(uint8_t value);
	inline void write16(uint16_t value);

	void setWriteDir(void);
	void setReadDir(void);
	void writeRegister8(uint8_t a, uint8_t d);
	void writeRegister16(uint8_t a, uint16_t d);
	void writeRegister24(uint8_t a, uint32_t d);
	void writeRegister32(uint8_t a, uint32_t d);
	void writeRegisterPair(uint8_t aH, uint8_t aL, uint16_t d);

	void read8(uint8_t& result);

	void setLR(void);
	void flood(uint16_t color, uint32_t len);

	uint8_t driver;

	mraa_gpio_context m_csPinCtx;
	mraa_gpio_context m_cdPinCtx;
	mraa_gpio_context m_wrPinCtx;
	mraa_gpio_context m_rdPinCtx;
	mraa_gpio_context m_dataPinCtx[8];
};

#endif
