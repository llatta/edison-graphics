/***************************************************
  This is our GFX example for the Adafruit ILI9341 Breakout and Shield
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

#include "Adafruit_GFX/Adafruit_GFX.h"
#include "Adafruit_ILI9341/Adafruit_ILI9341.h"
#include "Adafruit_TFTLCD/Adafruit_TFTLCD.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>

#define min(a, b) (((a) < (b)) ? (a) : (b))

int running = 0;

void
sig_handler(int signo)
{
	if (signo == SIGINT) {
		printf("closing\n");
		running = -1;
	}
}

unsigned long micros()
{
	timeval t;
	gettimeofday(&t, nullptr);

	return (unsigned long)((uint64_t)t.tv_sec * 1000000 + t.tv_usec);
}

static void delay(int milliseconds)
{
	usleep(milliseconds * 1000);
}

//#define TFT_USE_SPI

#ifdef TFT_USE_SPI

// For the Adafruit shield, these are the default.
#define TFT_CS 10
#define TFT_DC 9
#define TFT_RST 8

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

#else

Adafruit_TFTLCD tft;

#endif

void fillScreen(uint16_t color) {
	unsigned long start = micros();
	tft.fillScreen(color);
	printf("\n- fillColor %04x: %lu\n", color, micros() - start);
}
unsigned long testFillScreen() {
	unsigned long start = micros();
	fillScreen(ILI9341_BLACK);
	fillScreen(ILI9341_RED);
	fillScreen(ILI9341_GREEN);
	fillScreen(ILI9341_BLUE);
	fillScreen(ILI9341_BLACK);
	return micros() - start;
}

unsigned long testText() {
	tft.fillScreen(ILI9341_BLACK);
	unsigned long start = micros();
	tft.setCursor(0, 0);
	tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
	tft.println("Hello World!");
	tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
	//tft.println(1234.56);
	tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
	//tft.println(0xDEADBEEF, HEX);
	tft.println("");
	tft.setTextColor(ILI9341_GREEN);
	tft.setTextSize(5);
	tft.println("Groop");
	tft.setTextSize(2);
	tft.println("I implore thee,");
	tft.setTextSize(1);
	tft.println("my foonting turlingdromes.");
	tft.println("And hooptiously drangle me");
	tft.println("with crinkly bindlewurdles,");
	tft.println("Or I will rend thee");
	tft.println("in the gobberwarts");
	tft.println("with my blurglecruncheon,");
	tft.println("see if I don't!");
	return micros() - start;
}

unsigned long testLines(uint16_t color) {
	unsigned long start, t;
	int           x1, y1, x2, y2,
	w = tft.width(),
	h = tft.height();

	tft.fillScreen(ILI9341_BLACK);

	x1 = y1 = 0;
	y2    = h - 1;
	start = micros();
	for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
	x2    = w - 1;
	for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
	t     = micros() - start; // fillScreen doesn't count against timing

	tft.fillScreen(ILI9341_BLACK);

	x1    = w - 1;
	y1    = 0;
	y2    = h - 1;
	start = micros();
	for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
	x2    = 0;
	for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
	t    += micros() - start;

	tft.fillScreen(ILI9341_BLACK);

	x1    = 0;
	y1    = h - 1;
	y2    = 0;
	start = micros();
	for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
	x2    = w - 1;
	for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
	t    += micros() - start;

	tft.fillScreen(ILI9341_BLACK);

	x1    = w - 1;
	y1    = h - 1;
	y2    = 0;
	start = micros();
	for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
	x2    = 0;
	for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);

	return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2) {
	unsigned long start;
	int           x, y, w = tft.width(), h = tft.height();

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
	for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

	return micros() - start;
}

unsigned long testRects(uint16_t color) {
	unsigned long start;
	int           n, i, i2,
	cx = tft.width()  / 2,
	cy = tft.height() / 2;

	tft.fillScreen(ILI9341_BLACK);
	n     = min(tft.width(), tft.height());
	start = micros();
	for(i=2; i<n; i+=6) {
		i2 = i / 2;
		tft.drawRect(cx-i2, cy-i2, i, i, color);
	}

	return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
	unsigned long start, t = 0;
	int           n, i, i2,
	cx = tft.width()  / 2 - 1,
	cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	n = min(tft.width(), tft.height());
	for(i=n; i>0; i-=6) {
		i2    = i / 2;
		start = micros();
		tft.fillRect(cx-i2, cy-i2, i, i, color1);
		t    += micros() - start;
		// Outlines are not included in timing results
		tft.drawRect(cx-i2, cy-i2, i, i, color2);
	}

	return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
	unsigned long start;
	int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(x=radius; x<w; x+=r2) {
		for(y=radius; y<h; y+=r2) {
			tft.fillCircle(x, y, radius, color);
		}
	}

	return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
	unsigned long start;
	int           x, y, r2 = radius * 2,
			w = tft.width()  + radius,
			h = tft.height() + radius;

	// Screen is not cleared for this one -- this is
	// intentional and does not affect the reported time.
	start = micros();
	for(x=0; x<w; x+=r2) {
		for(y=0; y<h; y+=r2) {
			tft.drawCircle(x, y, radius, color);
		}
	}

	return micros() - start;
}

unsigned long testTriangles() {
	unsigned long start;
	int           n, i, cx = tft.width()  / 2 - 1,
			cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	n     = min(cx, cy);
	start = micros();
	for(i=0; i<n; i+=5) {
		tft.drawTriangle(
				cx    , cy - i, // peak
				cx - i, cy + i, // bottom left
				cx + i, cy + i, // bottom right
				tft.color565(0, 0, i));
	}

	return micros() - start;
}

unsigned long testFilledTriangles() {
	unsigned long start, t = 0;
	int           i, cx = tft.width()  / 2 - 1,
			cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(i=min(cx,cy); i>10; i-=5) {
		start = micros();
		tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
				tft.color565(0, i, i));
		t += micros() - start;
		tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
				tft.color565(i, i, 0));
	}

	return t;
}

unsigned long testRoundRects() {
	unsigned long start;
	int           w, i, i2,
	cx = tft.width()  / 2 - 1,
	cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	w     = min(tft.width(), tft.height());
	start = micros();
	for(i=0; i<w; i+=6) {
		i2 = i / 2;
		tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 0, 0));
	}

	return micros() - start;
}

unsigned long testFilledRoundRects() {
	unsigned long start;
	int           i, i2,
	cx = tft.width()  / 2 - 1,
	cy = tft.height() / 2 - 1;

	tft.fillScreen(ILI9341_BLACK);
	start = micros();
	for(i=min(tft.width(), tft.height()); i>20; i-=6) {
		i2 = i / 2;
		tft.fillRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(0, i, 0));
	}

	return micros() - start;
}

//#include "images/marci_320x240_srgb.h"
#include "images/videocalib.h"

unsigned long testDrawImage() {
	const void* image = imageData;
	//size_t imageSize = sizeof(imageData);
	//printf("Image size: %u\n", imageSize);

	tft.setRotation(1);

	size_t numPixel = tft.width() * tft.height();
	//assert(numPixel * 3 == imageSize);
	uint16_t* image16 = (uint16_t*)malloc(tft.width() * tft.height() * sizeof(uint16_t));

	for(int y = 0; y < tft.height(); ++y)
	{
		for(int x = 0; x < tft.width(); ++x)
		{
			uint8_t* i = ((uint8_t*)image) + (y * tft.width() + x) * 3;
			uint8_t r = *(i++);
			uint8_t g = *(i++);
			uint8_t b = *(i++);

			image16[y * tft.width() + x] = tft.color565(r, g, b);
		}
	}

	unsigned long start = micros();
	tft.pushColors(image16, numPixel, true);
	unsigned long duration = micros() - start;

	free(image16);

	return duration;
}

int main (int argc, char **argv)
{
	signal(SIGINT, sig_handler);

	printf("ILI9341 Test!\n");

#ifdef TFT_USE_SPI
	tft.begin();

	// read diagnostics (optional but can help debug problems)
	uint8_t x = tft.readcommand8(ILI9341_RDMODE);
	printf("Display Power Mode: 0x%02x\n", x);
	x = tft.readcommand8(ILI9341_RDMADCTL);
	printf("MADCTL Mode: 0x%02x\n", x);
	x = tft.readcommand8(ILI9341_RDPIXFMT);
	printf("Pixel Format: 0x%02x\n", x);
	x = tft.readcommand8(ILI9341_RDIMGFMT);
	printf("Image Format: 0x%02x\n", x);
	x = tft.readcommand8(ILI9341_RDSELFDIAG);
	printf("Self Diagnostic: 0x%02x\n", x);
#else
	tft.reset();

	//uint16_t identifier = tft.readID();
	uint16_t identifier = 0x9341;

	if (identifier == 0x9341) {
		printf("Found ILI9341 LCD driver\n");
	} else {
		printf("Unknown LCD driver chip: 0x%x\n", identifier);
		return 1;
	}

	tft.begin(identifier);
#endif

	printf("Benchmark                Time (microseconds)\n");

	printf("Screen fill              ");
	printf("%lu\n", testFillScreen());
	delay(500);

	printf("Draw image               ");
	printf("%lu\n", testDrawImage());
	delay(500);

	printf("Text                     ");
	printf("%lu\n", testText());
	delay(3000);

	printf("Lines                    ");
	printf("%lu\n", testLines(ILI9341_CYAN));
	delay(500);

	printf("Horiz/Vert Lines         ");
	printf("%lu\n", testFastLines(ILI9341_RED, ILI9341_BLUE));
	delay(500);

	printf("Rectangles (outline)     ");
	printf("%lu\n", testRects(ILI9341_GREEN));
	delay(500);

	printf("Rectangles (filled)      ");
	printf("%lu\n", testFilledRects(ILI9341_YELLOW, ILI9341_MAGENTA));
	delay(500);

	printf("Circles (filled)         ");
	printf("%lu\n", testFilledCircles(10, ILI9341_MAGENTA));

	printf("Circles (outline)        ");
	printf("%lu\n", testCircles(10, ILI9341_WHITE));
	delay(500);

	printf("Triangles (outline)      ");
	printf("%lu\n", testTriangles());
	delay(500);

	printf("Triangles (filled)       ");
	printf("%lu\n", testFilledTriangles());
	delay(500);

	printf("Rounded rects (outline)  ");
	printf("%lu\n", testRoundRects());
	delay(500);

	printf("Rounded rects (filled)   ");
	printf("%lu\n", testFilledRoundRects());
	delay(500);

	printf("Done!\n");


	while (running == 0) {
		for(uint8_t rotation=0; rotation<4; rotation++) {
			tft.setRotation(rotation);
			testText();
			sleep(1);
		}
	}

	return 1;
}
