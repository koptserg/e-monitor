
#ifndef EPDPAINT_H
#define EPDPAINT_H

// Display orientation
#define ROTATE_0            0
#define ROTATE_90           1
#define ROTATE_180          2
#define ROTATE_270          3

// Color inverse. 1 or 0 = set or reset a bit if set a colored pixel
#define IF_INVERT_COLOR     1

#define COLORED     0
#define UNCOLORED   1

#include "fonts.h"
extern    void PaintPaint(unsigned char* image, int width, int height);
extern    void PaintClear(int colored);
extern    int  PaintGetWidth(void);
extern    void PaintSetWidth(int width);
extern    int  PaintGetHeight(void);
extern    void PaintSetHeight(int height);
    int  GetRotate(void);
extern    void PaintSetRotate(int rotate);
extern    unsigned char* PaintGetImage(void);
extern    void PaintDrawAbsolutePixel(int x, int y, int colored);
extern    void PaintDrawPixel(int x, int y, int colored);
extern    void PaintDrawCharAt(int x, int y, char ascii_char, sFONT* font, int colored);
extern    void PaintDrawStringAt(int x, int y, const char* text, sFONT* font, int colored);
extern    void PaintDrawLine(int x0, int y0, int x1, int y1, int colored);
extern    void PaintDrawHorizontalLine(int x, int y, int width, int colored);
extern    void PaintDrawVerticalLine(int x, int y, int height, int colored);
extern    void PaintDrawRectangle(int x0, int y0, int x1, int y1, int colored);
extern    void PaintDrawFilledRectangle(int x0, int y0, int x1, int y1, int colored);
extern    void PaintDrawCircle(int x, int y, int radius, int colored);
extern    void PaintDrawFilledCircle(int x, int y, int radius, int colored);

//extern    unsigned char* image[896];
extern    unsigned char* image[672];
extern    int pwidth;
extern    int pheight;
extern    int protate;

#endif