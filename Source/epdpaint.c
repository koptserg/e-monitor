
//#include <avr/pgmspace.h>
#include "epdpaint.h"

unsigned char* pimage;
int pwidth ;
int pheight ;
int protate ;

void PaintPaint(unsigned char* image, int width, int height) {
    protate = ROTATE_0;
    pimage = image;
    /* 1 byte = 8 pixels, so the width should be the multiple of 8 */
    pwidth = width % 8 ? width + 8 - (width % 8) : width;
    pheight = height;
}

void PaintSetWidth(int width) {
    pwidth = width % 8 ? width + 8 - (width % 8) : width;
}

void PaintSetHeight(int height) {
    pheight = height;
}

void PaintSetRotate(int rotate){
    protate = rotate;
}

void PaintClear(int colored) {
    for (int x = 0; x < pwidth; x++) {
        for (int y = 0; y < pheight; y++) {
            PaintDrawAbsolutePixel(x, y, colored);
        }
    }
}

void PaintDrawStringAt(int x, int y, const char* text, sFONT* font, int colored) {
    const char* p_text = text;
    unsigned int counter = 0;
    int refcolumn = x;
    
    /* Send the string character by character on EPD */
    while (*p_text != 0) {
        /* Display one character on EPD */
        PaintDrawCharAt(refcolumn, y, *p_text, font, colored);
        /* Decrement the column position by 16 */
        refcolumn += font->Width;
        /* Point on the next character */
        p_text++;
        counter++;
    }
}
        
void PaintDrawCharAt(int x, int y, char ascii_char, sFONT* font, int colored) {
    int i, j;
    unsigned int char_offset = (ascii_char - ' ') * font->Height * (font->Width / 8 + (font->Width % 8 ? 1 : 0));
    const unsigned char* ptr = &font->table[char_offset];

    for (j = 0; j < font->Height; j++) {
        for (i = 0; i < font->Width; i++) {
          if (*ptr & (0x80 >> (i % 8))) {     
                PaintDrawPixel(x + i, y + j, colored);
            }
            if (i % 8 == 7) {
                ptr++;
            }
        }
        if (font->Width % 8 != 0) {
            ptr++;
        }
    }
}

void PaintDrawPixel(int x, int y, int colored) {
    int point_temp;
    if (protate == ROTATE_0) {
        if(x < 0 || x >= pwidth || y < 0 || y >= pheight) {
            return;
        }
        PaintDrawAbsolutePixel(x, y, colored);
    } else if (protate == ROTATE_90) {
        if(x < 0 || x >= pheight || y < 0 || y >= pwidth) {
          return;
        }
        point_temp = x;
        x = pwidth - y;
        y = point_temp;
        PaintDrawAbsolutePixel(x, y, colored);
    } else if (protate == ROTATE_180) {
        if(x < 0 || x >= pwidth || y < 0 || y >= pheight) {
          return;
        }
        x = pwidth - x;
        y = pheight - y;
        PaintDrawAbsolutePixel(x, y, colored);
    } else if (protate == ROTATE_270) {
        if(x < 0 || x >= pheight || y < 0 || y >= pwidth) {
          return;
        }
        point_temp = x;
        x = y;
        y = pheight - point_temp;
        PaintDrawAbsolutePixel(x, y, colored);
    }
}

void PaintDrawAbsolutePixel(int x, int y, int colored) {
    if (x < 0 || x >= pwidth || y < 0 || y >= pheight) {
        return;
    }
    if (IF_INVERT_COLOR) {
        if (colored) {
            pimage[(x + y * pwidth) / 8] |= 0x80 >> (x % 8);
        } else {
            pimage[(x + y * pwidth) / 8] &= ~(0x80 >> (x % 8));
        }
    } else {
        if (colored) {
            pimage[(x + y * pwidth) / 8] &= ~(0x80 >> (x % 8));
        } else {
            pimage[(x + y * pwidth) / 8] |= 0x80 >> (x % 8);
        }
    }
}

unsigned char* PaintGetImage(void) {
    return pimage;
}

int PaintGetWidth(void) {
    return pwidth;
}

int PaintGetHeight(void) {
    return pheight;
}

void PaintDrawRectangle(int x0, int y0, int x1, int y1, int colored) {
    int min_x, min_y, max_x, max_y;
    min_x = x1 > x0 ? x0 : x1;
    max_x = x1 > x0 ? x1 : x0;
    min_y = y1 > y0 ? y0 : y1;
    max_y = y1 > y0 ? y1 : y0;

    if (protate == ROTATE_0) {
      PaintDrawHorizontalLine(min_x, min_y, max_x - min_x + 1, colored);
    } else if (protate == ROTATE_90) {      
      PaintDrawHorizontalLine(min_x, min_y+1, max_x - min_x + 1, colored);
    }
    PaintDrawHorizontalLine(min_x, max_y, max_x - min_x + 1, colored);
    PaintDrawVerticalLine(min_x, min_y, max_y - min_y + 1, colored);
    PaintDrawVerticalLine(max_x, min_y, max_y - min_y + 1, colored);
}

void PaintDrawHorizontalLine(int x, int y, int line_width, int colored) {
    int i;
    for (i = x; i < x + line_width; i++) {
        PaintDrawPixel(i, y, colored);
    }
}

void PaintDrawVerticalLine(int x, int y, int line_height, int colored) {
    int i;
    for (i = y; i < y + line_height; i++) {
        PaintDrawPixel(x, i, colored);
    }
}

void PaintDrawLine(int x0, int y0, int x1, int y1, int colored) {
    /* Bresenham algorithm */
    int dx = x1 - x0 >= 0 ? x1 - x0 : x0 - x1;
    int sx = x0 < x1 ? 1 : -1;
    int dy = y1 - y0 <= 0 ? y1 - y0 : y0 - y1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;

    while((x0 != x1) && (y0 != y1)) {
        PaintDrawPixel(x0, y0 , colored);
        if (2 * err >= dy) {     
            err += dy;
            x0 += sx;
        }
        if (2 * err <= dx) {
            err += dx; 
            y0 += sy;
        }
    }
}

void PaintDrawFilledRectangle(int x0, int y0, int x1, int y1, int colored) {
    int min_x, min_y, max_x, max_y;
    int i;
    min_x = x1 > x0 ? x0 : x1;
    max_x = x1 > x0 ? x1 : x0;
    min_y = y1 > y0 ? y0 : y1;
    max_y = y1 > y0 ? y1 : y0;
    
    for (i = min_x; i <= max_x; i++) {
      PaintDrawVerticalLine(i, min_y, max_y - min_y + 1, colored);
    }
}
      
void PaintDrawCircle(int x, int y, int radius, int colored) {
    /* Bresenham algorithm */
    int x_pos = -radius;
    int y_pos = 0;
    int err = 2 - 2 * radius;
    int e2;

    do {
        PaintDrawPixel(x - x_pos, y + y_pos, colored);
        PaintDrawPixel(x + x_pos, y + y_pos, colored);
        PaintDrawPixel(x + x_pos, y - y_pos, colored);
        PaintDrawPixel(x - x_pos, y - y_pos, colored);
        e2 = err;
        if (e2 <= y_pos) {
            err += ++y_pos * 2 + 1;
            if(-x_pos == y_pos && e2 <= x_pos) {
              e2 = 0;
            }
        }
        if (e2 > x_pos) {
            err += ++x_pos * 2 + 1;
        }
    } while (x_pos <= 0);
}
              
void PaintDrawFilledCircle(int x, int y, int radius, int colored) {
    /* Bresenham algorithm */
    int x_pos = -radius;
    int y_pos = 0;
    int err = 2 - 2 * radius;
    int e2;

    do {
        PaintDrawPixel(x - x_pos, y + y_pos, colored);
        PaintDrawPixel(x + x_pos, y + y_pos, colored);
        PaintDrawPixel(x + x_pos, y - y_pos, colored);
        PaintDrawPixel(x - x_pos, y - y_pos, colored);
        PaintDrawHorizontalLine(x + x_pos, y + y_pos, 2 * (-x_pos) + 1, colored);
        PaintDrawHorizontalLine(x + x_pos, y - y_pos, 2 * (-x_pos) + 1, colored);
        e2 = err;
        if (e2 <= y_pos) {
            err += ++y_pos * 2 + 1;
            if(-x_pos == y_pos && e2 <= x_pos) {
                e2 = 0;
            }
        }
        if(e2 > x_pos) {
            err += ++x_pos * 2 + 1;
        }
    } while(x_pos <= 0);
}