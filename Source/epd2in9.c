
#include <stdlib.h>
#include "epd2in9.h"
//#include "hal_lcd.h" // HalLcd_HW_Control(), HalLcd_HW_Write()
#include "bme280spi.h" // HalLcd_HW_Control(), HalLcd_HW_Write()
#include "imagedata.h"
#include "epdpaint.h"

unsigned long epd_width = EPD_WIDTH;
unsigned long epd_height = EPD_HEIGHT;

int EpdInit(const unsigned char* lut) {
    /* this calls the peripheral hardware interface, see epdif */
//    if (IfInit() != 0) {
//        return -1;
//    }
    /* EPD hardware init start */
    const unsigned char* epd_lut = lut;
  /* Perform reset */
//    Reset();
    EpdSendCommand(DRIVER_OUTPUT_CONTROL);
    EpdSendData((EPD_HEIGHT - 1) & 0xFF);
    EpdSendData(((EPD_HEIGHT - 1) >> 8) & 0xFF);
    EpdSendData(0x00);                     // GD = 0; SM = 0; TB = 0;
    EpdSendCommand(BOOSTER_SOFT_START_CONTROL);
    EpdSendData(0xD7);
    EpdSendData(0xD6);
    EpdSendData(0x9D);
    EpdSendCommand(WRITE_VCOM_REGISTER);
    EpdSendData(0xA8);                     // VCOM 7C
    EpdSendCommand(SET_DUMMY_LINE_PERIOD);
    EpdSendData(0x1A);                     // 4 dummy lines per gate
    EpdSendCommand(SET_GATE_TIME);
    EpdSendData(0x08);                     // 2us per line
    EpdSendCommand(DATA_ENTRY_MODE_SETTING);
    EpdSendData(0x03);                     // X increment; Y increment
    EpdSetLut(epd_lut);
    /* EPD hardware init end */
//    unsigned char image[896];
    unsigned char image[672];
    PaintPaint(image, 0, 0);
    return 0;
}

void EpdSendCommand(unsigned char command) {
//    DigitalWrite(dc_pin, LOW);
    HalLcd_HW_Control(command);
}

void EpdSendData(unsigned char data) {
//    DigitalWrite(dc_pin, HIGH);
    HalLcd_HW_Write(data);
}

/**
 *  @brief: Wait until the busy_pin goes LOW
 */
void WaitUntilIdle(void) {
//    while(DigitalRead(busy_pin) == HIGH) {      //LOW: idle, HIGH: busy
//        DelayMs(100);
//  DelayMs(1);
//    }  

}

void DelayMs(unsigned int delaytime) {
//    delay(delaytime);
  while(delaytime--)
  {
//    _delay_us(1000);
    uint16 microSecs = 1000;
    while(microSecs--)
    {
      asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
  }
}

/**
 *  @brief: set the look-up table register
 */
void EpdSetLut(const unsigned char* lut) {
    const unsigned char* epd_lut = lut;
    EpdSendCommand(WRITE_LUT_REGISTER);
    /* the length of look-up table is 30 bytes */
    for (int i = 0; i < 30; i++) {
        EpdSendData(epd_lut[i]);
    }
}

void EpdClearFrameMemory(unsigned char color) {
    EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
    EpdSetMemoryPointer(0, 0);
    EpdSendCommand(WRITE_RAM);
    /* send the color data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData(color);
    }
}

void EpdSetMemoryArea(int x_start, int y_start, int x_end, int y_end) {
    EpdSendCommand(SET_RAM_X_ADDRESS_START_END_POSITION);
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    EpdSendData((x_start >> 3) & 0xFF);
    EpdSendData((x_end >> 3) & 0xFF);
    EpdSendCommand(SET_RAM_Y_ADDRESS_START_END_POSITION);
    EpdSendData(y_start & 0xFF);
    EpdSendData((y_start >> 8) & 0xFF);
    EpdSendData(y_end & 0xFF);
    EpdSendData((y_end >> 8) & 0xFF);
}

void EpdSetMemoryPointer(int x, int y) {
    EpdSendCommand(SET_RAM_X_ADDRESS_COUNTER);
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    EpdSendData((x >> 3) & 0xFF);
    EpdSendCommand(SET_RAM_Y_ADDRESS_COUNTER);
    EpdSendData(y & 0xFF);
    EpdSendData((y >> 8) & 0xFF);
    WaitUntilIdle();
}

void EpdDisplayFrame(void) {
    EpdSendCommand(DISPLAY_UPDATE_CONTROL_2);
    EpdSendData(0xC4);
    EpdSendCommand(MASTER_ACTIVATION);
    EpdSendCommand(TERMINATE_FRAME_READ_WRITE);
    WaitUntilIdle();
}

void EpdSetFrameMemory(const unsigned char* image_buffer) {
    EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
    EpdSetMemoryPointer(0, 0);
    EpdSendCommand(WRITE_RAM);
    /* send the image data */
    for (int i = 0; i < epd_width / 8 * epd_height; i++) {
        EpdSendData((uint8)(image_buffer[i]));
    }
}

void EpdSetFrameMemoryXY(const unsigned char* image_buffer, int x, int y, int image_width, int image_height) {
    int x_end;
    int y_end;

    if (
        image_buffer == NULL ||
        x < 0 || image_width < 0 ||
        y < 0 || image_height < 0
    ) {
        return;
    }
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    x &= 0xF8;
    image_width &= 0xF8;
    if (x + image_width >= epd_width) {
        x_end = epd_width - 1;
    } else {
        x_end = x + image_width - 1;
    }
    if (y + image_height >= epd_height) {
        y_end = epd_height - 1;
    } else {
        y_end = y + image_height - 1;
    }
    EpdSetMemoryArea(x, y, x_end, y_end);
    EpdSetMemoryPointer(x, y);
    EpdSendCommand(WRITE_RAM);
    /* send the image data */
    for (int j = 0; j < y_end - y + 1; j++) {
        for (int i = 0; i < (x_end - x + 1) / 8; i++) {
            EpdSendData(image_buffer[i + j * (image_width / 8)]);
        }
    }
}

const unsigned char lut_full_update[] = {
    0x50, 0xAA, 0x55, 0xAA, 0x11, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFF, 0xFF, 0x1F, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char lut_partial_update[] = {
    0x10, 0x18, 0x18, 0x08, 0x18, 0x18,
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x13, 0x14, 0x44, 0x12,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};