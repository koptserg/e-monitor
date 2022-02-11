
#include <stdlib.h>

#ifdef EPD3IN7

#include "epd3in7.h"
#include "bme280spi.h" // HalLcd_HW_Control(), HalLcd_HW_Write()
#include "imagedata.h"
#include "epdpaint.h"
#include "utils.h"

#define HAL_LCD_BUSY BNAME(HAL_LCD_BUSY_PORT, HAL_LCD_BUSY_PIN)

void DelayMs(unsigned int delaytime);

unsigned long epd_width = EPD_WIDTH;
unsigned long epd_height = EPD_HEIGHT;

void EpdInit(void) {  
  EpdReset();
  WaitUntilIdle();
  EpdSendCommand(SW_RESET);  //SWRESET
  WaitUntilIdle();
  
  EpdSendCommand(0x46); 
  EpdSendData(0xF7);
  WaitUntilIdle();
  EpdSendCommand(0x47);
  EpdSendData(0xF7);
  WaitUntilIdle(); 
  
  EpdSendCommand(DRIVER_OUTPUT_CONTROL);
  EpdSendData(0xDF);
  EpdSendData(0x01);
  EpdSendData(0x00);
  
  EpdSendCommand(SOURCE_VOLTAGE_CONTROL); // set source voltage
  EpdSendData(0x41);
  EpdSendData(0xA8);
  EpdSendData(0x32);
  
  EpdSendCommand(DATA_ENTRY_MODE_SETTING); // set data entry sequence
  EpdSendData(0x03);
  
  EpdSendCommand(BORDER_WAVEFORM_CONTROL);
  EpdSendData(0x60); //0x50 black, 0x60 white, 0x70 black,
  
  EpdSendCommand(BOOSTER_SOFT_START_CONTROL); // set booster strength
  EpdSendData(0xAE);
  EpdSendData(0xC7);
  EpdSendData(0xC3);
  EpdSendData(0xC0);
  EpdSendData(0xC0);  
  
  EpdSendCommand(BUILTINTEMPERATURE_SENSOR_CONTROL); // set internal sensor on
  EpdSendData(0x80);
  
  EpdSendCommand(WRITE_VCOM_REGISTER); // set vcom value
  EpdSendData(0x44);
  
  EpdSendCommand(OTP_SELECTION_CONTROL_1); // set display option, these setting turn on previous function
  EpdSendData(0x00);
  EpdSendData(0xff);
  EpdSendData(0xff);
  EpdSendData(0xff);
  EpdSendData(0xff);  
  EpdSendData(0x4f);
  EpdSendData(0xff);
  EpdSendData(0xff);
  EpdSendData(0xff);
  EpdSendData(0xff); 
  
  EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);

  EpdSendCommand(DISPLAY_UPDATE_CONTROL_2);
  EpdSendData(0xCF);

  EpdSetMemoryPointer(0, 0);
  WaitUntilIdle();

//  unsigned char image[672];

  PaintPaint(image, 0, 0);
}

void EpdInitFull(void) { 
  EpdInit();
  EpdSetLut(lut_full_update);
}


void EpdInitPartial(void) { 
  EpdInit();
  EpdSetLut(lut_partial_update);
}


void EpdSendCommand(unsigned char command) {
    HalLcd_HW_Control(command);
}


void EpdSendData(unsigned char data) {
    HalLcd_HW_Write(data);
}


void WaitUntilIdle(void) {  
  uint8 error_time = 20; // over 2.5 sec return
     while(HAL_LCD_BUSY == 1) {      //LOW: idle, HIGH: busy
        DelayMs(100);
        error_time = error_time - 1;
        if (error_time == 0){    
          EpdReset();
          return;
        }
    } 
}


void DelayMs(unsigned int delaytime) {
  while(delaytime--)
  {
    uint16 microSecs = 1000;
    while(microSecs--)
    {
      asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
    }
  }
}

const unsigned char lut_full_update[] =
{											
0x2A,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//1
0x05,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//2
0x2A,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//3
0x05,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//4
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//5
0x00,0x02,0x03,0x0A,0x00,0x02,0x06,0x0A,0x05,0x00,//6
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//7
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//8
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//9
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//10
0x22,0x22,0x22,0x22,0x22
};

const unsigned char lut_partial_update[] = {
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//1
0x01,0x2A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x0A,0x55,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//3
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//5
0x00,0x00,0x05,0x05,0x00,0x05,0x03,0x05,0x05,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//7
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//9
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x22,0x22,0x22,0x22,0x22
};

void EpdSetLut(const unsigned char *lut) {       
  unsigned char count;
  EpdSendCommand(WRITE_LUT_REGISTER);
  for(count=0; count<105; count++) 
    EpdSendData(lut[count]); 
  WaitUntilIdle();
}

void EpdSetFrameMemoryImageXY(const unsigned char* image_buffer, int x, int y, int image_width, int image_height, uint8 invert) {
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
    x &= 0xFFF8;
    image_width &= 0xFFF8;
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
 
    // send the image data
    for (int j = 0; j < y_end - y + 1; j++) {
        for (int i = 0; i < (x_end - x + 1) / 8; i++) {
          uint8 inv_image = image_buffer[i + j * (image_width / 8)];
          if (invert){
            inv_image ^= 0x00;  
          } else {  
            inv_image ^= 0xFF;
          }
          EpdSendData((uint8)( inv_image ));
        }
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
    x &= 0xFFF8;
    image_width &= 0xFFF8;
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
    // send the image data     
    for (int j = 0; j < y_end - y + 1; j++) {
        for (int i = 0; i < (x_end - x + 1) / 8; i++) {
            EpdSendData(image_buffer[i + j * (image_width / 8)]);
        }
    }
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

void EpdSetFrameMemoryBase(const unsigned char* image_buffer, uint8 invert) {
  EpdSetMemoryArea(0, 0, epd_width - 1, epd_height - 1);
  EpdSetMemoryPointer(0, 0);
  
  EpdSendCommand(WRITE_RAM);
  /* send the image data */
  for (int i = 0; i < epd_width / 8 * epd_height; i++) {
    uint8 inv_image = image_buffer[i];
    if (invert){
      inv_image ^= 0x00;  
    } else {  
      inv_image ^= 0xFF;
    }
    EpdSendData((uint8)( inv_image ));
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
    // x point must be the multiple of 8 or the last 3 bits will be ignored
    EpdSendData((x_start) & 0xFF);
    EpdSendData((x_start >> 8) & 0xFF);
    EpdSendData((x_end) & 0xFF);
    EpdSendData((x_end >> 8) & 0xFF);
    EpdSendCommand(SET_RAM_Y_ADDRESS_START_END_POSITION);
    EpdSendData(y_start & 0xFF);
    EpdSendData((y_start >> 8) & 0xFF);
    EpdSendData(y_end & 0xFF);
    EpdSendData((y_end >> 8) & 0xFF);
}


void EpdSetMemoryPointer(int x, int y) {
    EpdSendCommand(SET_RAM_X_ADDRESS_COUNTER);
    /* x point must be the multiple of 8 or the last 3 bits will be ignored */
    EpdSendData((x) & 0xFF);
    EpdSendData((x >> 8) & 0xFF);
    EpdSendCommand(SET_RAM_Y_ADDRESS_COUNTER);
    EpdSendData(y & 0xFF);
    EpdSendData((y >> 8) & 0xFF);
    WaitUntilIdle();
}


void EpdDisplayFramePartial(void) {
  EpdSendCommand(DISPLAY_UPDATE_CONTROL_2);
  EpdSendData(0xCF);
  EpdSendCommand(MASTER_ACTIVATION);
  WaitUntilIdle();
}

void EpdDisplayFrame(void) {
  EpdSendCommand(DISPLAY_UPDATE_CONTROL_2);
  EpdSendData(0xCF);
  EpdSendCommand(MASTER_ACTIVATION);
  WaitUntilIdle();
}

void EpdSleep(void) {
/*
    EpdSendCommand(0X50);
    EpdSendData(0xf7);
    EpdSendCommand(0X02);   //power off
    EpdSendCommand(0X07);   //deep sleep
    EpdSendData(0xA5);
*/
    EpdSendCommand(DEEP_SLEEP_MODE);
    EpdSendData(0x01);
}


void EpdReset(void) {
  HalLcd_HW_Init();   
}

#endif //end EPD3IN7