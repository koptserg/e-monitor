
#include <stdlib.h>
#include "epd2in9.h"
#include "epdpaint.h"
#include "epdtest.h"
#include "imagedata.h"

#include <stdio.h>
#include "version.h"


void EpdtestNotRefresh(void)
{
  EpdClearFrameMemory(0xFF); 
    
  //Rectangle
  PaintSetWidth(56);
  PaintSetHeight(42);
  PaintSetRotate(ROTATE_90);
  
  PaintClear(UNCOLORED);
  PaintDrawRectangle(0, 0, 40, 50, COLORED);
  PaintDrawLine(0, 0, 40, 50, COLORED);
  PaintDrawLine(40, 0, 0, 50, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 15, PaintGetWidth(), PaintGetHeight());

  //Filled Rectangle
  PaintClear(UNCOLORED);
  PaintDrawFilledRectangle(0, 0, 40, 50, COLORED);
  PaintDrawLine(0, 0, 40, 50, UNCOLORED);
  PaintDrawLine(40, 0, 0, 50, UNCOLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 82, PaintGetWidth(), PaintGetHeight());  

  //Circle
  PaintSetWidth(64);
  PaintSetHeight(64);
  PaintSetRotate(ROTATE_90);
  
  PaintClear(UNCOLORED);
  PaintDrawCircle(32, 32, 30, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 64, 5, PaintGetWidth(), PaintGetHeight());

  //Filled Circle
  PaintClear(UNCOLORED);
  PaintDrawFilledCircle(32, 32, 30, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 64, 72, PaintGetWidth(), PaintGetHeight());
  //
 
  EpdDisplayFrame();
}
/*
void EpdtestRefresh(uint8 occupancy, uint16 illuminance, uint16 temperature, uint16 humidity, uint16 pressure,  uint8 percentage, unsigned long time_now_s)
{
//  EpdSetFrameMemory(IMAGE_DATA);

  //Rectangle
  PaintSetWidth(54);
  PaintSetHeight(100);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawRectangle(0, 0, 96, 50, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 40, 4, PaintGetWidth(), PaintGetHeight());
  
  // clock init Firmware build date 20/08/2021 13:47
  //zclApp_DateCode[] = { 16, '2', '0', '/', '0', '8', '/', '2', '0', '2', '1', ' ', '1', '3', ':', '4', '7' };
  //zclApp_DateCodeNT[] = "20/08/2021 13:47";
  char time_string[] = {zclApp_DateCode[12], zclApp_DateCode[13], ':', zclApp_DateCode[15], zclApp_DateCode[16], '\0'};
  time_string[0] = time_now_s / 60 / 10 + zclApp_DateCode[12];
  time_string[1] = time_now_s / 60 % 10 + zclApp_DateCode[13];
  time_string[3] = time_now_s % 60 / 10 + zclApp_DateCode[15];
  time_string[4] = time_now_s % 60 % 10 + zclApp_DateCode[16];

  char date_string[] = {zclApp_DateCode[1], zclApp_DateCode[2], '/', zclApp_DateCode[4], zclApp_DateCode[5], '/', 
                        zclApp_DateCode[9], zclApp_DateCode[10], '\0'};

//  date_string[0] = time_now_s / 60 / 10 + zclApp_DateCode[12];
//  date_string[1] = time_now_s / 60 % 10 + zclApp_DateCode[13];
//  date_string[3] = time_now_s % 60 / 10 + zclApp_DateCode[15];
//  date_string[4] = time_now_s % 60 % 10 + zclApp_DateCode[16];
//  date_string[8] = time_now_s % 60 / 10 + zclApp_DateCode[15];
//  date_string[9] = time_now_s % 60 % 10 + zclApp_DateCode[16];


//  char time_string[] = {'0', '0', ':', '0', '0', '\0'};
//  time_string[0] = time_now_s / 60 / 10 + '0';
//  time_string[1] = time_now_s / 60 % 10 + '0';
//  time_string[3] = time_now_s % 60 / 10 + '0';
//  time_string[4] = time_now_s % 60 % 10 + '0';

  PaintSetWidth(24);
  PaintSetHeight(85);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, time_string, &Font24, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 70, 10, PaintGetWidth(), PaintGetHeight());
  
  PaintSetWidth(16);
  PaintSetHeight(90);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 50, 8, PaintGetWidth(), PaintGetHeight());
 
  // Occupancy
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);    
  PaintClear(UNCOLORED);
  if (occupancy == 0) {
    PaintDrawStringAt(0, 0, "UnOccupied", &Font16, COLORED);
  } else {
    PaintDrawStringAt(0, 0, "Occupied  ", &Font16, COLORED);
  }
  EpdSetFrameMemoryXY(PaintGetImage(), 90, 148, PaintGetWidth(), PaintGetHeight());
  
  //Illuminance
  char illum_string[] = {'0','0', '0', '0', '0', ' ', ' ','l', 'x','\0'};
  illum_string[0] = illuminance / 10000 % 10 + '0';
  illum_string[1] = illuminance / 1000 % 10 + '0';
  illum_string[2] = illuminance / 100 % 10 + '0';
  illum_string[3] = illuminance / 10 % 10 + '0';
  illum_string[4] = illuminance % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, illum_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 65, 148, PaintGetWidth(), PaintGetHeight());
  
  //temperature
  char temp_string[] = {'0', '0', '.', '0', '0', ' ', ' ','^', 'C','\0'};
  temp_string[0] = temperature / 1000 % 10 + '0';
  temp_string[1] = temperature / 100 % 10 + '0';
  temp_string[3] = temperature / 10 % 10 + '0';
  temp_string[4] = temperature % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, temp_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 49, 148, PaintGetWidth(), PaintGetHeight()); 
  
  //humidity
  char hum_string[] = {'0', '0', '.', '0', '0', ' ', ' ','%', ' ','\0'};
  hum_string[0] = humidity / 1000 % 10 + '0';
  hum_string[1] = humidity / 100 % 10 + '0';
  hum_string[3] = humidity / 10 % 10 + '0';
  hum_string[4] = humidity % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, hum_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 33, 148, PaintGetWidth(), PaintGetHeight()); 
  
  //pressure
  char pres_string[] = {'0', '0', '0', '0', '.', '0', ' ','h', 'P','\0'};
  pres_string[0] = pressure*10 / 10000 % 10 + '0';
  pres_string[1] = pressure*10 / 1000 % 10 + '0';
  pres_string[2] = pressure*10 / 100 % 10 + '0';
  pres_string[3] = pressure*10 / 10 % 10 + '0';
  pres_string[5] = pressure*10 % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, pres_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 17, 148, PaintGetWidth(), PaintGetHeight()); 
  
  //percentage
  char perc_string[] = {'0', '0', '0', '%', '\0'};
  perc_string[0] = percentage/2 / 100 % 10 + '0';
  perc_string[1] = percentage/2 / 10 % 10 + '0';
  perc_string[2] = percentage/2 % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, perc_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 17, 36, PaintGetWidth(), PaintGetHeight()); 

  EpdDisplayFrame();    
}
*/