#ifndef EPD2IN9_H
#define EPD2IN9_H

//#include "epdif.h"

// Display resolution
#define EPD_WIDTH       128
#define EPD_HEIGHT      296

// EPD2IN9 commands
#define DRIVER_OUTPUT_CONTROL                       0x01
#define BOOSTER_SOFT_START_CONTROL                  0x0C
#define GATE_SCAN_START_POSITION                    0x0F
#define DEEP_SLEEP_MODE                             0x10
#define DATA_ENTRY_MODE_SETTING                     0x11
#define SW_RESET                                    0x12
#define TEMPERATURE_SENSOR_CONTROL                  0x1A
#define MASTER_ACTIVATION                           0x20
#define DISPLAY_UPDATE_CONTROL_1                    0x21
#define DISPLAY_UPDATE_CONTROL_2                    0x22
#define WRITE_RAM                                   0x24
#define WRITE_VCOM_REGISTER                         0x2C
#define WRITE_LUT_REGISTER                          0x32
#define SET_DUMMY_LINE_PERIOD                       0x3A
#define SET_GATE_TIME                               0x3B
#define BORDER_WAVEFORM_CONTROL                     0x3C
#define SET_RAM_X_ADDRESS_START_END_POSITION        0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION        0x45
#define SET_RAM_X_ADDRESS_COUNTER                   0x4E
#define SET_RAM_Y_ADDRESS_COUNTER                   0x4F
#define TERMINATE_FRAME_READ_WRITE                  0xFF

extern const unsigned char lut_full_update[];
extern const unsigned char lut_partial_update[];

extern    unsigned long epd_width;
extern    unsigned long epd_height;

extern    int  EpdInit(const unsigned char* lut);
extern    void EpdSendCommand(unsigned char command);
extern    void EpdSendData(unsigned char data);
extern    void WaitUntilIdle(void);
extern    void Reset(void);

extern    void EpdSetFrameMemoryXY(const unsigned char* image_buffer,int x, int y, int image_width, int image_height);

extern    void EpdSetFrameMemory(const unsigned char* image_buffer);
extern    void EpdClearFrameMemory(unsigned char color);
extern    void EpdDisplayFrame(void);
extern    void Sleep(void);

extern    void EpdSetLut(const unsigned char* lut);
extern    void EpdSetMemoryArea(int x_start, int y_start, int x_end, int y_end);
extern    void EpdSetMemoryPointer(int x, int y);

extern    void DelayMs(unsigned int delaytime);

#endif /* EPD2IN9_H */

/* END OF FILE */