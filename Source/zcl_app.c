
#include "AF.h"
#include "OSAL.h"
#include "OSAL_Clock.h"
#include "OSAL_PwrMgr.h"
#include "ZComDef.h"
#include "ZDApp.h"
#include "ZDNwkMgr.h"
#include "ZDObject.h"
#include "math.h"

#include "nwk_util.h"
#include "zcl.h"
#include "zcl_app.h"
#include "zcl_diagnostic.h"
#include "zcl_general.h"
#include "zcl_lighting.h"
#include "zcl_ms.h"

#include "bdb.h"
#include "bdb_interface.h"
#include "gp_interface.h"

#include "Debug.h"

#include "OnBoard.h"

/* HAL */
#include "hal_adc.h"
#include "hal_drivers.h"
#include "hal_i2c.h"
#include "hal_key.h"
#include "hal_led.h"

#include "bme280spi.h"
#include "bh1750.h"
#include "battery.h"
#include "commissioning.h"
#include "factory_reset.h"
#include "utils.h"
#include "version.h"

#ifdef EPD2IN9
#include "epd2in9.h"
#endif
#ifdef EPD2IN9V2
#include "epd2in9v2.h"
#endif
#ifdef EPD2IN13V2
#include "epd2in13v2.h"
#endif
#ifdef EPD1IN54V2
#include "epd1in54v2.h"
#endif

#include "imagedata.h"
#include "epdpaint.h"

/*********************************************************************
 * MACROS
 */
#ifndef MOTION_PORT
#define MOTION_PORT 1 
#endif   
#ifndef MOTION_PIN
#define MOTION_PIN  3
#endif   
#define MOTION BNAME(MOTION_PORT, MOTION_PIN)

#ifndef MOTION_POWER_PORT
#define MOTION_POWER_PORT 1 
#endif   
#ifndef MOTION_POWER_PIN
#define MOTION_POWER_PIN  0
#endif 
 
//#define HAL_KEY_P0_EDGE_BITS HAL_KEY_BIT0

#define HAL_KEY_P1_EDGE_BITS (HAL_KEY_BIT1 | HAL_KEY_BIT2)

//#define HAL_KEY_CODE_RELEASE_KEY HAL_KEY_CODE_NOKEY

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

extern bool requestNewTrustCenterLinkKey;
byte zclApp_TaskID;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint8 currentSensorsReadingPhase = 0;

uint8 report = 0;
uint8 power = 0;
bool bmeDetect = 0;
uint8 motionDetect = 0;
uint8 bh1750Detect = 0;
uint8 BH1750_mode = ONE_TIME_HIGH_RES_MODE;

uint16 temp_bh1750IlluminanceSensor_MeasuredValue;
uint16 temp_Temperature_Sensor_MeasuredValue;
uint16 temp_PressureSensor_MeasuredValue;
uint16 temp_PressureSensor_ScaledValue;
uint16 temp_HumiditySensor_MeasuredValue;

//unsigned long time_now_s;
uint16 time_now_s;
uint16 date_now;
//uint8 month_now;
//uint8 year_now;
uint8 fullupdate_hour = 0;

afAddrType_t inderect_DstAddr = {.addrMode = (afAddrMode_t)AddrNotPresent, .endPoint = 0, .addr.shortAddr = 0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclApp_HandleKeys(byte shift, byte keys);
static void zclApp_Report(void);

static void zclApp_BasicResetCB(void);
static void zclApp_RestoreAttributesFromNV(void);
static void zclApp_SaveAttributesToNV(void);
static void zclApp_StopReloadTimer(void);
static void zclApp_StartReloadTimer(void);

static ZStatus_t zclApp_ReadWriteAuthCB(afAddrType_t *srcAddr, zclAttrRec_t *pAttr, uint8 oper);

static void zclApp_ReadSensors(void);
static void zclApp_ReadBME280Temperature(void);
static void zclApp_ReadBME280Pressure(void);
static void zclApp_ReadBME280Humidity(void);
static void zclApp_bh1750StartLumosity(void);
static void zclApp_bh1750ReadLumosity(void);
static void zclApp_bh1750setMTreg(void);
static void zclApp_MotionPullUpDown(void);

static void EpdRefresh(void);
void EpdtestRefresh(void);
static void _delay_us(uint16 microSecs);
static void _delay_ms(uint16 milliSecs);

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclApp_CmdCallbacks = {
    zclApp_BasicResetCB, // Basic Cluster Reset command
    NULL, // Identify Trigger Effect command
    NULL, // On/Off cluster commands
    NULL, // On/Off cluster enhanced command Off with Effect
    NULL, // On/Off cluster enhanced command On with Recall Global Scene
    NULL, // On/Off cluster enhanced command On with Timed Off
    NULL, // RSSI Location command
    NULL  // RSSI Location Response command
};

void zclApp_MotionPullUpDown(void) {
    if (zclApp_Occupied == 1){
      P2INP &= ~HAL_KEY_BIT6; // pull up port1
//      IO_PUD_PORT(MOTION_PORT, IO_PUP); // pull up port1
      MicroWait(50);
      PICTL |= HAL_KEY_P1_EDGE_BITS; // set falling edge on port
    } else {
      P2INP |= HAL_KEY_BIT6; // pull down port1
//      IO_PUD_PORT(MOTION_PORT, IO_PDN); // pull down port1
      MicroWait(50);
      PICTL &= ~(HAL_KEY_P1_EDGE_BITS);     
    }
}

void zclApp_Init(byte task_id) {
    zclApp_RestoreAttributesFromNV();
    
    P1SEL &= ~BV(0); // Set P1_0 to GPIO
//    IO_FUNC_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_GIO); // Set P1_0 to GPIO
    P1DIR |= BV(0); // P1_0 output
//    IO_DIR_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_OUT); // P1_0 output
    P1 &= ~BV(0);   // power off DD //--
//    BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 0; // P1_0 = 0 power off DD
    
    motionDetect = P1_3;
//    motionDetect = MOTION;
    zclApp_Occupied_OnOff = motionDetect;
    zclApp_Occupied = motionDetect;
    zclApp_MotionPullUpDown();
    
    SPIInit();
    
    bmeDetect = BME280Init();
    
    HalI2CInit();
    bh1750Detect = bh1750_init(BH1750_mode);
    zclApp_bh1750setMTreg();   
    // this is important to allow connects throught routers
    // to make this work, coordinator should be compiled with this flag #define TP2_LEGACY_ZC
    requestNewTrustCenterLinkKey = FALSE;

    zclApp_TaskID = task_id;

    zclGeneral_RegisterCmdCallbacks(1, &zclApp_CmdCallbacks);
    zcl_registerAttrList(zclApp_FirstEP.EndPoint, zclApp_AttrsFirstEPCount, zclApp_AttrsFirstEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FirstEP);
    zcl_registerReadWriteCB(zclApp_FirstEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
/*
    zcl_registerAttrList(zclApp_SecondEP.EndPoint, zclApp_AttrsSecondEPCount, zclApp_AttrsSecondEP);
    bdb_RegisterSimpleDescriptor(&zclApp_SecondEP);
    zcl_registerReadWriteCB(zclApp_SecondEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
*/    
    zcl_registerAttrList(zclApp_ThirdEP.EndPoint, zclApp_AttrsThirdEPCount, zclApp_AttrsThirdEP);
    bdb_RegisterSimpleDescriptor(&zclApp_ThirdEP);
    zcl_registerReadWriteCB(zclApp_ThirdEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);
    
    zcl_registerAttrList(zclApp_FourthEP.EndPoint, zclApp_AttrsFourthEPCount, zclApp_AttrsFourthEP);
    bdb_RegisterSimpleDescriptor(&zclApp_FourthEP);   
    zcl_registerReadWriteCB(zclApp_FourthEP.EndPoint, NULL, zclApp_ReadWriteAuthCB);

    zcl_registerForMsg(zclApp_TaskID);

    // Register for all key events - This app will handle all key events
    RegisterForKeys(zclApp_TaskID);
//    LREP("Started build %s \r\n", zclApp_DateCodeNT);
   
    zclApp_StartReloadTimer();
    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_CLOCK_EVT, 60000);
    
  // epd full screen
  EpdInitFull();

  EpdSetFrameMemoryBase(IMAGE_DATA);
  EpdDisplayFrame();
  _delay_ms(2000);
  EpdClearFrameMemory(0xFF);
  EpdDisplayFrame();
  EpdClearFrameMemory(0xFF);
  EpdDisplayFrame();
  
  // epd partial screen

  EpdInitPartial();
  EpdClearFrameMemory(0xFF);
  EpdDisplayFramePartial();
  EpdClearFrameMemory(0xFF);
  EpdDisplayFramePartial();

//  time_now_s = (('1'-48)*10 + ('3'-48))*60 + ('4'-48)*10 + ('7'-48);
//  date_now =(('0'-48)*10 + ('0'-48))*360 + (('1'-48)*10 + ('1'-48))*30 + ('2'-48)*10 + ('7'-48);
  time_now_s = ((zclApp_BatteryManu[12]-48)*10 + (zclApp_BatteryManu[13]-48))*60 + (zclApp_BatteryManu[15]-48)*10 + (zclApp_BatteryManu[16]-48);
  date_now =((zclApp_BatteryManu[4]-48)*10 + (zclApp_BatteryManu[5]-48))*30 + (zclApp_BatteryManu[1]-48)*10 + (zclApp_BatteryManu[2]-48);

  EpdRefresh();
}

uint16 zclApp_event_loop(uint8 task_id, uint16 events) {
    afIncomingMSGPacket_t *MSGpkt;
    devStates_t zclApp_NwkState; //---
    
    (void)task_id; // Intentionally unreferenced parameter
    if (events & SYS_EVENT_MSG) {
        while ((MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(zclApp_TaskID))) {
            switch (MSGpkt->hdr.event) {
            case KEY_CHANGE:
                zclApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
                
                break;
            case ZDO_STATE_CHANGE:
                zclApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                LREP("NwkState=%d\r\n", zclApp_NwkState);
                if (zclApp_NwkState == DEV_END_DEVICE) {
                  IEN2 |= HAL_KEY_BIT4; // enable port1 int
                  P1DIR |=  BV(0); // P1_0 output
//                  IO_DIR_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_OUT); // P1_0 output
                  P1 |=  BV(0);   // power on DD
//                  BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 1; // power on DD
                } else {
                  IEN2 &= ~HAL_KEY_BIT4; // disable port1 int
                  P1 &= ~BV(0);   // power off DD //--
//                  BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 0; // power off DD 
                  P1DIR &= ~BV(0); // P1_0 input
//                  IO_DIR_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_IN); // P1_0 input
                }
                break;
            case ZCL_INCOMING_MSG:
                if (((zclIncomingMsg_t *)MSGpkt)->attrCmd) {
                    osal_mem_free(((zclIncomingMsg_t *)MSGpkt)->attrCmd);
                }
                break;

            default:
                break;
            }
            // Release the memory
            osal_msg_deallocate((uint8 *)MSGpkt);
        }
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if (events & APP_REPORT_CLOCK_EVT) {
        LREPMaster("APP_REPORT_CLOCK_EVT\r\n");
        time_now_s = time_now_s +1;
        fullupdate_hour = fullupdate_hour +1;
        if (fullupdate_hour == 5){
          fullupdate_hour = 0;
          // epd full screen
          EpdInitFull();
          EpdClearFrameMemory(0xFF);
          EpdDisplayFrame();
          EpdClearFrameMemory(0xFF);
          EpdDisplayFrame();
          // epd partial screen
          EpdInitPartial();
          EpdClearFrameMemory(0xFF);
          EpdDisplayFramePartial();
          EpdClearFrameMemory(0xFF);
          EpdDisplayFramePartial();
        }
        EpdRefresh();

        return (events ^ APP_REPORT_CLOCK_EVT);
    }

    if (events & APP_REPORT_TEMPERATURE_EVT) {
        LREPMaster("APP_REPORT_TEMPERATURE_EVT\r\n");
        report = 0;
        zclApp_ReadBME280Temperature();
        
        return (events ^ APP_REPORT_TEMPERATURE_EVT);
    }
    
    if (events & APP_REPORT_PRESSURE_EVT) {
        LREPMaster("APP_REPORT_PRESSURE_EVT\r\n");
        report = 0;
        zclApp_ReadBME280Pressure();
   
        return (events ^ APP_REPORT_PRESSURE_EVT);
    }
    
    if (events & APP_REPORT_HUMIDITY_EVT) {
        LREPMaster("APP_REPORT_HUMIDITY_EVT\r\n");
        report = 0;
        zclApp_ReadBME280Humidity();
   
        return (events ^ APP_REPORT_HUMIDITY_EVT);
    }
    
    if (events & APP_REPORT_ILLUMINANCE_EVT) {
        LREPMaster("APP_REPORT_ILLUMINANCE_EVT\r\n");       
        report = 0;
        zclApp_bh1750StartLumosity();
           
        return (events ^ APP_REPORT_ILLUMINANCE_EVT);
    }
    
    if (events & APP_REPORT_BATTERY_EVT) {
        LREPMaster("APP_REPORT_BATTERY_EVT\r\n");
        report = 0;
        zclBattery_Report();

        return (events ^ APP_REPORT_BATTERY_EVT);
    }
    
    if (events & APP_REPORT_EVT) {
        LREPMaster("APP_REPORT_EVT\r\n");
        report = 1;
        zclApp_Report();
        return (events ^ APP_REPORT_EVT);
    }

    if (events & APP_READ_SENSORS_EVT) {
        LREPMaster("APP_READ_SENSORS_EVT\r\n");
        zclApp_ReadSensors();
        return (events ^ APP_READ_SENSORS_EVT);
    }
        
    if (events & APP_MOTION_ON_EVT) {
        LREPMaster("APP_MOTION_ON_EVT\r\n");
        P1 &= ~BV(0);   // power off motion
//        BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 0; //power off motion
        osal_start_timerEx(zclApp_TaskID, APP_MOTION_DELAY_EVT, (uint32)zclApp_Config.PirOccupiedToUnoccupiedDelay * 1000);
        LREPMaster("START_DELAY\r\n");
        //report
        zclApp_Occupied = 1;
        zclApp_Occupied_OnOff = 1;
        zclGeneral_SendOnOff_CmdOn(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
        EpdRefresh();
        
        return (events ^ APP_MOTION_ON_EVT);
    }
    
    if (events & APP_MOTION_OFF_EVT) {
        LREPMaster("APP_MOTION_OFF_EVT\r\n");
        //report    
        zclApp_Occupied = 0;
        zclApp_Occupied_OnOff = 0;
        zclGeneral_SendOnOff_CmdOff(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
        EpdRefresh();
        
        return (events ^ APP_MOTION_OFF_EVT);
    }
    
    if (events & APP_MOTION_DELAY_EVT) {
        LREPMaster("APP_MOTION_DELAY_EVT\r\n");
        power = 2;       
        P1DIR |=  BV(0); // P1_0 output
//        IO_DIR_PORT_PIN(MOTION_POWER_PORT, MOTION_POWER_PIN, IO_OUT); // P1_0 output
        P1 |=  BV(0);   // power on motion
//        BNAME(MOTION_POWER_PORT, MOTION_POWER_PIN)= 1; // power on motion
        
        return (events ^ APP_MOTION_DELAY_EVT);
    }

    if (events & APP_EPD_DELAY_EVT) {
        LREPMaster("APP_EPD_DELAY_EVT\r\n");
        EpdtestRefresh();

        return (events ^ APP_EPD_DELAY_EVT);
    }
    
    if (events & APP_EPD_SLEEP_EVT) {
        LREPMaster("APP_EPD_SLEEP_EVT\r\n");
         EpdReset();
         EpdSleep();

        return (events ^ APP_EPD_SLEEP_EVT);
    }
    
    if (events & APP_BH1750_DELAY_EVT) {
        LREPMaster("APP_BH1750_DELAY_EVT\r\n");
        zclApp_bh1750ReadLumosity();
        
        return (events ^ APP_BH1750_DELAY_EVT);
    }
    
    if (events & APP_SAVE_ATTRS_EVT) {
        LREPMaster("APP_SAVE_ATTRS_EVT\r\n");
        zclApp_SaveAttributesToNV();
        
        return (events ^ APP_SAVE_ATTRS_EVT);
    }

    // Discard unknown events
    return 0;
}

static void zclApp_HandleKeys(byte portAndAction, byte keyCode) {
    LREP("zclApp_HandleKeys portAndAction=0x%X keyCode=0x%X\r\n", portAndAction, keyCode);
    zclFactoryResetter_HandleKeys(portAndAction, keyCode);
    zclCommissioning_HandleKeys(portAndAction, keyCode);
    if (portAndAction & HAL_KEY_PRESS) {
        LREPMaster("Key press\r\n");
    }

    bool contact = portAndAction & HAL_KEY_PRESS ? TRUE : FALSE;
    uint8 endPoint = 0;
    if (portAndAction & HAL_KEY_PORT0) {
        LREPMaster("Key press PORT0\r\n");
        
    } else if (portAndAction & HAL_KEY_PORT1) {     
        LREPMaster("Key press PORT1\r\n");
        if (!contact) {
          P2INP |= HAL_KEY_BIT6;  // pull down
        } else {
          P2INP &= ~HAL_KEY_BIT6; // pull up
        }
        if (power == 0){
          if (contact) {
            osal_start_timerEx(zclApp_TaskID, APP_MOTION_ON_EVT, 100);
            osal_stop_timerEx(zclApp_TaskID, APP_MOTION_OFF_EVT);
            osal_clear_event(zclApp_TaskID, APP_MOTION_OFF_EVT);
            
          }
        } else {
          if (power == 1){
            //end adaptive motion
            osal_start_timerEx(zclApp_TaskID, APP_MOTION_OFF_EVT, (uint32)zclApp_Config.PirUnoccupiedToOccupiedDelay * 1000); 
          }
          power = power - 1;
        }
        LREP("power=%d\r\n", power);
     } else if (portAndAction & HAL_KEY_PORT2) {
       LREPMaster("Key press PORT2\r\n");
       if (contact) {
          HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);
//          IEN2 &= ~HAL_KEY_BIT4; // disable port1 int
          osal_start_timerEx(zclApp_TaskID, APP_REPORT_EVT, 200);
       } else {
//          IEN2 |= HAL_KEY_BIT4; // enable port1 int
       }
     }
     LREP("contact=%d endpoint=%d\r\n", contact, endPoint);
     uint16 alarmStatus = 0;
     if (!contact) {
        alarmStatus |= BV(0);
     } 
}

static void zclApp_ReadSensors(void) {
    LREP("currentSensorsReadingPhase %d\r\n", currentSensorsReadingPhase);
    /**
     * FYI: split reading sensors into phases, so single call wouldn't block processor
     * for extensive ammount of time
     * */
  if (report == 1) {
    switch (currentSensorsReadingPhase++) {
    case 0:
//        HalLedSet(HAL_LED_1, HAL_LED_MODE_BLINK);    
      zclBattery_Report();      
        break;
    case 1:

          break;
    case 2:
      if (bmeDetect == 1){
          zclApp_ReadBME280Temperature();
      }
        break;
    case 3:
      if (bmeDetect == 1){
          zclApp_ReadBME280Pressure();
      }
        break;
    case 4:
      if (bmeDetect == 1){
          zclApp_ReadBME280Humidity();
      }
        break;
    case 5:
      if (bh1750Detect == 1){
        zclApp_bh1750StartLumosity();
      }      
        break;
//    case 6:
//        bdb_RepChangedAttrValue(zclApp_SecondEP.EndPoint, BINARY_INPUT, ATTRID_GEN_BINARY_INPUT_PRESENTVALUE);      
//        break;
    default:
        osal_stop_timerEx(zclApp_TaskID, APP_READ_SENSORS_EVT);
        osal_clear_event(zclApp_TaskID, APP_READ_SENSORS_EVT);
        currentSensorsReadingPhase = 0;
        break;
    }
  }

}

static void zclApp_bh1750StartLumosity(void) {
        bh1850_Write(BH1750_POWER_ON);        
        bh1850_Write(BH1750_mode);
   
        if (BH1750_mode == CONTINUOUS_LOW_RES_MODE || BH1750_mode == ONE_TIME_LOW_RES_MODE) {
          osal_start_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT, 30);
        } else {
          osal_start_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT, 180);
        }
}

static void zclApp_bh1750ReadLumosity(void) {
    zclApp_bh1750IlluminanceSensor_MeasuredValue = (uint16)(bh1850_Read());
    bh1850_PowerDown();
        
    uint16 illum = 0;
    if (temp_bh1750IlluminanceSensor_MeasuredValue > zclApp_bh1750IlluminanceSensor_MeasuredValue){
      illum = (temp_bh1750IlluminanceSensor_MeasuredValue - zclApp_bh1750IlluminanceSensor_MeasuredValue);
    } else {
      illum = (zclApp_bh1750IlluminanceSensor_MeasuredValue - temp_bh1750IlluminanceSensor_MeasuredValue);
    }
    if (illum > zclApp_Config.MsIlluminanceMinAbsoluteChange || report == 1){ // 10 lux
      temp_bh1750IlluminanceSensor_MeasuredValue = zclApp_bh1750IlluminanceSensor_MeasuredValue;
      bdb_RepChangedAttrValue(zclApp_FourthEP.EndPoint, ILLUMINANCE, ATTRID_MS_ILLUMINANCE_MEASURED_VALUE);
      EpdRefresh();
    }
    LREP("bh1750IlluminanceSensor_MeasuredValue value=%d\r\n", zclApp_bh1750IlluminanceSensor_MeasuredValue);
}

static void zclApp_ReadBME280Temperature(void) {
    uint8 chip = bme280_read8(BME280_REGISTER_CHIPID);
    LREP("BME280_REGISTER_CHIPID=%d\r\n", chip);;
    if (chip == 0x60) {
        bme280_takeForcedMeasurement();
        zclApp_Temperature_Sensor_MeasuredValue = (int16)(bme280_readTemperature() *100);
        LREP("Temperature=%d\r\n", zclApp_Temperature_Sensor_MeasuredValue);
        
        uint16 temp = 0;
        if (temp_Temperature_Sensor_MeasuredValue > zclApp_Temperature_Sensor_MeasuredValue){
          temp = (temp_Temperature_Sensor_MeasuredValue - zclApp_Temperature_Sensor_MeasuredValue);
        } else {
          temp = (zclApp_Temperature_Sensor_MeasuredValue - temp_Temperature_Sensor_MeasuredValue);
        }
        if (temp > zclApp_Config.MsTemperatureMinAbsoluteChange || report == 1){ //50 - 0.5 
          temp_Temperature_Sensor_MeasuredValue = zclApp_Temperature_Sensor_MeasuredValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
          EpdRefresh();
        }        
    } else {
        LREPMaster("NOT BME280\r\n");
    }
}

static void zclApp_ReadBME280Pressure(void) {
    uint8 chip = bme280_read8(BME280_REGISTER_CHIPID);
    LREP("BME280_REGISTER_CHIPID=%d\r\n", chip);;
    if (chip == 0x60) {
        bme280_takeForcedMeasurement();
        zclApp_PressureSensor_ScaledValue = (int16) (pow(10.0, (double) zclApp_PressureSensor_Scale) * (double) bme280_readPressure()* 100);

        zclApp_PressureSensor_MeasuredValue = (uint16)bme280_readPressure();
        LREP("Pressure=%d\r\n", zclApp_PressureSensor_MeasuredValue);
                
        uint16 press = 0;
        if (temp_PressureSensor_MeasuredValue > zclApp_PressureSensor_MeasuredValue){
          press = (temp_PressureSensor_MeasuredValue - zclApp_PressureSensor_MeasuredValue);
        } else {
          press = (zclApp_PressureSensor_MeasuredValue - temp_PressureSensor_MeasuredValue);
        }
        if (press > zclApp_Config.MsPressureMinAbsoluteChange || report == 1){ //1gPa
          temp_PressureSensor_MeasuredValue = zclApp_PressureSensor_MeasuredValue;
          temp_PressureSensor_ScaledValue = zclApp_PressureSensor_ScaledValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, PRESSURE, ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE);
          EpdRefresh();
        }
    } else {
        LREPMaster("NOT BME280\r\n");
    }
}

static void zclApp_ReadBME280Humidity(void) {
    uint8 chip = bme280_read8(BME280_REGISTER_CHIPID);
    LREP("BME280_REGISTER_CHIPID=%d\r\n", chip);;
    if (chip == 0x60) {
        bme280_takeForcedMeasurement();
        zclApp_HumiditySensor_MeasuredValue = (uint16)(bme280_readHumidity() * 100);
        LREP("Humidity=%d\r\n", zclApp_HumiditySensor_MeasuredValue);
                
        uint16 humid = 0;
        if (temp_HumiditySensor_MeasuredValue > zclApp_HumiditySensor_MeasuredValue){
          humid = (temp_HumiditySensor_MeasuredValue - zclApp_HumiditySensor_MeasuredValue);
        } else {
          humid = (zclApp_HumiditySensor_MeasuredValue - temp_HumiditySensor_MeasuredValue);
        }
        if (humid > zclApp_Config.MsHumidityMinAbsoluteChange || report == 1){ //10%
          temp_HumiditySensor_MeasuredValue = zclApp_HumiditySensor_MeasuredValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HUMIDITY, ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE);
          EpdRefresh();
        }
    } else {
        LREPMaster("NOT BME280\r\n");
    }
}

static void zclApp_Report(void) { osal_start_reload_timer(zclApp_TaskID, APP_READ_SENSORS_EVT, 100); }

static void zclApp_BasicResetCB(void) {
    LREPMaster("BasicResetCB\r\n");
    zclApp_ResetAttributesToDefaultValues();
    zclApp_SaveAttributesToNV();
}

static ZStatus_t zclApp_ReadWriteAuthCB(afAddrType_t *srcAddr, zclAttrRec_t *pAttr, uint8 oper) {
    LREPMaster("AUTH CB called\r\n");

    osal_start_timerEx(zclApp_TaskID, APP_SAVE_ATTRS_EVT, 2000);
    return ZSuccess;
}

static void zclApp_SaveAttributesToNV(void) {
    uint8 writeStatus = osal_nv_write(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
    LREP("Saving attributes to NV write=%d\r\n", writeStatus);
    time_now_s = ((zclApp_BatteryManu[12]-48)*10 + (zclApp_BatteryManu[13]-48))*60 + (zclApp_BatteryManu[15]-48)*10 + (zclApp_BatteryManu[16]-48);
    date_now =((zclApp_BatteryManu[4]-48)*10 + (zclApp_BatteryManu[5]-48))*30 + (zclApp_BatteryManu[1]-48)*10 + (zclApp_BatteryManu[2]-48);
    EpdRefresh();
    zclApp_bh1750setMTreg();
    zclApp_StopReloadTimer();
    zclApp_StartReloadTimer();
}

static void zclApp_StopReloadTimer(void) {
    osal_stop_timerEx(zclApp_TaskID, APP_REPORT_BATTERY_EVT);
    osal_clear_event(zclApp_TaskID, APP_REPORT_BATTERY_EVT);    
  
    if (bmeDetect == 1){
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT);
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_PRESSURE_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_PRESSURE_EVT);
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT);
    }
    if (bh1750Detect == 1){
      osal_stop_timerEx(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT);
      osal_clear_event(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT);
    }
}

static void zclApp_StartReloadTimer(void) {
    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_BATTERY_EVT, (uint32)zclApp_Config.CfgBatteryPeriod * 60000);
    
    if (bmeDetect == 1){
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT, (uint32)zclApp_Config.MsTemperaturePeriod * 1000);
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_PRESSURE_EVT, (uint32)zclApp_Config.MsPressurePeriod * 1000);
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT, (uint32)zclApp_Config.MsHumidityPeriod * 1000);
    }
    if (bh1750Detect == 1){
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT, (uint32)zclApp_Config.MsIlluminancePeriod * 1000);
    } 
}

static void zclApp_RestoreAttributesFromNV(void) {
    uint8 status = osal_nv_item_init(NW_APP_CONFIG, sizeof(application_config_t), NULL);
    LREP("Restoring attributes from NV  status=%d \r\n", status);
    if (status == NV_ITEM_UNINIT) {
        uint8 writeStatus = osal_nv_write(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
        LREP("NV was empty, writing %d\r\n", writeStatus);
    }
    if (status == ZSUCCESS) {
        LREPMaster("Reading from NV\r\n");
        osal_nv_read(NW_APP_CONFIG, 0, sizeof(application_config_t), &zclApp_Config);
    }
}

static void zclApp_bh1750setMTreg(void) {
    if (bh1750Detect == 1){
      uint8 MTreg = (uint8)zclApp_Config.MsIlluminanceLevelSensingSensitivity;
      bh1750_setMTreg(MTreg);
    }
}

static void EpdRefresh(void){
  osal_start_timerEx(zclApp_TaskID, APP_EPD_DELAY_EVT, 2000);
}

static void _delay_us(uint16 microSecs)
{
  while(microSecs--)
  {
    asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  }
}

static void _delay_ms(uint16 milliSecs)
{
  while(milliSecs--)
  {
    _delay_us(1000);
  }
}

void EpdtestRefresh(void)
{ 
  EpdReset(); //disable sleep EPD
  
  // clock init Firmware build date 20/08/2021 13:47
  if (time_now_s == 1440){
    time_now_s = 0;
    date_now = date_now +1;
  }
  char time_string[] = {'0', '0', ':', '0', '0', '\0'};
  time_string[0] = time_now_s / 60 / 10 + '0';
  time_string[1] = time_now_s / 60 % 10 + '0';
  time_string[3] = time_now_s % 60 / 10 + '0';
  time_string[4] = time_now_s % 60 % 10 + '0';
  
  char date_string[] = {'0', '0', '.', '0', '0', '.', '2', '1', '\0'};
  date_string[0] = date_now/10 % 3  + '0';
  date_string[1] = date_now % 10 + '0';
  date_string[3] = date_now/30/10 % 2 + '0';
  date_string[4] = date_now/30 % 10 + '0';

  PaintSetWidth(24);
  PaintSetHeight(85);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, time_string, &Font24, COLORED);
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 70, 10, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 170, 70, PaintGetWidth(), PaintGetHeight());
#endif
  
  PaintSetWidth(16);
  PaintSetHeight(90);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 50, 8, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 150, 68, PaintGetWidth(), PaintGetHeight());
#endif
  
  //percentage
  char perc_string[] = {'0', '0', '0', '%', '\0'};
  perc_string[0] = zclBattery_PercentageRemainig/2 / 100 % 10 + '0';
  perc_string[1] = zclBattery_PercentageRemainig/2 / 10 % 10 + '0';
  perc_string[2] = zclBattery_PercentageRemainig/2 % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(48);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, perc_string, &Font16, COLORED);
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 17, 36, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 120, 96, PaintGetWidth(), PaintGetHeight());
#endif
  
  // Occupancy
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);    
  PaintClear(UNCOLORED);
  if (zclApp_Occupied == 0) {
    PaintDrawStringAt(0, 0, "UnOccupied", &Font16, COLORED);
  } else {
    PaintDrawStringAt(0, 0, "Occupied  ", &Font16, COLORED);
  }
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 90, 118, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 90, 60, PaintGetWidth(), PaintGetHeight());
#endif
  
  //Illuminance
  char illum_string[] = {'0','0', '0', '0', '0', ' ', ' ','l', 'x','\0'};
  illum_string[0] = temp_bh1750IlluminanceSensor_MeasuredValue / 10000 % 10 + '0';
  illum_string[1] = temp_bh1750IlluminanceSensor_MeasuredValue / 1000 % 10 + '0';
  illum_string[2] = temp_bh1750IlluminanceSensor_MeasuredValue / 100 % 10 + '0';
  illum_string[3] = temp_bh1750IlluminanceSensor_MeasuredValue / 10 % 10 + '0';
  illum_string[4] = temp_bh1750IlluminanceSensor_MeasuredValue % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, illum_string, &Font16, COLORED);
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 65, 118, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 65, 60, PaintGetWidth(), PaintGetHeight());
#endif
  
  //temperature
  char temp_string[] = {'0', '0', '.', '0', '0', ' ', ' ','^', 'C','\0'};
  temp_string[0] = temp_Temperature_Sensor_MeasuredValue / 1000 % 10 + '0';
  temp_string[1] = temp_Temperature_Sensor_MeasuredValue / 100 % 10 + '0';
  temp_string[3] = temp_Temperature_Sensor_MeasuredValue / 10 % 10 + '0';
  temp_string[4] = temp_Temperature_Sensor_MeasuredValue % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, temp_string, &Font16, COLORED);
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 49, 118, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 49, 60, PaintGetWidth(), PaintGetHeight());
#endif
  
  //humidity
  char hum_string[] = {'0', '0', '.', '0', '0', ' ', ' ','%', ' ','\0'};
  hum_string[0] = temp_HumiditySensor_MeasuredValue / 1000 % 10 + '0';
  hum_string[1] = temp_HumiditySensor_MeasuredValue / 100 % 10 + '0';
  hum_string[3] = temp_HumiditySensor_MeasuredValue / 10 % 10 + '0';
  hum_string[4] = temp_HumiditySensor_MeasuredValue % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, hum_string, &Font16, COLORED);
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 33, 118, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 33, 60, PaintGetWidth(), PaintGetHeight());
#endif
  
  //pressure
  char pres_string[] = {'0', '0', '0', '0', '.', '0', ' ','h', 'P','\0'};
  pres_string[0] = temp_PressureSensor_MeasuredValue / 1000 % 10 + '0';
  pres_string[1] = temp_PressureSensor_MeasuredValue / 100 % 10 + '0';
  pres_string[2] = temp_PressureSensor_MeasuredValue / 10 % 10 + '0';
  pres_string[3] = temp_PressureSensor_MeasuredValue % 10 + '0';
  pres_string[5] = temp_PressureSensor_ScaledValue % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, pres_string, &Font16, COLORED); 
#if defined(EPD2IN9) || defined(EPD2IN9V2) || defined(EPD2IN13V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 17, 118, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  EpdSetFrameMemoryXY(PaintGetImage(), 17, 60, PaintGetWidth(), PaintGetHeight());
#endif

  EpdDisplayFramePartial();
//  EpdDisplayFrame();
//  osal_start_timerEx(zclApp_TaskID, APP_EPD_SLEEP_EVT, 2000);
  EpdSleep();
}

/****************************************************************************
****************************************************************************/
