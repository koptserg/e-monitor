
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
#include "zcl_hvac.h"

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
#ifdef EPD3IN7
#include "epd3in7.h"
#endif

#define HAL_LCD_BUSY BNAME(HAL_LCD_BUSY_PORT, HAL_LCD_BUSY_PIN)

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

bool EpdDetect = 1; 
uint8 fullupdate_hour = 0;
uint8 zclApp_color = 1;
uint8 zclApp_EpdUpDown = 0x00; 

#ifdef LQI_REQ
uint8 zclApp_lqi = 255;
uint8 zclApp_startIndex = 0;
#endif
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

static void zclApp_LocalTime(void);
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

#ifdef LQI_REQ
void zclApp_ProcessZDOMsgs(zdoIncomingMsg_t *InMsg);
void zclApp_RequestLqi(void);
#endif

void zclApp_SetTimeDate(void);
void zclApp_EpdUpdateClock(void);

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
#ifdef LQI_REQ
    // Register the callback Mgmt_Lqi_rsp
    ZDO_RegisterForZDOMsg(zclApp_TaskID, Mgmt_Lqi_rsp);
#endif
    
  // check epd

  EpdReset();  
  uint8 error_time = 25; // over 2.5 sec return
  while(HAL_LCD_BUSY == 1) {      //LOW: idle, HIGH: busy
    _delay_ms(100);
    error_time = error_time - 1;
    if (error_time == 0){    
      EpdDetect = 0;
      break;
    }
  }   
    
  if (EpdDetect == 1) { 
    if (zclApp_Config.HvacUiDisplayMode & 0x01){
      zclApp_color = 0xFF;
    } else {
      zclApp_color = 0x00;
    }
    // epd full screen
    EpdInitFull();
    EpdSetFrameMemoryBase(IMAGE_DATA, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdDisplayFrame();
    _delay_ms(2000);

    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFrame();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFrame();
  
    // epd partial screen
    EpdInitPartial();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFramePartial();
    EpdClearFrameMemory(zclApp_color);
    EpdDisplayFramePartial();

    zclApp_SetTimeDate();  
    EpdRefresh();

  }
    zclApp_StartReloadTimer();
    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_CLOCK_EVT, 60000);
}

#ifdef LQI_REQ
void zclApp_ProcessZDOMsgs(zdoIncomingMsg_t *InMsg){
  
  switch (InMsg->clusterID){
    case Mgmt_Lqi_rsp:
//      LREP("InMsg->clusterID=0x%X\r\n", InMsg->clusterID);      
    {
      ZDO_MgmtLqiRsp_t *LqRsp;
      uint8 num;
      uint8 find = 0;
      uint8 i;
      uint8 EndDevice_Lqi;

      LqRsp = ZDO_ParseMgmtLqiRsp(InMsg);

      num = LqRsp->neighborLqiCount;
      LREP("neighborLqiCount=%d\r\n",LqRsp->neighborLqiCount);
      LREP("startIndex=%d\r\n",LqRsp->startIndex);
      if (num != 0) {
        for(i= 0; i < num && find == 0; i++)
        {
          if (LqRsp->list[i].nwkAddr == _NIB.nwkDevAddress) {
            EndDevice_Lqi = LqRsp->list[i].lqi;
            zclApp_lqi = EndDevice_Lqi;
            zclApp_startIndex = LqRsp->startIndex;
            find = 1;
          } else {
            zclApp_startIndex = zclApp_startIndex + 1;
          }
          LREP("nwkAddr=0x%X\r\n", LqRsp->list[i].nwkAddr);
          LREP("lqi=%d\r\n", LqRsp->list[i].lqi);
        }
      } else {
        zclApp_startIndex = 0;
        zclApp_lqi = 255;
      }
      osal_mem_free(LqRsp);      
      
      EpdRefresh();
    }

    break;
  }
  if (InMsg->asdu) {
        osal_mem_free(InMsg->asdu);
  }
}
#endif

#ifdef LQI_REQ
void zclApp_RequestLqi(void){
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
        zAddrType_t destAddr;
        uint8 startIndex;
        /* Dev address */
        destAddr.addrMode = Addr16Bit;
//        destAddr.addr.shortAddr = 0; // Coordinator always address 0
        destAddr.addr.shortAddr = _NIB.nwkCoordAddress; // Parent
        startIndex = zclApp_startIndex;
        ZDP_MgmtLqiReq(&destAddr,startIndex, 0);
//        ZDP_MgmtLqiReq(&destAddr,startIndex, AF_EN_SECURITY);
        zclApp_lqi = 255;
        LREP("REQ_sI=%d\r\n", zclApp_startIndex);
  }
}
#endif

void zclApp_EpdUpdateClock(void) {
        if (EpdDetect == 1) {
          if (zclApp_Config.HvacUiDisplayMode & 0x01){
              zclApp_color = 0xFF;
          } else {
              zclApp_color = 0x00;
          }
            // epd full screen
            EpdInitFull();
            EpdClearFrameMemory(zclApp_color);
            EpdDisplayFrame();
            EpdClearFrameMemory(zclApp_color);
            EpdDisplayFrame();
            // epd partial screen
            EpdInitPartial();
            EpdClearFrameMemory(zclApp_color);
            EpdDisplayFramePartial();
            EpdClearFrameMemory(zclApp_color);
            EpdDisplayFramePartial();        
        }
}

uint16 zclApp_event_loop(uint8 task_id, uint16 events) {
    afIncomingMSGPacket_t *MSGpkt;
    devStates_t zclApp_NwkState; //---
    
    (void)task_id; // Intentionally unreferenced parameter
    if (events & SYS_EVENT_MSG) {
        while ((MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(zclApp_TaskID))) {          
            switch (MSGpkt->hdr.event) {
#ifdef LQI_REQ
            case ZDO_CB_MSG: // ZDO incoming message callback
                zclApp_ProcessZDOMsgs((zdoIncomingMsg_t *)MSGpkt);
                LREP("MSGpkt->hdr.event=0x%X\r\n", MSGpkt->hdr.event);

              break;
#endif
            case KEY_CHANGE:
                zclApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
                
                break;
            case ZDO_STATE_CHANGE: //devStates_t ZDApp.h             
                zclApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                LREP("NwkState=%d\r\n", zclApp_NwkState);
                if (zclApp_NwkState == DEV_END_DEVICE) {
#ifdef LQI_REQ                  
                  zclApp_startIndex = 0;
                  zclApp_lqi = 255;
                  EpdRefresh();
#endif
                  IEN2 |= HAL_KEY_BIT4; // enable port1 int
                  P1DIR |=  BV(0); // P1_0 output
                  P1 |=  BV(0);   // power on DD
                } else {
                  IEN2 &= ~HAL_KEY_BIT4; // disable port1 int
                  P1 &= ~BV(0);   // power off DD //-- 
                  P1DIR &= ~BV(0); // P1_0 input
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
      if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {  
        osalTimeUpdate();
        zclApp_GenTime_TimeUTC = osal_getClock();                
#ifdef LQI_REQ        
        zclApp_RequestLqi();
#endif 
        fullupdate_hour = fullupdate_hour +1;
        if (fullupdate_hour == 30){ // over 5 min clear
          zclApp_EpdUpdateClock();
          fullupdate_hour = 0;
        }
//        zclApp_LocalTime(); //report local time
        EpdRefresh();
      }
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
        LREPMaster("MOTION_START_DELAY\r\n");
        //report
        zclApp_Occupied = 1;
        zclApp_Occupied_OnOff = 1;
        zclGeneral_SendOnOff_CmdOn(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
//        EpdRefresh();
        
        return (events ^ APP_MOTION_ON_EVT);
    }
    
    if (events & APP_MOTION_OFF_EVT) {
        LREPMaster("APP_MOTION_OFF_EVT\r\n");
        //report    
        zclApp_Occupied = 0;
        zclApp_Occupied_OnOff = 0;
        zclGeneral_SendOnOff_CmdOff(zclApp_ThirdEP.EndPoint, &inderect_DstAddr, TRUE, bdb_getZCLFrameCounter());
        bdb_RepChangedAttrValue(zclApp_ThirdEP.EndPoint, OCCUPANCY, ATTRID_MS_OCCUPANCY_SENSING_CONFIG_OCCUPANCY);
//        EpdRefresh();
        
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
  zclApp_StopReloadTimer();
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
        osal_stop_timerEx(zclApp_TaskID, APP_BH1750_DELAY_EVT);
        osal_clear_event(zclApp_TaskID, APP_BH1750_DELAY_EVT);
        zclApp_bh1750StartLumosity();
      }      
        break;
     case 6:
      if (EpdDetect == 1){
        bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HVAC_UI_CONFIG, ATTRID_HVAC_THERMOSTAT_UI_CONFIG_DISPLAY_MODE);
      }
        break;
     case 7:
       zclApp_LocalTime();
        break;
#ifdef LQI_REQ
    case 8:
        zclApp_RequestLqi();      
        break;
#endif
    default:
        osal_stop_timerEx(zclApp_TaskID, APP_READ_SENSORS_EVT);
        osal_clear_event(zclApp_TaskID, APP_READ_SENSORS_EVT);
        currentSensorsReadingPhase = 0;
        break;
    }
  }
  zclApp_StartReloadTimer();
}

static void zclApp_LocalTime(void) {
  bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, GEN_TIME, ATTRID_TIME_LOCAL_TIME);
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
      if (temp_bh1750IlluminanceSensor_MeasuredValue > zclApp_bh1750IlluminanceSensor_MeasuredValue){
        zclApp_EpdUpDown &= ~BV(0); // down
      } else {
        zclApp_EpdUpDown |=  BV(0); // up
      }
      temp_bh1750IlluminanceSensor_MeasuredValue = zclApp_bh1750IlluminanceSensor_MeasuredValue;
      bdb_RepChangedAttrValue(zclApp_FourthEP.EndPoint, ILLUMINANCE, ATTRID_MS_ILLUMINANCE_MEASURED_VALUE);
      EpdRefresh();
    }
//    LREP("bh1750IlluminanceSensor_MeasuredValue value=%d\r\n", zclApp_bh1750IlluminanceSensor_MeasuredValue);
}

static void zclApp_ReadBME280Temperature(void) {
        bme280_takeForcedMeasurement();
        zclApp_Temperature_Sensor_MeasuredValue = (int16)(bme280_readTemperature() *100);
//        LREP("Temperature=%d\r\n", zclApp_Temperature_Sensor_MeasuredValue);
        
        uint16 temp = 0;
        if (temp_Temperature_Sensor_MeasuredValue > zclApp_Temperature_Sensor_MeasuredValue){
          temp = (temp_Temperature_Sensor_MeasuredValue - zclApp_Temperature_Sensor_MeasuredValue);
        } else {
          temp = (zclApp_Temperature_Sensor_MeasuredValue - temp_Temperature_Sensor_MeasuredValue);
        }
        if (temp > zclApp_Config.MsTemperatureMinAbsoluteChange || report == 1){ //50 - 0.5 
          if (temp_Temperature_Sensor_MeasuredValue > zclApp_Temperature_Sensor_MeasuredValue){
            zclApp_EpdUpDown &= ~BV(1); // down
          } else {
            zclApp_EpdUpDown |=  BV(1); // up
          }
          temp_Temperature_Sensor_MeasuredValue = zclApp_Temperature_Sensor_MeasuredValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, TEMP, ATTRID_MS_TEMPERATURE_MEASURED_VALUE);
          EpdRefresh();
        }        
}

static void zclApp_ReadBME280Pressure(void) {
        bme280_takeForcedMeasurement();
        zclApp_PressureSensor_ScaledValue = (int16) (pow(10.0, (double) zclApp_PressureSensor_Scale) * (double) bme280_readPressure()* 100);

        zclApp_PressureSensor_MeasuredValue = (uint16)bme280_readPressure();
//        LREP("Pressure=%d\r\n", zclApp_PressureSensor_MeasuredValue);
                
        uint16 press = 0;
        if (temp_PressureSensor_MeasuredValue > zclApp_PressureSensor_MeasuredValue){
          press = (temp_PressureSensor_MeasuredValue - zclApp_PressureSensor_MeasuredValue);
        } else {
          press = (zclApp_PressureSensor_MeasuredValue - temp_PressureSensor_MeasuredValue);
        }
        if (press > zclApp_Config.MsPressureMinAbsoluteChange || report == 1){ //1gPa
          if (temp_PressureSensor_MeasuredValue > zclApp_PressureSensor_MeasuredValue){
            zclApp_EpdUpDown &= ~BV(3); // down
          } else {
            zclApp_EpdUpDown |=  BV(3); // up
          }
          temp_PressureSensor_MeasuredValue = zclApp_PressureSensor_MeasuredValue;
          temp_PressureSensor_ScaledValue = zclApp_PressureSensor_ScaledValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, PRESSURE, ATTRID_MS_PRESSURE_MEASUREMENT_MEASURED_VALUE);
          EpdRefresh();
        }
}

static void zclApp_ReadBME280Humidity(void) {
        bme280_takeForcedMeasurement();
        zclApp_HumiditySensor_MeasuredValue = (uint16)(bme280_readHumidity() * 100);
//        LREP("Humidity=%d\r\n", zclApp_HumiditySensor_MeasuredValue);
                
        uint16 humid = 0;
        if (temp_HumiditySensor_MeasuredValue > zclApp_HumiditySensor_MeasuredValue){
          humid = (temp_HumiditySensor_MeasuredValue - zclApp_HumiditySensor_MeasuredValue);
        } else {
          humid = (zclApp_HumiditySensor_MeasuredValue - temp_HumiditySensor_MeasuredValue);
        }
        if (humid > zclApp_Config.MsHumidityMinAbsoluteChange || report == 1){ //10%
          if (temp_HumiditySensor_MeasuredValue > zclApp_HumiditySensor_MeasuredValue){
            zclApp_EpdUpDown &= ~BV(2); // down
          } else {
            zclApp_EpdUpDown |=  BV(2); // up
          }
          temp_HumiditySensor_MeasuredValue = zclApp_HumiditySensor_MeasuredValue;
          bdb_RepChangedAttrValue(zclApp_FirstEP.EndPoint, HUMIDITY, ATTRID_MS_RELATIVE_HUMIDITY_MEASURED_VALUE);
          EpdRefresh();
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
    osal_setClock(zclApp_GenTime_TimeUTC);
    zclApp_EpdUpdateClock();    
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
  if (zclApp_Config.CfgBatteryPeriod != 0) {
    osal_start_reload_timer(zclApp_TaskID, APP_REPORT_BATTERY_EVT, (uint32)zclApp_Config.CfgBatteryPeriod * 60000);
  }  
  if (bmeDetect == 1){
    if (zclApp_Config.MsTemperaturePeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_TEMPERATURE_EVT, (uint32)zclApp_Config.MsTemperaturePeriod * 1000);
    }
    if (zclApp_Config.MsPressurePeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_PRESSURE_EVT, (uint32)zclApp_Config.MsPressurePeriod * 1000);
    }
    if (zclApp_Config.MsHumidityPeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_HUMIDITY_EVT, (uint32)zclApp_Config.MsHumidityPeriod * 1000);
    }
  }
  if (bh1750Detect == 1){
    if (zclApp_Config.MsIlluminancePeriod != 0) {
      osal_start_reload_timer(zclApp_TaskID, APP_REPORT_ILLUMINANCE_EVT, (uint32)zclApp_Config.MsIlluminancePeriod * 1000);
    }
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
  if (EpdDetect == 1) {
    if(zclApp_Occupied == 1 || bdbAttributes.bdbNodeIsOnANetwork == 0) {
      osal_start_timerEx(zclApp_TaskID, APP_EPD_DELAY_EVT, 2000);
    }
  }
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
  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  
  //status network
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(16);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_90);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
    PaintClear(UNCOLORED);
    if ( bdbAttributes.bdbNodeIsOnANetwork ){
      PaintDrawImage(IMAGE_ONNETWORK, 0, 0, 16, 16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());
    } else {
      PaintDrawImage(IMAGE_OFFNETWORK, 0, 0, 16, 16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());    
    }
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
  } else { // landscape
    if ( bdbAttributes.bdbNodeIsOnANetwork ){
      EpdSetFrameMemoryImageXY(IMAGE_ONNETWORK, 184, 0, 16, 16, zclApp_Config.HvacUiDisplayMode  & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_OFFNETWORK, 184, 0, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(16);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_90);
  PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
  PaintClear(UNCOLORED);
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
    PaintDrawImage(IMAGE_ONNETWORK, 0, 0, 16, 16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());
  } else {
    PaintDrawImage(IMAGE_OFFNETWORK, 0, 0, 16, 16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());    
  }
  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(16);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_90);
  PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
  PaintClear(UNCOLORED);
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
    PaintDrawImage(IMAGE_ONNETWORK, 0, 0, 16, 16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());
  } else {
    PaintDrawImage(IMAGE_OFFNETWORK, 0, 0, 16, 16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());    
  }
  PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);

#endif
#if defined(EPD2IN9)
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
    EpdSetFrameMemoryImageXY(IMAGE_ONNETWORK, 112, 0, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_OFFNETWORK, 112, 0, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD3IN7)
  if ( bdbAttributes.bdbNodeIsOnANetwork ){
    EpdSetFrameMemoryImageXY(IMAGE_ONNETWORK, 264, 1, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_OFFNETWORK, 264, 1, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
  PaintSetWidth(8);
  PaintSetHeight(480);
  PaintSetRotate(ROTATE_90); 
  PaintClear(UNCOLORED);
//  PaintDrawRectangle(0, 4, 480, 4, COLORED);
  PaintDrawHorizontalLine(0, 4, 480, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 248, 0, PaintGetWidth(), PaintGetHeight());
  EpdSetFrameMemoryXY(PaintGetImage(), 4, 0, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(240);
  PaintSetHeight(8);
  PaintSetRotate(ROTATE_90); 
  PaintClear(UNCOLORED);  
  PaintDrawVerticalLine(0, 0, 240, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 8, 216, PaintGetWidth(), PaintGetHeight());
#endif

#ifdef LQI_REQ  
  // LQI 
  char lqi_string[] = {'0', '0', '0', '\0'};
  lqi_string[0] = zclApp_lqi / 100 % 10 + '0';
  lqi_string[1] = zclApp_lqi / 10 % 10 + '0';
  lqi_string[2] = zclApp_lqi % 10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(36);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, lqi_string, &Font16, COLORED); 
#if defined(EPD2IN9)
  EpdSetFrameMemoryXY(PaintGetImage(), 110, 18, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)

  if (zclApp_lqi != 255) { 
    PaintSetWidth(24);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_270);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
    PaintClear(UNCOLORED);
    if(zclApp_lqi > 40){
      PaintDrawImage(IMAGE_LQI_100, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 40 && zclApp_lqi > 30) {
      PaintDrawImage(IMAGE_LQI_80, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 30 && zclApp_lqi > 20) {
      PaintDrawImage(IMAGE_LQI_60, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 20 && zclApp_lqi > 10) {
      PaintDrawImage(IMAGE_LQI_40, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 10 && zclApp_lqi > 0) {
      PaintDrawImage(IMAGE_LQI_20, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi == 0) {
      PaintDrawImage(IMAGE_LQI_0, 0, 0, 16, 24, COLORED);
    }
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 0, PaintGetWidth(), PaintGetHeight());
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
    PaintSetWidth(36);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, lqi_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 32, 0, PaintGetWidth(), PaintGetHeight());
  }
 
#endif
#if defined(EPD3IN7)
  if (zclApp_lqi != 255) {
    EpdSetFrameMemoryXY(PaintGetImage(), 256, 64, PaintGetWidth(), PaintGetHeight());
    if(zclApp_lqi > 40){
      EpdSetFrameMemoryImageXY(IMAGE_LQI_100, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclApp_lqi <= 40 && zclApp_lqi > 30) {
      EpdSetFrameMemoryImageXY(IMAGE_LQI_80, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclApp_lqi <= 30 && zclApp_lqi > 20) {
      EpdSetFrameMemoryImageXY(IMAGE_LQI_60, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclApp_lqi <= 20 && zclApp_lqi > 10) {
      EpdSetFrameMemoryImageXY(IMAGE_LQI_40, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclApp_lqi <= 10 && zclApp_lqi > 0) {
      EpdSetFrameMemoryImageXY(IMAGE_LQI_20, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclApp_lqi == 0) {
      EpdSetFrameMemoryImageXY(IMAGE_LQI_0, 256, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
#if defined(EPD2IN13V2) 
  if (zclApp_lqi != 255) {    
    PaintSetWidth(24);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_270);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
    PaintClear(UNCOLORED);
    if(zclApp_lqi > 40){
      PaintDrawImage(IMAGE_LQI_100, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 40 && zclApp_lqi > 30) {
      PaintDrawImage(IMAGE_LQI_80, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 30 && zclApp_lqi > 20) {
      PaintDrawImage(IMAGE_LQI_60, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 20 && zclApp_lqi > 10) {
      PaintDrawImage(IMAGE_LQI_40, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi <= 10 && zclApp_lqi > 0) {
      PaintDrawImage(IMAGE_LQI_20, 0, 0, 16, 24, COLORED);
    } else if (zclApp_lqi == 0) {
      PaintDrawImage(IMAGE_LQI_0, 0, 0, 16, 24, COLORED);
    }
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 0, PaintGetWidth(), PaintGetHeight());
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
    
    PaintSetWidth(36);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, lqi_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 24, 0, PaintGetWidth(), PaintGetHeight());
  }
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    if (zclApp_lqi != 255) { 
      PaintSetWidth(24);
      PaintSetHeight(16);
      PaintSetRotate(ROTATE_270);
      PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
      PaintClear(UNCOLORED);
      if(zclApp_lqi > 40){
        PaintDrawImage(IMAGE_LQI_100, 0, 0, 16, 24, COLORED);
      } else if (zclApp_lqi <= 40 && zclApp_lqi > 30) {
        PaintDrawImage(IMAGE_LQI_80, 0, 0, 16, 24, COLORED);
      } else if (zclApp_lqi <= 30 && zclApp_lqi > 20) {
        PaintDrawImage(IMAGE_LQI_60, 0, 0, 16, 24, COLORED);
      } else if (zclApp_lqi <= 20 && zclApp_lqi > 10) {
        PaintDrawImage(IMAGE_LQI_40, 0, 0, 16, 24, COLORED);
      } else if (zclApp_lqi <= 10 && zclApp_lqi > 0) {
        PaintDrawImage(IMAGE_LQI_20, 0, 0, 16, 24, COLORED);
      } else if (zclApp_lqi == 0) {
        PaintDrawImage(IMAGE_LQI_0, 0, 0, 16, 24, COLORED);
      }
      EpdSetFrameMemoryXY(PaintGetImage(), 32, 0, PaintGetWidth(), PaintGetHeight());
      PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);
      PaintSetWidth(36);
      PaintSetHeight(16);
      PaintSetRotate(ROTATE_0);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 2, lqi_string, &Font16, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 64, 0, PaintGetWidth(), PaintGetHeight());
    }    
  } else { // lanscape
    if (zclApp_lqi != 255) {
      PaintSetWidth(16);
      PaintSetHeight(36);
      PaintSetRotate(ROTATE_90);
      PaintClear(UNCOLORED);
      PaintDrawStringAt(0, 0, lqi_string, &Font16, COLORED); 
      EpdSetFrameMemoryXY(PaintGetImage(), 184, 64, PaintGetWidth(), PaintGetHeight());
      if(zclApp_lqi > 40){
        EpdSetFrameMemoryImageXY(IMAGE_LQI_100, 184, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclApp_lqi <= 40 && zclApp_lqi > 30) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_80, 184, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclApp_lqi <= 30 && zclApp_lqi > 20) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_60, 184, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclApp_lqi <= 20 && zclApp_lqi > 10) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_40, 184, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclApp_lqi <= 10 && zclApp_lqi > 0) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_20, 184, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclApp_lqi == 0) {
        EpdSetFrameMemoryImageXY(IMAGE_LQI_0, 184, 34, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      }
    }
  }
#endif
#endif  //LQI_REQ
  
  // nwkDevAddress, nwkPanId, nwkLogicalChannel 
  char nwk_string[] = {'0', 'x','0', '0', '0', '0', ' ', '0', 'x','0', '0', '0', '0',' ', '0', '0','\0'};
  nwk_string[2] = _NIB.nwkDevAddress / 4096 %16 + '0';
  if ((_NIB.nwkDevAddress/4096 %16) > 9){
    nwk_string[2] = nwk_string[2]+7;
  }
  nwk_string[3] = _NIB.nwkDevAddress / 256 %16 + '0';
  if ((_NIB.nwkDevAddress/256 %16) > 9){
    nwk_string[3] = nwk_string[3]+7;
  }
  nwk_string[4] = _NIB.nwkDevAddress / 16 %16 + '0';
  if ((_NIB.nwkDevAddress/16 %16) > 9){
    nwk_string[4] = nwk_string[4]+7;
  }
  nwk_string[5] = _NIB.nwkDevAddress %16 + '0';
  if ((_NIB.nwkDevAddress %16) > 9){
    nwk_string[5] = nwk_string[5]+7;
  }
  // nwkCoordAddress
  nwk_string[9] = _NIB.nwkCoordAddress / 4096 %16 + '0';
  if ((_NIB.nwkCoordAddress/4096 %16) > 9){
    nwk_string[9] = nwk_string[9]+7;
  }
  nwk_string[10] = _NIB.nwkCoordAddress / 256 %16 + '0';
  if ((_NIB.nwkCoordAddress/256 %16) > 9){
    nwk_string[10] = nwk_string[10]+7;
  }
  nwk_string[11] = _NIB.nwkCoordAddress / 16 %16 + '0';
  if ((_NIB.nwkCoordAddress/16 %16) > 9){
    nwk_string[11] = nwk_string[11]+7;
  }
  nwk_string[12] = _NIB.nwkCoordAddress %16 + '0';
  if ((_NIB.nwkCoordAddress %16) > 9){
    nwk_string[12] = nwk_string[12]+7;
  }
  
  nwk_string[14] = _NIB.nwkLogicalChannel / 10 %10 + '0';
  nwk_string[15] = _NIB.nwkLogicalChannel %10 + '0';
  
  PaintSetWidth(16);
  PaintSetHeight(192);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, nwk_string, &Font16, COLORED); 
#if defined(EPD2IN9)
  EpdSetFrameMemoryXY(PaintGetImage(), 110, 64, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)
//  EpdSetFrameMemoryXY(PaintGetImage(), 110, 64, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD3IN7)
  EpdSetFrameMemoryXY(PaintGetImage(), 256, 216, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN13V2)
//  EpdSetFrameMemoryXY(PaintGetImage(), 106, 64, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
 /*
    PaintSetWidth(16);
    PaintSetHeight(176);
    PaintSetRotate(ROTATE_270);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, nwk_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 0, 24, PaintGetWidth(), PaintGetHeight());
*/
  } else { // landscape
/*
    PaintSetWidth(176);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, nwk_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 0, 0, PaintGetWidth(), PaintGetHeight());
*/
  }
#endif
  
  // clock init Firmware build date 20/08/2021 13:47
  // Update RTC and get new clock values
  osalTimeUpdate();
  UTCTimeStruct time;
  osal_ConvertUTCTime(&time, osal_getClock());

  char time_string[] = {'0', '0', ':', '0', '0', '\0'};
  time_string[0] = time.hour / 10 % 10 + '0';
  time_string[1] = time.hour % 10 + '0';
  time_string[3] = time.minutes / 10 % 10 + '0';
  time_string[4] = time.minutes % 10 + '0';
/*
  char time_string[] = {'0', '0', ':', '\0'};
  time_string[0] = time.hour / 10 % 10 + '0';
  time_string[1] = time.hour % 10 + '0';
  
  char time_string_1[] = {'0', '0', '\0'};
  time_string_1[0] = time.minutes / 10 % 10 + '0';
  time_string_1[1] = time.minutes % 10 + '0';
*/  
  // covert UTCTimeStruct date and month to display
  time.day = time.day + 1;
  time.month = time.month + 1;  
  char date_string[] = {'0', '0', '.', '0', '0', '.', '0', '0', '\0'};
  date_string[0] = time.day /10 % 10  + '0';
  date_string[1] = time.day % 10 + '0';
  date_string[3] = time.month / 10 % 10 + '0';
  date_string[4] = time.month % 10 + '0';
  date_string[6] = time.year / 10 % 10 + '0';
  date_string[7] = time.year % 10 + '0';

  uint8 day_week = (uint8)floor((float)(zclApp_GenTime_TimeUTC/86400)) % 7;

#if defined(EPD2IN9)  
  PaintSetWidth(48);
  PaintSetHeight(72);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 56, 10, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(48);
  PaintSetHeight(48);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, time_string_1, &Font48, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 56, 82, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD2IN9V2)
  PaintSetWidth(124);
  PaintSetHeight(48);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 4, time_string, &Font48, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 16, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(120);
  PaintSetHeight(48);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 16, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD3IN7)
  PaintSetWidth(48);
  PaintSetHeight(120);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 80, 40, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(120);
    PaintSetHeight(48);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 72, 16, PaintGetWidth(), PaintGetHeight());  
  } else { // landscape
    PaintSetWidth(48);
    PaintSetHeight(120);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, time_string, &Font48, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 136, 72, PaintGetWidth(), PaintGetHeight());
  }
#endif
  
  PaintSetWidth(16);
  PaintSetHeight(136);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
#if defined(EPD2IN9)
  EpdSetFrameMemoryXY(PaintGetImage(), 40, 24, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(92);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 4, date_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 64, PaintGetWidth(), PaintGetHeight());

#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(88);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 64, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD3IN7)
  EpdSetFrameMemoryXY(PaintGetImage(), 64, 57, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(88);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 88, 64, PaintGetWidth(), PaintGetHeight());
  } else { //landscape
    PaintSetWidth(16);
    PaintSetHeight(136);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 4, date_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 120, 88, PaintGetWidth(), PaintGetHeight());
  }
#endif

  char* day_string = "";
  if (day_week == 0) {
    day_string = "Thursday";
  } else if (day_week == 1) {
    day_string = " Friday ";
  } else if (day_week == 2) {
    day_string = "Saturday";
  } else if (day_week == 3) {
    day_string = " Sunday";
  } else if (day_week == 4) {
    day_string = " Monday";
  } else if (day_week == 5) {
    day_string = "Tuesday";
  } else if (day_week == 6) {
    day_string = "Wednesday";
  }
  PaintSetWidth(16);
  PaintSetHeight(136);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
#if defined(EPD3IN7)
  EpdSetFrameMemoryXY(PaintGetImage(), 48, 57, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(99);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 80, PaintGetWidth(), PaintGetHeight());
#endif 
#if defined(EPD2IN9V2)

  PaintSetWidth(104);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, day_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 80, PaintGetWidth(), PaintGetHeight());

#endif 
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(99);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 88, 80, PaintGetWidth(), PaintGetHeight());
  } else { //landscape
    PaintSetWidth(16);
    PaintSetHeight(136);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, day_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 104, 88, PaintGetWidth(), PaintGetHeight());
  }
#endif
  
  //percentage
  char perc_string[] = {' ', ' ', ' ', ' ', '\0'};
  if (zclBattery_PercentageRemainig != 0xFF) {
    perc_string[0] = zclBattery_PercentageRemainig/2 / 100 % 10 + '0';
    perc_string[1] = zclBattery_PercentageRemainig/2 / 10 % 10 + '0';
    perc_string[2] = zclBattery_PercentageRemainig/2 % 10 + '0';
    perc_string[3] = '%';
  }
  
  PaintSetWidth(16);
  PaintSetHeight(48);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, perc_string, &Font16, COLORED);
#if defined(EPD2IN9)
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 52, PaintGetWidth(), PaintGetHeight());
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(32);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 2, perc_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 96, 0, PaintGetWidth(), PaintGetHeight());

  if (zclBattery_PercentageRemainig != 0xFF) {
    PaintSetWidth(24);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_270);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
    PaintClear(UNCOLORED);
    if(zclBattery_PercentageRemainig/2 > 75){
      PaintDrawImage(IMAGE_BATTERY_100, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
      PaintDrawImage(IMAGE_BATTERY_75, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
      PaintDrawImage(IMAGE_BATTERY_50, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
      PaintDrawImage(IMAGE_BATTERY_25, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
      PaintDrawImage(IMAGE_BATTERY_0, 0, 0, 16, 24, COLORED);
    }
    EpdSetFrameMemoryXY(PaintGetImage(), 72, 0, PaintGetWidth(), PaintGetHeight());
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);  
  }

#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(32);
  PaintSetHeight(18);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 2, perc_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 88, 0, PaintGetWidth(), PaintGetHeight());

  if (zclBattery_PercentageRemainig != 0xFF) {
    PaintSetWidth(24);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_270);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
    PaintClear(UNCOLORED);
    if(zclBattery_PercentageRemainig/2 > 75){
      PaintDrawImage(IMAGE_BATTERY_100, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
      PaintDrawImage(IMAGE_BATTERY_75, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
      PaintDrawImage(IMAGE_BATTERY_50, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
      PaintDrawImage(IMAGE_BATTERY_25, 0, 0, 16, 24, COLORED);
    } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
      PaintDrawImage(IMAGE_BATTERY_0, 0, 0, 16, 24, COLORED);
    }
    EpdSetFrameMemoryXY(PaintGetImage(), 64, 0, PaintGetWidth(), PaintGetHeight());
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);  
  }
#endif   
#if defined(EPD3IN7)
  EpdSetFrameMemoryXY(PaintGetImage(), 256, 144, PaintGetWidth(), PaintGetHeight());
  if (zclBattery_PercentageRemainig != 0xFF) {   
    if(zclBattery_PercentageRemainig/2 > 75){
      EpdSetFrameMemoryImageXY(IMAGE_BATTERY_100, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
      EpdSetFrameMemoryImageXY(IMAGE_BATTERY_75, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
      EpdSetFrameMemoryImageXY(IMAGE_BATTERY_50, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
      EpdSetFrameMemoryImageXY(IMAGE_BATTERY_25, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
      EpdSetFrameMemoryImageXY(IMAGE_BATTERY_0, 256, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(48);
    PaintSetHeight(18);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 2, perc_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 144, 0, PaintGetWidth(), PaintGetHeight());
    if (zclBattery_PercentageRemainig != 0xFF) {
      PaintSetWidth(24);
      PaintSetHeight(16);
      PaintSetRotate(ROTATE_270);
      PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
      PaintClear(UNCOLORED);
      if(zclBattery_PercentageRemainig/2 > 75){
        PaintDrawImage(IMAGE_BATTERY_100, 0, 0, 16, 24, COLORED);
      } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
        PaintDrawImage(IMAGE_BATTERY_75, 0, 0, 16, 24, COLORED);
      } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
        PaintDrawImage(IMAGE_BATTERY_50, 0, 0, 16, 24, COLORED);
      } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
        PaintDrawImage(IMAGE_BATTERY_25, 0, 0, 16, 24, COLORED);
      } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
        PaintDrawImage(IMAGE_BATTERY_0, 0, 0, 16, 24, COLORED);
      }
      EpdSetFrameMemoryXY(PaintGetImage(), 112, 0, PaintGetWidth(), PaintGetHeight());
      PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);  
    }   
  } else { // landscape
    PaintSetWidth(16);
    PaintSetHeight(48);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, perc_string, &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 184, 144, PaintGetWidth(), PaintGetHeight());
    if (zclBattery_PercentageRemainig != 0xFF) {   
      if(zclBattery_PercentageRemainig/2 > 75){
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_100, 184, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 75 && zclBattery_PercentageRemainig/2 > 50) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_75, 184, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 50 && zclBattery_PercentageRemainig/2 > 25) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_50, 184, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 25 && zclBattery_PercentageRemainig/2 > 6) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_25, 184, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      } else if (zclBattery_PercentageRemainig/2 <= 6 && zclBattery_PercentageRemainig/2 > 0) {
        EpdSetFrameMemoryImageXY(IMAGE_BATTERY_0, 184, 116, 16, 25, zclApp_Config.HvacUiDisplayMode & 0x01);
      }
    }
  }
#endif
  
  // Occupancy
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);    
  PaintClear(UNCOLORED);
  char* occup_string = "";
  if (zclApp_Occupied == 0) {
    occup_string = "UnOccupied";
  } else {
    occup_string = " Occupied ";
  }
  PaintDrawStringAt(0, 0, occup_string, &Font16, COLORED);
#if defined(EPD2IN9)
  EpdSetFrameMemoryXY(PaintGetImage(), 88, 148, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)
/*  
  PaintSetWidth(110);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);    
  PaintClear(UNCOLORED);
//  PaintDrawStringAt(0, 0, occup_string, &Font16, COLORED);
//  EpdSetFrameMemoryXY(PaintGetImage(), 8, 280, PaintGetWidth(), PaintGetHeight());  
*/
#endif
#if defined(EPD3IN7)
  EpdSetFrameMemoryXY(PaintGetImage(), 128, 46, PaintGetWidth(), PaintGetHeight()); 
  if (zclApp_Occupied == 0) {
    EpdSetFrameMemoryImageXY(IMAGE_MOTION_NOT, 144, 74, 64, 64, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_MOTION,     144, 74, 64, 64, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(110);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);    
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, occup_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 8, 232, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait   
    PaintSetWidth(64);
    PaintSetHeight(64);
    PaintSetRotate(ROTATE_270);
    PaintSetInvert(!(zclApp_Config.HvacUiDisplayMode & 0x01));
    PaintClear(UNCOLORED);    
    if (zclApp_Occupied == 0) {
      PaintDrawImage(IMAGE_MOTION_NOT, 0, 0, 64, 64, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 8, 24, PaintGetWidth(), PaintGetHeight());     
    } else {
      PaintDrawImage(IMAGE_MOTION, 0, 0, 64, 64, COLORED);
      EpdSetFrameMemoryXY(PaintGetImage(), 8, 24, PaintGetWidth(), PaintGetHeight()); 
    }
    PaintSetInvert(zclApp_Config.HvacUiDisplayMode & 0x01);    
  } else { // landscape
    if (zclApp_Occupied == 0) {
      EpdSetFrameMemoryImageXY(IMAGE_MOTION_NOT, 112, 8, 64, 64, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_MOTION,     112, 8, 64, 64, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
  
  //Illuminance
  char illum_string[] = {'0','0', '0', '0', '0', '\0'};
  illum_string[0] = temp_bh1750IlluminanceSensor_MeasuredValue / 10000 % 10 + '0';
  illum_string[1] = temp_bh1750IlluminanceSensor_MeasuredValue / 1000 % 10 + '0';
  illum_string[2] = temp_bh1750IlluminanceSensor_MeasuredValue / 100 % 10 + '0';
  illum_string[3] = temp_bh1750IlluminanceSensor_MeasuredValue / 10 % 10 + '0';
  illum_string[4] = temp_bh1750IlluminanceSensor_MeasuredValue % 10 + '0';
  
#if defined(EPD2IN9)
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, illum_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 64, 148, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(84);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, illum_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 104, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Lx", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 104, 120, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x01){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 118, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 118, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
  PaintSetWidth(126);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, "Illuminance", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 134, PaintGetWidth(), PaintGetHeight());

#endif
#if defined(EPD3IN7)
  PaintSetWidth(32);
  PaintSetHeight(80);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 200, 300, PaintGetWidth(), PaintGetHeight());

  PaintSetWidth(16);
  PaintSetHeight(33);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 200, 388, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(16);
  PaintSetHeight(121);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Illuminance", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 184, 300, PaintGetWidth(), PaintGetHeight());
  EpdSetFrameMemoryImageXY(IMAGE_ILLUMINANCE, 184, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
  if (zclApp_EpdUpDown & 0x01){
    EpdSetFrameMemoryImageXY(IMAGE_UP, 216, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 184, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_DOWN, 184, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 216, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(80);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 104, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Lx", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 96, 104, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x01){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 120, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 120, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(80);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 120, 104, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(33);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 120, 136, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x01){
      EpdSetFrameMemoryImageXY(IMAGE_LEFT, 104, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 104, 120, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 104, 120, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 104, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    }    
  } else { //landscape
    PaintSetWidth(32);
    PaintSetHeight(80);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, illum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 64, 120, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "Lux", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 48, 120, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x01){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 80, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 64, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 64, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 80, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
  
  //temperature
  char temp_string[] = {'0', '0', '.', '0', '0', '\0'};
  temp_string[0] = temp_Temperature_Sensor_MeasuredValue / 1000 % 10 + '0';
  temp_string[1] = temp_Temperature_Sensor_MeasuredValue / 100 % 10 + '0';
  temp_string[3] = temp_Temperature_Sensor_MeasuredValue / 10 % 10 + '0';
  temp_string[4] = temp_Temperature_Sensor_MeasuredValue % 10 + '0';
  
#if defined(EPD2IN9)
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, temp_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 48, 148, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(84);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, temp_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 152, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 104, 168, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x02){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 166, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 166, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } 
  PaintSetWidth(126);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, "Temperature", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 182, PaintGetWidth(), PaintGetHeight());

#endif
#if defined(EPD3IN7)
  PaintSetWidth(32);
  PaintSetHeight(80);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 144, 300, PaintGetWidth(), PaintGetHeight());

  PaintSetWidth(16);
  PaintSetHeight(33);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 144, 388, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(16);
  PaintSetHeight(121);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Temperature", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 128, 300, PaintGetWidth(), PaintGetHeight());
  EpdSetFrameMemoryImageXY(IMAGE_TEMPERATURE, 128, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
  if (zclApp_EpdUpDown & 0x02){
    EpdSetFrameMemoryImageXY(IMAGE_UP, 160, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 128, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_DOWN, 128, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 160, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(80);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 136, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 96, 136, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x02){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 136, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 136, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }  
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(80);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 104, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(22);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 136, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x02){
      EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 120, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 120, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 104, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    }    
  } else { //landscape
    PaintSetWidth(32);
    PaintSetHeight(80);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, temp_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 64, 16, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(22);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "^C", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 48, 16, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x02){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 80, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 64, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 64, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 80, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
  
  //humidity
  char hum_string[] = {'0', '0', '.', '0', '0', '\0'};
  hum_string[0] = temp_HumiditySensor_MeasuredValue / 1000 % 10 + '0';
  hum_string[1] = temp_HumiditySensor_MeasuredValue / 100 % 10 + '0';
  hum_string[3] = temp_HumiditySensor_MeasuredValue / 10 % 10 + '0';
  hum_string[4] = temp_HumiditySensor_MeasuredValue % 10 + '0';
  
#if defined(EPD2IN9)
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, hum_string, &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 32, 148, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(84);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, hum_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 200, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "%H", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 104, 216, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x04){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 200, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 214, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 214, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 200, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
  PaintSetWidth(92);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, "Humidity", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 230, PaintGetWidth(), PaintGetHeight());

#endif
#if defined(EPD3IN7)
  PaintSetWidth(32);
  PaintSetHeight(80);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 88, 300, PaintGetWidth(), PaintGetHeight());

  PaintSetWidth(16);
  PaintSetHeight(33);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 88, 388, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(16);
  PaintSetHeight(121);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Humidity", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 72, 300, PaintGetWidth(), PaintGetHeight());
  EpdSetFrameMemoryImageXY(IMAGE_HUMIDITY, 72, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
  if (zclApp_EpdUpDown & 0x04){
    EpdSetFrameMemoryImageXY(IMAGE_UP, 104, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 72, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_DOWN, 72, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 104, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(80);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 168, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "%H", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 96, 168, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x04){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 168, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 184, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 184, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 168, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }  
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(80);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 152, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(33);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 184, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x04){
      EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 168, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 168, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  } else { // landscape
    PaintSetWidth(32);
    PaintSetHeight(80);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, hum_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 16, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "%Ha", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 1, 16, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x04){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 32, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 16, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 16, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 32, 4, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif
  
  //pressure
  char pres_string[] = {'0', '0', '0', '0', '.', '0', '\0'};
  pres_string[0] = temp_PressureSensor_MeasuredValue / 1000 % 10 + '0';
  pres_string[1] = temp_PressureSensor_MeasuredValue / 100 % 10 + '0';
  pres_string[2] = temp_PressureSensor_MeasuredValue / 10 % 10 + '0';
  pres_string[3] = temp_PressureSensor_MeasuredValue % 10 + '0';
  pres_string[5] = temp_PressureSensor_ScaledValue % 10 + '0';
  
#if defined(EPD2IN9)
  PaintSetWidth(16);
  PaintSetHeight(110);
  PaintSetRotate(ROTATE_90);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, pres_string, &Font16, COLORED); 
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 148, PaintGetWidth(), PaintGetHeight()); 
#endif
#if defined(EPD2IN9V2)

  PaintSetWidth(80);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, pres_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 248, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "hP", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 104, 264, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x08){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 248, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 262, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 262, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 248, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
  PaintSetWidth(92);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(4, 0, "Pressure", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 0, 278, PaintGetWidth(), PaintGetHeight());

#endif
#if defined(EPD3IN7)
  PaintSetWidth(32);
  PaintSetHeight(96);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 32, 300, PaintGetWidth(), PaintGetHeight());

  PaintSetWidth(16);
  PaintSetHeight(33);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 32, 404, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(16);
  PaintSetHeight(121);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "Pressure", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 300, PaintGetWidth(), PaintGetHeight());
  EpdSetFrameMemoryImageXY(IMAGE_PRESSURE, 16, 240, 48, 48, zclApp_Config.HvacUiDisplayMode & 0x01);
  if (zclApp_EpdUpDown & 0x08){
    EpdSetFrameMemoryImageXY(IMAGE_UP, 48, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 16, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_DOWN, 16, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 48, 284, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
  }
#endif
#if defined(EPD2IN13V2)
  PaintSetWidth(80);
  PaintSetHeight(32);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 16, 200, PaintGetWidth(), PaintGetHeight());
  PaintSetWidth(22);
  PaintSetHeight(16);
  PaintSetRotate(ROTATE_0);
  PaintClear(UNCOLORED);
  PaintDrawStringAt(0, 0, "hP", &Font16, COLORED);
  EpdSetFrameMemoryXY(PaintGetImage(), 96, 200, PaintGetWidth(), PaintGetHeight());
  if (zclApp_EpdUpDown & 0x08){
    EpdSetFrameMemoryImageXY(IMAGE_LEFT, 0, 200, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); // up
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 216, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  } else {
    EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 0, 216, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01); //down
    EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 0, 200, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
  }  
#endif
#if defined(EPD1IN54V2)
  if (zclApp_Config.HvacUiDisplayMode & 0x02){ // portrait
    PaintSetWidth(64);
    PaintSetHeight(32);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 120, 152, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(33);
    PaintSetHeight(16);
    PaintSetRotate(ROTATE_0);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 120, 184, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x08){
      EpdSetFrameMemoryImageXY(IMAGE_LEFT, 104, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 104, 168, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_RIGHT, 104, 168, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR16, 104, 152, 16, 16, zclApp_Config.HvacUiDisplayMode & 0x01);
    }    
  } else { //landscape
    PaintSetWidth(32);
    PaintSetHeight(64);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, pres_string, &Font32, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 16, 120, PaintGetWidth(), PaintGetHeight());
    PaintSetWidth(16);
    PaintSetHeight(33);
    PaintSetRotate(ROTATE_90);
    PaintClear(UNCOLORED);
    PaintDrawStringAt(0, 0, "hPa", &Font16, COLORED);
    EpdSetFrameMemoryXY(PaintGetImage(), 1, 120, PaintGetWidth(), PaintGetHeight());
    if (zclApp_EpdUpDown & 0x08){
      EpdSetFrameMemoryImageXY(IMAGE_UP, 32, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 16, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    } else {
      EpdSetFrameMemoryImageXY(IMAGE_DOWN, 16, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
      EpdSetFrameMemoryImageXY(IMAGE_CLEAR, 32, 108, 16, 12, zclApp_Config.HvacUiDisplayMode & 0x01);
    }
  }
#endif

  EpdDisplayFramePartial();

  EpdSleep();
}

void zclApp_SetTimeDate(void){
  // Set Time and Date
  UTCTimeStruct time;
   time.seconds = 00;
   time.minutes = (zclApp_DateCode[15]-48)*10 + (zclApp_DateCode[16]-48);
   time.hour = (zclApp_DateCode[12]-48)*10 + (zclApp_DateCode[13]-48);
   time.day = (zclApp_DateCode[1]-48)*10 + (zclApp_DateCode[2]-48) - 1;
   time.month = (zclApp_DateCode[4]-48)*10 + (zclApp_DateCode[5]-48) - 1;
   time.year = 2000+(zclApp_DateCode[9]-48)*10 + (zclApp_DateCode[10]-48);
  // Update OSAL time
  osal_setClock( osal_ConvertUTCSecs( &time ) );
  // Get time structure from OSAL
  osal_ConvertUTCTime( &time, osal_getClock() );
  osalTimeUpdate();
  zclApp_GenTime_TimeUTC = osal_getClock();
}

/****************************************************************************
****************************************************************************/
