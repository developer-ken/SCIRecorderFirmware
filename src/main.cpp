#include <ModbusMaster.h>
#include <Arduino.h>
#include <IniFile.h>
#include <SPIFFS.h>
#include <M1820.h>
#include <SPI.h>

#include <WiFi.h>
#include <WiFiAP.h>
#include <ESPNowW.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <uri/UriRegex.h>
#include <uri/UriBraces.h>

#include "soc/soc.h"
#include "soc/rtc_wdt.h"
#include "soc/rtc_cntl_reg.h"

#include <SD.h>
#include <FS.h>

#include "irda.h"
#include "pinout.h"
#include "structs.h"

#ifdef ENABLE_RTC
#include <Wire.h>
#include <RtcDS1307.h>
RtcDS1307<TwoWire> Rtc(Wire);
#endif

#define DEBUG Serial
#define RS485 Serial1
#define IRDA Serial2

#define SETSTATE(STATE, INDEX) (LED_STATE = (LED_STATE & ~(0b11 << INDEX)) | (STATE << INDEX))
uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define LED_OFF 0b00
#define LED_ON 0b01
#define LED_BLINK 0b10
#define LED_CYCLE 0b11

#define ERR 4
#define SAP 2
#define REC 0

#define LED_ERR_ON SETSTATE(LED_ON, ERR)
#define LED_ERR_OFF SETSTATE(LED_OFF, ERR)
#define LED_SAP_ON SETSTATE(LED_ON, SAP)
#define LED_SAP_OFF SETSTATE(LED_OFF, SAP)
#define LED_REC_ON SETSTATE(LED_ON, REC)
#define LED_REC_OFF SETSTATE(LED_OFF, REC)

//////
uint8_t FSD_ADDR = 1;
uint32_t FSD_BAUD = 9600;
uint32_t FSD_POLLINGINTERVAL = 500;

uint8_t IRDA_ADDR = 1;
uint32_t IRDA_BAUD = 4800;
bool IRDA_ALLOWCTRL = false;

String ESPNOW_KEY = "CFFF00CE";
bool ESPNOW_ALLOWCTRL = 0;

uint8_t START_TRIGGER = 1,
        STOP_TRIGGER = 1;
uint32_t SAMPLE_INTERVAL = 1000; // ms, 采样周期

uint8_t CRITICAL_TEMP = 80;
uint8_t MAX_TEMP = 85;
byte ONBOARD_SENSOR_ID[8];

String WIFI_AP_SSID = "KD510-IOT-EXT";
String WIFI_AP_PASSWORD = "12345678";
int WIFI_AP_CHANNEL = 1;
int WIFI_MAX_CLIENTS = 2;
bool WIFI_HIDDEN = false;
//////

M1820 OnboardTempSensor; // 1 onboard sensor
M1820 TempSensors[8];    // 8 external sensors
OneWire oneWire;         // OneWire bus
int SensorCount = 0;
uint8_t LED_STATE = 0;
TickType_t PollTime = 0;
TaskHandle_t TASK_LEDControl = NULL, TASK_Sample = NULL, TASK_Poll = NULL, TASK_ESPNow = NULL;
FSDState latest_fsd_status;
bool sampling = false;
float temp; // latest ob_sensor temperature
uint32_t modbus_sent = 0, modbus_error = 0;
WiFiAPClass WiFiAP;
ModbusMaster modbus;
File recordfile;
File rawfile;

bool DEBUG_MODE = false;      // Use debug mode, will override self-test result so can continue anyway.
bool DEBUG_NO_SDCARD = false; // In debug move, no SD card found, worlaround so can continue anyway.

const char *FirmwareCompiletimeHash = __TIME__ " " __DATE__;

WebServer webServer(80);
DNSServer dnsServer;

void RS485TXNOW();
void RS485RXNOW();

void LEDControl(void *);
void Sample(void *);
void Poll(void *);
void FSDStateBroadcaster(void *);
void IOHangup();
void LoadConfig();
bool isAllZero(byte *bytes, int len);
void hexStringToByteArray(const char *hexString, unsigned char *byteArray, size_t byteArraySize);
String bytesToHexString(const byte *bytes, const int length);
String ContentType(String filename);
bool bytesEquals(byte *bytes1, byte *bytes2, int len);

void HandleRootWeb();
void GetStatus();
void ListFiles();
void SetTime();
void DownloadFile();
void RedirectHome();
void AssetFile();

void setup()
{
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SAP, OUTPUT);
  pinMode(LED_REC, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  pinMode(DEBUG_SWITCH, INPUT_PULLUP);
  pinMode(RS485_TNOW, OUTPUT);
  digitalWrite(RS485_TNOW, LOW); // 保证释放485总线

  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  analogWriteFrequency(10000);

  DEBUG_MODE = digitalRead(DEBUG_SWITCH) == LOW;

  rtc_wdt_set_time(RTC_WDT_STAGE0, 1000);

  xTaskCreate(
      LEDControl,        /* 任务函数 */
      "LEDControl",      /* 任务名 */
      2 * 1024,          /* 任务栈大小，根据需要自行设置*/
      NULL,              /* 参数，入参为空 */
      1,                 /* 优先级 */
      &TASK_LEDControl); /* 任务句柄 */
  LED_ERR_ON;
  LED_SAP_ON;
  LED_REC_ON;

  DEBUG.begin(921600);

  DEBUG.println("SCIDrive Recorder by Developer_ken");
  DEBUG.print("Build ");
  DEBUG.print(__DATE__);
  DEBUG.print(" ");
  DEBUG.print(__TIME__);
  DEBUG.print("\n");

  if (DEBUG_MODE)
  {
    DEBUG.println("* DEBUG_MODE_ENABLED *");
  }

#ifdef ENABLE_RTC
  DEBUG.println("* Firmware has RTC enabled *");
  DEBUG.println("Probing...");
  Wire.setPins(RTC_SDA, RTC_SCL);
  Rtc.Begin();
  uint8_t rtcmem[55];
  Rtc.GetMemory(0, rtcmem, 55);
  RtcDateTime compile = RtcDateTime(__DATE__, __TIME__);
  if ((!Rtc.IsDateTimeValid()) || strcmp((FirmwareCompiletimeHash), (char *)rtcmem) != 0 || Rtc.GetDateTime() < compile)
  {
    DEBUG.println("! RTC datetime invalid, setting to compile time...");
    Rtc.SetDateTime(compile);
    Rtc.SetMemory(0, (uint8_t *)FirmwareCompiletimeHash, strlen(FirmwareCompiletimeHash) + 1);
    Rtc.SetIsRunning(true);
  }
  RtcDateTime now = Rtc.GetDateTime();

  DEBUG.printf("RTC datetime: %4d-%02d-%02d %02d:%02d:%02d\n", now.Year(), now.Month(), now.Day(), now.Hour(), now.Minute(), now.Second());
#endif

  // SPIFFS
  {
    if (!SPIFFS.begin(true))
    {
      DEBUG.println("FAILED. SPIFFS partition not correctly programmed.");
      LED_ERR_ON;
      SETSTATE(LED_BLINK, SAP);
      LED_REC_OFF;
      while (1)
        delay(1000);
    }
  }

  // Mount SD card
  {
    DEBUG.println("Mounting SD card...");
    SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
    if (SD.begin(SD_CS, SPI))
    {
      uint32_t total_size = SD.cardSize();
      uint32_t used_size = SD.usedBytes();
      DEBUG.println("SD card mounted:");
      DEBUG.printf("- Type: %d\n", (int)SD.cardType());
      DEBUG.printf("- Size: %lu kB\n", total_size / 1024);
      DEBUG.printf("- Used: %lu kB / %d%\n", used_size / 1024, (int)(used_size * 100 / total_size));
    }
    else
    {
      DEBUG.println("SD card mount failed.");
      LED_ERR_ON;
      LED_SAP_ON;
      SETSTATE(LED_BLINK, REC);

      if (DEBUG_MODE)
      {
        delay(3000);
        DEBUG.println("* DEBUG_MODE override *");
        DEBUG_NO_SDCARD = true;
      }
      else
        while (1)
          delay(1000);
    }
    if (!DEBUG_NO_SDCARD)
      SD.mkdir("/records");
  }

  IniFile confFile("/config.ini");
  if (DEBUG_NO_SDCARD)
  {
    DEBUG.println("* DEBUG_NO_SDCARD override *");
    DEBUG.println("Skip config loading, using developer default.");
  }
  else
  {
    // Load/Prepare config
    {
      DEBUG.println("Loading config...");
      if (!SD.exists("/config.ini"))
      {
        DEBUG.println("/config.ini does not exists.");
        DEBUG.println("- extracting default from SPIFFS...");
        if (!SPIFFS.exists("/config.ini"))
        {
          DEBUG.println("FAILED. SPIFFS partition not correctly programmed.");
          LED_ERR_ON;
          SETSTATE(LED_BLINK, SAP);
          LED_REC_OFF;
          while (1)
            delay(1000);
        }
        File defconf = SPIFFS.open("/config.ini");
        File sdconf = SD.open("/config.ini", "w", true);
        uint8_t copybuf[64];
        while (defconf.available())
        {
          defconf.read(copybuf, sizeof(copybuf));
          sdconf.write(copybuf, sizeof(copybuf));
        }
        sdconf.flush();
        if (sdconf.size() == defconf.size())
        {
          DEBUG.println("Done.");
        }
        else
        {
          DEBUG.println("Writing failed maybe? continue anyway.");
        }
        defconf.close();
        sdconf.close();
      }
      if (!confFile.open())
      {
        DEBUG.println("/config.ini open failed.");
        LED_ERR_ON;
        SETSTATE(LED_BLINK, SAP);
        LED_REC_OFF;
        while (1)
          delay(1000);
      }
    }

    // Parse config
    {
      char inibuffer[128];
      char inivaluebuffer[128];
      String inival;
      if (!confFile.validate(inibuffer, 128))
      {
        inibuffer[127] = '\0';
        DEBUG.println("Config file invalid:");
        DEBUG.println(inibuffer);
        LED_ERR_ON;
        SETSTATE(LED_BLINK, SAP);
        LED_REC_OFF;
        while (1)
          delay(1000);
      }
      bool ls = true;
      ls &= confFile.getValue("FSD", "Address", inivaluebuffer, 128);
      FSD_ADDR = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("FSD", "Baudrate", inivaluebuffer, 128);
      FSD_BAUD = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("FSD", "PollInterval", inivaluebuffer, 128);
      FSD_POLLINGINTERVAL = (inival = inivaluebuffer).toInt();

      ls &= confFile.getValue("IRDA", "Address", inivaluebuffer, 128);
      IRDA_ADDR = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("IRDA", "Baudrate", inivaluebuffer, 128);
      IRDA_BAUD = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("IRDA", "AllowControl", inivaluebuffer, 128);
      IRDA_ALLOWCTRL = inivaluebuffer[0] == '1';

      ls &= confFile.getValue("ESPNOW", "NetKey", inivaluebuffer, 128);
      ESPNOW_KEY = inivaluebuffer;
      ls &= confFile.getValue("ESPNOW", "AllowControl", inivaluebuffer, 128);
      ESPNOW_ALLOWCTRL = inivaluebuffer[0] == '1';

      ls &= confFile.getValue("WIFIAP", "SSID", inivaluebuffer, 128);
      WIFI_AP_SSID = inivaluebuffer;
      ls &= confFile.getValue("WIFIAP", "Passphrase", inivaluebuffer, 128);
      WIFI_AP_PASSWORD = inivaluebuffer;
      ls &= confFile.getValue("WIFIAP", "Channel", inivaluebuffer, 128);
      WIFI_AP_CHANNEL = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("WIFIAP", "Hidden", inivaluebuffer, 128);
      WIFI_HIDDEN = inivaluebuffer[0] == '1';
      ls &= confFile.getValue("WIFIAP", "Clients", inivaluebuffer, 128);
      WIFI_MAX_CLIENTS = (inival = inivaluebuffer).toInt();

      ls &= confFile.getValue("Sample", "StartTrigger", inivaluebuffer, 128);
      START_TRIGGER = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("Sample", "StopTrigger", inivaluebuffer, 128);
      STOP_TRIGGER = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("Sample", "Interval", inivaluebuffer, 128);
      SAMPLE_INTERVAL = (inival = inivaluebuffer).toInt();

      ls &= confFile.getValue("Sensor", "BoardCriticalTemp", inivaluebuffer, 128);
      CRITICAL_TEMP = (inival = inivaluebuffer).toInt();
      ls &= confFile.getValue("Sensor", "BoardMaxTemp", inivaluebuffer, 128);
      MAX_TEMP = (inival = inivaluebuffer).toInt();
      if (confFile.getValue("Sensor", "OnboardSensorId", inivaluebuffer, 128))
      {
        hexStringToByteArray(inivaluebuffer, ONBOARD_SENSOR_ID, sizeof(ONBOARD_SENSOR_ID));
      }
      else
      {
        memset(ONBOARD_SENSOR_ID, 0, sizeof(ONBOARD_SENSOR_ID));
      }

      confFile.close();
      DEBUG.println("---- Current Configuration ----");
      DEBUG.printf("FSD_ADDR=%02X\n", FSD_ADDR);
      DEBUG.printf("FSD_BAUD=%d\n", FSD_BAUD);
      DEBUG.printf("FSD_POLL=%d\n", FSD_POLLINGINTERVAL);
      DEBUG.printf("IRDA_ADDR=%02X\n", IRDA_ADDR);
      DEBUG.printf("IRDA_BAUD=%d\n", IRDA_BAUD);
      DEBUG.printf("IRDA_CTRL=%c\n", IRDA_ALLOWCTRL ? 'Y' : 'N');
      DEBUG.printf("ESPN_KEY=%s\n", ESPNOW_KEY.c_str());
      DEBUG.printf("ESPN_CTRL=%c\n", ESPNOW_ALLOWCTRL ? 'Y' : 'N');
      DEBUG.printf("START_TRIGGER=%d\n", START_TRIGGER);
      DEBUG.printf("STOP_TRIGGER=%d\n", STOP_TRIGGER);
      DEBUG.printf("SAMPLE_INTERVAL=%d\n", SAMPLE_INTERVAL);
      DEBUG.printf("CRITICAL_TEMP=%d\n", CRITICAL_TEMP);
      DEBUG.printf("MAX_TEMP=%d\n", MAX_TEMP);
      DEBUG.printf("ONBOARD_SENSOR_ID=%02X%02X%02X%02X%02X%02X%02X%02X\n",
                   ONBOARD_SENSOR_ID[0], ONBOARD_SENSOR_ID[1],
                   ONBOARD_SENSOR_ID[2], ONBOARD_SENSOR_ID[3],
                   ONBOARD_SENSOR_ID[4], ONBOARD_SENSOR_ID[5],
                   ONBOARD_SENSOR_ID[6], ONBOARD_SENSOR_ID[7]);
      DEBUG.println("-------------------------------");

      if (!ls)
      {
        DEBUG.println("Failed parsing some of the config entrys.");
        DEBUG.println("Check you have latest version of config template.");
        LED_ERR_ON;
        SETSTATE(LED_BLINK, SAP);
        LED_REC_OFF;
        while (1)
          delay(1000);
      }
    }
  }
  // Communication init
  {
    DEBUG.println("Init RS485 and IRDA...");
    RS485.setPins(RS485_RX, RS485_TX);
    RS485.begin(FSD_BAUD);
    IRDA.setPins(IRDA_RX, IRDA_TX);
    IRDA.begin(IRDA_BAUD);
    IRDA_Init();
    modbus.begin(FSD_ADDR, RS485);
    modbus.preTransmission(RS485TXNOW);
    modbus.postTransmission(RS485RXNOW);
    modbus.idle(IOHangup); // yield when idle, preventing blocking.
  }

  // 1Wire Temperature sensors
  {
    DEBUG.println("Init OneWire bus...");
    oneWire.begin(TEMPSENSOR);
    M1820::oneWire = &oneWire;
    DEBUG.println("Scanning the OneWire bus for M1820 sensors...");
    byte address[8]; // array to hold the address of each sensor
    bool set_obsid = isAllZero(ONBOARD_SENSOR_ID, sizeof(ONBOARD_SENSOR_ID));
    if (set_obsid)
    {
      DEBUG.println("! ONBOARD_SENSOR_ID not set. First sensor found will be filled there.");
    }
    while (oneWire.search(address))
    {
      if (bytesEquals(address, ONBOARD_SENSOR_ID, sizeof(ONBOARD_SENSOR_ID)))
      {
        OnboardTempSensor.BindAddr(address);
        OnboardTempSensor.Valid = true;
        DEBUG.println(" - Matched onboard sensor.");
        continue;
      }
      DEBUG.printf(" - [%d] Hit addr = %02X%02X%02X%02X%02X%02X%02X%02X\n", SensorCount,
                   address[0], address[1], address[2], address[3], address[4], address[5],
                   address[6], address[7]);
      if (set_obsid)
      {
        File conffile = SD.open("/config.ini", "a");
        conffile.println();
        conffile.printf("OnboardSensorId = %02X%02X%02X%02X%02X%02X%02X%02X",
                        address[0], address[1], address[2], address[3],
                        address[4], address[5], address[6], address[7]);
        conffile.println();
        conffile.close();
        set_obsid = false;
        DEBUG.println("! ONBOARD_SENSOR_ID set.");
        OnboardTempSensor.BindAddr(address);
        OnboardTempSensor.Valid = true;
        DEBUG.println(" - Set as onboard sensor.");
        continue;
      }
      if (SensorCount >= 9)
      {
        DEBUG.println("More than 8 (external) sensors found!");
        DEBUG.println("Remove extra sensors, then reboot.");
        LED_ERR_ON;
        LED_REC_OFF;
        LED_SAP_ON;
        while (1)
          delay(1000);
      }
      TempSensors[SensorCount].BindAddr(address);
      TempSensors[SensorCount].Valid = true;
      SensorCount++;
    }
    DEBUG.printf("Scan complete. %d sensors found.\n", SensorCount);
    if (OnboardTempSensor.Valid == false)
    {
      DEBUG.println("Error: Onboard sensor not found on bus.");
      LED_ERR_ON;
      LED_REC_OFF;
      LED_SAP_ON;
      while (1)
        delay(1000);
    }
    DEBUG.println("System init done.");
  }

  // Working temperature check
  {
    DEBUG.println("Checking working temperature...");
    OnboardTempSensor.SampleNow();
    float temperature = OnboardTempSensor.receiveTemperature();
    DEBUG.printf("TEMP_OB_SENSOR=%.2f\n", temperature);
    if (temperature < -25 || temperature > 80)
    {
      DEBUG.println("Warning: System designed to work only under -25°C ~ 80°C.");
      SETSTATE(LED_BLINK, ERR);
    }
  }

  // Wifi AP
  {
    DEBUG.println("Init WiFi AP...");
    IPAddress local_IP(192, 168, 45, 1);
    IPAddress gateway(192, 168, 45, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFiAP.softAPConfig(local_IP, gateway, subnet);
    WiFiAP.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, WIFI_HIDDEN, WIFI_MAX_CLIENTS);
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    DEBUG.print("Created AP \"");
    DEBUG.print(WIFI_AP_SSID);
    DEBUG.print("\" with password \"");
    DEBUG.print(WIFI_AP_PASSWORD);
    DEBUG.print("\" @ch");
    DEBUG.print(WIFI_AP_CHANNEL);
    DEBUG.print(".\n");

    ESPNow.init();
    ESPNow.add_peer(BROADCAST_ADDR);
  }

  // Web
  {
    webServer.begin();
    webServer.on("/", HandleRootWeb);
    webServer.on("/api/status", GetStatus);
    webServer.on("/api/files", ListFiles);
    webServer.on("/api/settime", SetTime);
    webServer.on(UriBraces("/api/assets/{}"), AssetFile);
    webServer.on(UriBraces("/api/download/{}"), DownloadFile);
    webServer.onNotFound(RedirectHome);
  }

  // DNS
  {
    dnsServer.start(53, "*", WiFiAP.softAPIP());
  }

  xTaskCreate(
      Sample,        /* 任务函数 */
      "Sample",      /* 任务名 */
      2 * 1024,      /* 任务栈大小，根据需要自行设置*/
      NULL,          /* 参数，入参为空 */
      1,             /* 优先级 */
      &TASK_Sample); /* 任务句柄 */
  xTaskCreate(
      Poll,        /* 任务函数 */
      "Poll",      /* 任务名 */
      4 * 1024,    /* 任务栈大小，根据需要自行设置*/
      NULL,        /* 参数，入参为空 */
      1,           /* 优先级 */
      &TASK_Poll); /* 任务句柄 */
  xTaskCreate(
      FSDStateBroadcaster,   /* 任务函数 */
      "FSDStateBroadcaster", /* 任务名 */
      2 * 1024,              /* 任务栈大小，根据需要自行设置*/
      NULL,                  /* 参数，入参为空 */
      1,                     /* 优先级 */
      &TASK_ESPNow);         /* 任务句柄 */
  LED_ERR_OFF;
  LED_SAP_OFF;
  SETSTATE(LED_CYCLE, REC);
}

void loop()
{
  webServer.handleClient();
  delay(1);
  dnsServer.processNextRequest();
  delay(1);
}

inline void _applyLEDState(uint8_t pin, uint8_t state, uint8_t bstate)
{
  if (state == LED_ON)
    analogWrite(pin, 203);
  else if (state == LED_OFF)
    analogWrite(pin, 255);
  else if (state == LED_BLINK)
    analogWrite(pin, (bstate > 456 / 2) ? 255 : 203);
  else if (state == LED_CYCLE)
    analogWrite(pin, bstate);
}

void LEDControl(void *param)
{
  uint8_t bState = 0;
  bool direction = true;
  while (true)
  {
    _applyLEDState(LED_ERR, (LED_STATE & (0b11 << ERR)) >> ERR, bState);
    _applyLEDState(LED_SAP, (LED_STATE & (0b11 << SAP)) >> SAP, bState);
    _applyLEDState(LED_REC, (LED_STATE & (0b11 << REC)) >> REC, bState);
    if (bState <= 203)
    {
      bState = 203;
      direction = true;
    }
    else if (bState >= 251)
    {
      bState = 251;
      direction = false;
    }
    if (direction)
    {
      bState += 4;
    }
    else
    {
      bState -= 4;
    }
    delay(50);
  }
}

void Sample(void *param)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xLastflashTime;
  const TickType_t taskPeriod = SAMPLE_INTERVAL;
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, taskPeriod);

    M1820::SampleNow();
    temp = OnboardTempSensor.receiveTemperature();
    for (int i = 0; i < SensorCount; i++)
    {
      TempSensors[i].receiveTemperature();
    }

    if (!sampling)
    {
      continue;
    }
    xLastflashTime = xTaskGetTickCount();
    LED_SAP_ON;
    bool poll_warn = false;
    { // 在这里采样和保存
      if (abs(static_cast<long>(PollTime - xLastWakeTime)) > 2 * FSD_POLLINGINTERVAL)
      {
        DEBUG.println("WARNING: Polling can't keep up the pace.");
        LED_ERR_ON;
        poll_warn = true;
      }
      {
        // 写变频器参数
        if (poll_warn)
        {
          recordfile.printf("%u,POLLING_DID_NOT_KEEP_UP,,,,,,,,,,,,,,,", xLastWakeTime);
        }
        else
        {
          recordfile.printf("%u,%04hX,%04hX,%f,%f,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%04hX",
                            xLastWakeTime, latest_fsd_status.StateWord, latest_fsd_status.MalfunctionWord,
                            latest_fsd_status.TargetFrequency, latest_fsd_status.CurrentFrequency,
                            latest_fsd_status.RailVotage, latest_fsd_status.OutputVotage,
                            latest_fsd_status.OutputCurrent, latest_fsd_status.OutputPower,
                            latest_fsd_status.OutputFrequency, latest_fsd_status.OutputTorque,
                            latest_fsd_status.SystemTemperature, latest_fsd_status.MotorRPM,
                            latest_fsd_status.AI1Val, latest_fsd_status.AI2Val, latest_fsd_status.PulseInFreq,
                            latest_fsd_status.DigitalInState);
          rawfile.write((byte *)(&xLastWakeTime), 4);         // 时间戳   4byte
          rawfile.write((byte *)(latest_fsd_status.Raw), 32); // 寄存器值 32byte (16寄存器*2byte)
        }
      }
      {
        // 写温度传感器参数
        recordfile.printf(",%f", temp);
        for (int i = 0; i < SensorCount; i++)
        {
          recordfile.printf(",%f", TempSensors[i].LastTemperature);
        }
        if (temp < -25 || temp > 80)
        {
          DEBUG.printf("Warning: TEMP_OB_SENSOR=%.2f overrange! (-25°C ~ 80°C)\n", temp);
          SETSTATE(LED_BLINK, ERR);
        }
      }
      {
        // 写Modbus通信状态
        recordfile.printf(",%u,%u", modbus_sent, modbus_error);
      }
      recordfile.println(); // 行末尾
      recordfile.flush();   // 写入卡
      rawfile.flush();      // 寄存器记录写入卡
    }
    vTaskDelayUntil(&xLastflashTime, 150);
    LED_SAP_OFF;
    if (poll_warn)
      LED_ERR_OFF;
  }
}

void RecEnd()
{
  sampling = false;
  recordfile.flush();
  recordfile.close();
  rawfile.flush();
  rawfile.close();
  DEBUG.println("Record stop.");
  LED_REC_OFF;
}

void RecStart()
{
  DEBUG.println("Trigger an rec-start.");
  if (sampling == true)
  {
    DEBUG.println("Already recording. CMD ignore.");
    return;
  }
  char filepath[64];
  RtcDateTime now = Rtc.GetDateTime();
  for (int i = 0; i < 2147483647; i++)
  {
#ifdef ENABLE_RTC
    sprintf(filepath, "/records/%04d-%02d-%02d-%02d.%02d.%02d_%d.csv", now.Year(), now.Month(), now.Day(), now.Hour(), now.Minute(), now.Second(), i);
#else
    sprintf(filepath, "/records/%d.csv", i);
#endif
    if (!SD.exists(filepath))
      break;
  }
  DEBUG.println("Recording into file:");
  DEBUG.println(filepath);

  recordfile = SD.open(filepath, "w", true);
  rawfile = SD.open(String(filepath) + ".hex", "w", true);
  DEBUG.println("Opened that file.");
  recordfile.print("Time(ms),Status(WORD),Malfunction(WORD),TargetFreq(Hz),CurrentFreq(Hz),RailVoltage(V),OutputVotage(V),OutputCurrent(A),OutputPower(W),OutputFreq(Hz),OutputTorque(%),SystemTemp(C),MotorSpeed(RPM),AI1(V),AI2(V),PulseIn(kHz),DigitalIn(WORD),BoardTemp(C)");
  for (int i = 0; i < SensorCount; i++)
  {
    recordfile.printf(",%02X%02X%02X%02X%02X%02X%02X%02X(C)",
                      TempSensors[i].Address[0], TempSensors[i].Address[1],
                      TempSensors[i].Address[2], TempSensors[i].Address[3],
                      TempSensors[i].Address[4], TempSensors[i].Address[5],
                      TempSensors[i].Address[6], TempSensors[i].Address[7]);
  }
  recordfile.print(",Modbus Sent,Modbus Err");
  recordfile.println();
  recordfile.flush();
  sampling = true;
  LED_REC_ON;
}

void Poll(void *param)
{
  PollTime = xTaskGetTickCount();
  const TickType_t taskPeriod = FSD_POLLINGINTERVAL;
  while (true)
  {
    vTaskDelayUntil(&PollTime, taskPeriod);
    {
      // 在这里拉取变频器状态
      uint8_t scode = modbus.readHoldingRegisters(0x7100, 16);
      modbus_sent++;
      if (scode == 0)
      {
        for (int i = 0; i < 16; i++)
        {
          latest_fsd_status.Raw[i] = modbus.getResponseBuffer(i);
        }
        latest_fsd_status.StateWord = modbus.getResponseBuffer(0x00);
        latest_fsd_status.MalfunctionWord = modbus.getResponseBuffer(0x01);
        latest_fsd_status.TargetFrequency = modbus.getResponseBuffer(0X02) * 0.01;
        latest_fsd_status.CurrentFrequency = modbus.getResponseBuffer(0x03) * 0.01;
        latest_fsd_status.RailVotage = modbus.getResponseBuffer(0x04);
        latest_fsd_status.OutputVotage = modbus.getResponseBuffer(0x05);
        latest_fsd_status.OutputCurrent = modbus.getResponseBuffer(0x06) * 0.1;
        latest_fsd_status.OutputPower = (int16_t)modbus.getResponseBuffer(0x07) * 0.1;
        latest_fsd_status.OutputFrequency = modbus.getResponseBuffer(0x08) * 0.01;
        latest_fsd_status.OutputTorque = (int16_t)modbus.getResponseBuffer(0x09) * 0.1;
        latest_fsd_status.SystemTemperature = (int16_t)modbus.getResponseBuffer(0x0A) * 0.1;
        latest_fsd_status.MotorRPM = modbus.getResponseBuffer(0x0B);
        latest_fsd_status.AI1Val = modbus.getResponseBuffer(0x0C) * 0.01;
        latest_fsd_status.AI2Val = modbus.getResponseBuffer(0x0D) * 0.01;
        latest_fsd_status.PulseInFreq = modbus.getResponseBuffer(0x0E) * 0.01;
        latest_fsd_status.DigitalInState = modbus.getResponseBuffer(0x0F);
        if (!sampling)
        {
          LED_ERR_OFF;
        }
      }
      else
      {
        if (!sampling)
        {
          SETSTATE(LED_CYCLE, ERR);
        }
        DEBUG.printf("Polling FSD status failed with code 0x%02X\n", scode);
        modbus_error++;
      }
      if (!sampling)
      {
        switch (START_TRIGGER)
        {
        case 1:
          if (latest_fsd_status.RailVotage > 450)
          {
            DEBUG.println("Power-On-Start-Trigger triggered.");
            RecStart();
          }
          break;
        case 2:
          if (latest_fsd_status.StateWord == 0b0001 || latest_fsd_status.StateWord == 0b0010)
          {
            DEBUG.println("FSD-Run-Trigger triggered.");
            RecStart();
          }
          break;
        }
      }
      else
      {
        switch (STOP_TRIGGER)
        {
        case 1:
          if (latest_fsd_status.RailVotage <= 200)
          {
            DEBUG.println("FSD-Power-Down-Trigger triggered.");
            RecEnd();
          }
          break;
        case 2:
          if (latest_fsd_status.StateWord == 0b0011)
          {
            DEBUG.println("FSD-Stop-Trigger triggered.");
            RecStart();
          }
          break;
        }
      }
    }
  }
}

void hexStringToByteArray(const char *hexString, unsigned char *byteArray, size_t byteArraySize)
{
  for (size_t i = 0; i < byteArraySize; i++)
  {
    sscanf(hexString + 2 * i, "%2hhx", &byteArray[i]); // 每次读取两个字符并转换为byte
  }
}

void ByteArrayToHexString(const unsigned char *byteArray, size_t byteArraySize, char *hexString)
{
  for (size_t i = 0; i < byteArraySize; i++)
  {
    sprintf(hexString + 2 * i, "%02X", byteArray[i]); // 每次写入两个字符
  }
}

bool bytesEquals(byte *bytes1, byte *bytes2, int len)
{
  for (int i = 0; i < len; i++)
  {
    if (bytes1[i] != bytes2[i])
      return false;
  }
  return true;
}

bool isAllZero(byte *bytes, int len)
{
  for (int i = 0; i < len; i++)
    if (bytes[i] != 0)
      return false;
  return true;
}

void RS485TXNOW()
{
  digitalWrite(RS485_TNOW, HIGH);
}

void RS485RXNOW()
{
  digitalWrite(RS485_TNOW, LOW);
}

void IOHangup()
{
  delay(10);
}

void FSDStateBroadcaster()
{
  ESPNow.send_message(BROADCAST_ADDR, (uint8_t *)&latest_fsd_status, sizeof(FSDState));
  delay(100);
}

void HandleRootWeb()
{
  auto f = SPIFFS.open("/index.html");
  webServer.streamFile(f, "text/html", 200);
  f.close();
}

void RedirectHome()
{
  webServer.sendHeader("Location", "http://" + WiFiAP.softAPIP().toString());
  webServer.send(302, "text/plain", "302 Redirect to lading page");
}

void GetStatus()
{
  String statusjson;
  statusjson += '{';
  statusjson += "\"LTime\":";
  statusjson += xTaskGetTickCount();
  statusjson += ',';
  statusjson += "\"Freq\":";
  statusjson += latest_fsd_status.CurrentFrequency;
  statusjson += ',';
  statusjson += "\"RVolt\":";
  statusjson += latest_fsd_status.RailVotage;
  statusjson += ',';
  statusjson += "\"OVolt\":";
  statusjson += latest_fsd_status.OutputVotage;
  statusjson += ',';
  statusjson += "\"OCurr\":";
  statusjson += latest_fsd_status.OutputCurrent;
  statusjson += ',';
  statusjson += "\"OPwr\":";
  statusjson += latest_fsd_status.OutputPower;
  statusjson += ',';
  statusjson += "\"OTorq\":";
  statusjson += latest_fsd_status.OutputTorque;
  statusjson += ',';
  statusjson += "\"IGBTTemp\":";
  statusjson += latest_fsd_status.SystemTemperature;
  statusjson += ',';
  statusjson += "\"RPM\":";
  statusjson += latest_fsd_status.MotorRPM;
  statusjson += ',';
  statusjson += "\"AI1\":";
  statusjson += latest_fsd_status.AI1Val;
  statusjson += ',';
  statusjson += "\"AI2\":";
  statusjson += latest_fsd_status.AI2Val;
  statusjson += ',';
  statusjson += "\"PulseIn\":";
  statusjson += latest_fsd_status.PulseInFreq;
  statusjson += ',';
  statusjson += "\"DigitalIn\":";
  statusjson += latest_fsd_status.DigitalInState;
  statusjson += ',';
  statusjson += "\"StaWord\":";
  statusjson += latest_fsd_status.StateWord;
  statusjson += ',';
  statusjson += "\"ObTemp\":";
  statusjson += temp;
  statusjson += ',';
  statusjson += "\"ErrWord\":";
  statusjson += latest_fsd_status.MalfunctionWord;
  statusjson += ',';
  statusjson += "\"ExtSensors\":{";
  for (int i = 0; i < 8; i++)
  {
    if (!TempSensors[i].Valid)
      break;
    char temp[17];
    ByteArrayToHexString(TempSensors[i].Address, 8, temp);
    if (i > 0)
      statusjson += ",";
    statusjson += "\"" + String(temp) + "\":";
    statusjson += TempSensors[i].LastTemperature;
  }
  statusjson += "}}";
  webServer.send(200, "application/json", statusjson);
}

void ListFiles()
{
  File root = SD.open("/records");
  File file = root.openNextFile();
  webServer.sendContent("{\"files\":[");
  bool fst = true;
  while (file)
  {
    if (fst)
    {
      fst = false;
    }
    else
    {
      webServer.sendContent(",");
    }
    webServer.sendContent("\"" + String(file.name()) + "\"");
    file = root.openNextFile();
  }
  webServer.sendContent("]}");
}

void DownloadFile()
{
  String filename = webServer.pathArg(0);
  File file = SD.open("/records/" + filename);
  if (!file)
  {
    webServer.send(404, "text/plain", "404 File not found");
    return;
  }
  webServer.streamFile(file, "application/octet-stream");
  file.close();
}

void AssetFile()
{
  String filename = webServer.pathArg(0);
  File file = SPIFFS.open("/" + filename);
  if (!file)
  {
    webServer.send(404, "text/plain", "404 File not found");
    return;
  }
  webServer.streamFile(file, ContentType(filename));
  file.close();
}

void SetTime()
{
  String timestamp = webServer.arg("timestamp");
  RtcDateTime time;
  time = Rtc.GetDateTime();
  DEBUG.println("Client requested a time sync.");
  DEBUG.print("Local time:");
  DEBUG.print(time.Year());
  DEBUG.print('/');
  DEBUG.print(time.Month());
  DEBUG.print('/');
  DEBUG.print(time.Day());
  DEBUG.print(' ');
  DEBUG.print(time.Hour());
  DEBUG.print(':');
  DEBUG.print(time.Minute());
  DEBUG.print(':');
  DEBUG.println(time.Second());

  time.InitWithUnix32Time(timestamp.toInt() + 8 * 60 * 60); // 本地使用UTC+8时区

  DEBUG.print("Client time:");
  DEBUG.print(time.Year());
  DEBUG.print('/');
  DEBUG.print(time.Month());
  DEBUG.print('/');
  DEBUG.print(time.Day());
  DEBUG.print(' ');
  DEBUG.print(time.Hour());
  DEBUG.print(':');
  DEBUG.print(time.Minute());
  DEBUG.print(':');
  DEBUG.println(time.Second());

  DEBUG.println("Updating RTC...");
  Rtc.SetDateTime(time);

  time = Rtc.GetDateTime();
  DEBUG.println("Readback:");
  DEBUG.print(time.Year());
  DEBUG.print('/');
  DEBUG.print(time.Month());
  DEBUG.print('/');
  DEBUG.print(time.Day());
  DEBUG.print(' ');
  DEBUG.print(time.Hour());
  DEBUG.print(':');
  DEBUG.print(time.Minute());
  DEBUG.print(':');
  DEBUG.println(time.Second());
  webServer.sendContent("{\"status\":\"ok\"}");
}

String ContentType(String filename)
{
  String extention = filename.substring(filename.lastIndexOf('.') + 1);
  extention.toLowerCase();
  if (extention == "html")
  {
    return "text/html";
  }
  else if (extention == "css")
  {
    return "text/css";
  }
  else if (extention == "js")
  {
    return "application/javascript";
  }
  else if (extention == "json")
  {
    return "application/json";
  }
  else if (extention == "png")
  {
    return "image/png";
  }
  else if (extention == "jpg")
  {
    return "image/jpeg";
  }
}