#include <Arduino.h>
#include <IniFile.h>
#include <SPIFFS.h>
#include <M1820.h>
#include <SPI.h>

#include <SD.h>
#include <FS.h>

#include "irda.h"
#include "pinout.h"

#define DEBUG Serial
#define RS485 Serial1
#define IRDA Serial2

#define SETSTATE(STATE, INDEX) (LED_STATE = (LED_STATE & ~(0b11 << INDEX)) | (STATE << INDEX))

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
uint8_t FSD_ADDR;
uint32_t FSD_BAUD;
uint32_t FSD_POLLINGINTERVAL;

uint8_t IRDA_ADDR;
uint32_t IRDA_BAUD;
bool IRDA_ALLOWCTRL;

String ESPNOW_KEY;
bool ESPNOW_ALLOWCTRL;

uint8_t START_TRIGGER,
    STOP_TRIGGER;
uint32_t SAMPLE_INTERVAL = 5000; // ms, 采样周期

uint8_t CRITICAL_TEMP;
uint8_t MAX_TEMP;
byte ONBOARD_SENSOR_ID[8];
//////
M1820 OnboardTempSensor; // 1 onboard sensor
M1820 TempSensors[8];    // 8 external sensors
OneWire oneWire;         // OneWire bus
int SensorCount = 0;
uint8_t LED_STATE = 0;
uint64_t LASTSAMPLE = 0;
TaskHandle_t TASK_LEDControl = NULL, TASK_Sample = NULL, TASK_Poll = NULL;
bool sampling = false;

void LEDControl(void *);
void Sample(void *);
void Poll(void *);
void LoadConfig();
bool isAllZero(byte *bytes, int len);
void hexStringToByteArray(const char *hexString, unsigned char *byteArray, size_t byteArraySize);
bool bytesEquals(byte *bytes1, byte *bytes2, int len);

void setup()
{
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SAP, OUTPUT);
  pinMode(LED_REC, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  TaskHandle_t TASK_HandleOne = NULL;
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

  DEBUG.begin(115200);

  DEBUG.println("SCIDrive Recorder by Developer_ken");
  DEBUG.print("Build ");
  DEBUG.print(__DATE__);
  DEBUG.print(" ");
  DEBUG.print(__TIME__);
  DEBUG.print("\n");

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
      while (1)
        delay(1000);
    }
  }
  // SD.remove("/config.ini");
  IniFile confFile("/config.ini");
  // Load/Prepare config
  {
    DEBUG.println("Loading config...");
    if (!SD.exists("/config.ini"))
    {
      DEBUG.println("/config.ini does not exists.");
      DEBUG.println("- extracting default from SPIFFS...");
      if (!SPIFFS.begin(true) || !SPIFFS.exists("/config.ini"))
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

  // Communication init
  {
    DEBUG.println("Init RS485 and IRDA phy...");
    RS485.setPins(RS485_RX, RS485_TX);
    RS485.begin(FSD_BAUD);
    IRDA.setPins(IRDA_RX, IRDA_TX);
    IRDA.begin(IRDA_BAUD);
    IRDA_Init();
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
        TempSensors[SensorCount].Valid = true;
        DEBUG.println(" - Matched onboard sensor.");
        continue;
      }
      DEBUG.printf(" - [%d] Hit addr = %02X%02X%02X%02X%02X%02X%02X%02X\n", SensorCount,
                   address[0], address[1], address[2], address[3], address[4], address[5],
                   address[6], address[7]);
      if (set_obsid)
      {
        File conffile = SD.open("/config.ini", "a");
        conffile.printf("\nOnboardSensorId = %02X%02X%02X%02X%02X%02X%02X%02X\n",
                        address[0], address[1], address[2], address[3],
                        address[4], address[5], address[6], address[7]);
        conffile.close();
        set_obsid = false;
        DEBUG.println("! ONBOARD_SENSOR_ID set.");
        OnboardTempSensor.BindAddr(address);
        TempSensors[SensorCount].Valid = true;
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
    if (TempSensors[SensorCount].Valid = false)
    {
      DEBUG.println("Error: Onboard sensor not found on bus.");
      LED_ERR_ON;
      LED_REC_OFF;
      LED_SAP_ON;
      while (1)
        delay(1000);
    }
  }

  LED_ERR_OFF;
  LED_REC_OFF;
  SETSTATE(LED_CYCLE, SAP);
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
      2 * 1024,    /* 任务栈大小，根据需要自行设置*/
      NULL,        /* 参数，入参为空 */
      1,           /* 优先级 */
      &TASK_Poll); /* 任务句柄 */
}

void loop()
{
  delay(10000);
}

inline void _applyLEDState(uint8_t pin, uint8_t state, uint8_t bstate)
{
  if (state == LED_ON)
    digitalWrite(pin, LOW);
  else if (state == LED_OFF)
    digitalWrite(pin, HIGH);
  else if (state == LED_BLINK)
    digitalWrite(pin, (bstate > 456 / 2) ? HIGH : LOW);
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
      bState += 8;
    }
    else
    {
      bState -= 8;
    }
    delay(100);
  }
}

void Sample(void *param)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  TickType_t xLastflashTime;
  const TickType_t taskPeriod = SAMPLE_INTERVAL;
  LED_SAP_OFF;
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
    if (!sampling)
      continue;
    xLastflashTime = xTaskGetTickCount();
    LED_SAP_ON;
    { // 在这里采样和保存
      M1820::SampleNow();
      float temp = TempSensors[0].receiveTemperature();
      DEBUG.printf("Time:%d Temp: %.2f\n", xLastWakeTime, temp);
    }
    vTaskDelayUntil(&xLastflashTime, 150);
    LED_SAP_OFF;
  }
}

void Poll(void *param)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t taskPeriod = FSD_POLLINGINTERVAL;
  while (true)
  {
    vTaskDelayUntil(&xLastWakeTime, taskPeriod);
    {
      //在这里拉取变频器状态

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