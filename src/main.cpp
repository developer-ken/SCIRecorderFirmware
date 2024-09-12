#include <Arduino.h>
#include <IniFile.h>
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

M1820 TempSensors[9]; // 1 internal sensors and 8 external sensors
OneWire oneWire;      // OneWire bus
int SensorCount = 0;
uint8_t LED_STATE = 0;
uint64_t LASTSAMPLE = 0;

void LEDControl(void *);
void Sample(void *param);
void LoadConfig();

void setup()
{
  pinMode(LED_ERR, OUTPUT);
  pinMode(LED_SAP, OUTPUT);
  pinMode(LED_REC, OUTPUT);
  pinMode(SD_CS, OUTPUT);

  TaskHandle_t TASK_HandleOne = NULL;
  xTaskCreate(
      LEDControl,       /* 任务函数 */
      "LEDControl",     /* 任务名 */
      2 * 1024,         /* 任务栈大小，根据需要自行设置*/
      NULL,             /* 参数，入参为空 */
      1,                /* 优先级 */
      &TASK_HandleOne); /* 任务句柄 */
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

  DEBUG.println("Mounting SD card...");
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, SPI))
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

  DEBUG.println("Loading config...");
  const char *ccharx = "/config.ini";
  IniFile confFile(ccharx);
  if (!confFile.open())
  {
    DEBUG.println("/config.ini does not exists.");
    LED_ERR_ON;
    SETSTATE(LED_BLINK, SAP);
    LED_REC_OFF;
    while (1)
      delay(1000);
  }

  {
    char inibuffer[128];
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
  }

  DEBUG.println("Init RS485 and IRDA phy...");
  RS485.setPins(RS485_RX, RS485_TX);
  RS485.begin(230400);
  IRDA.setPins(IRDA_RX, IRDA_TX);
  IRDA.begin(9600);
  IRDA_Init();

  DEBUG.println("Init OneWire bus...");
  oneWire.begin(TEMPSENSOR);
  M1820::oneWire = &oneWire;
  DEBUG.println("Scanning the OneWire bus for M1820 sensors...");
  byte address[8]; // array to hold the address of each sensor
  while (oneWire.search(address))
  {
    DEBUG.printf(" - [%d] Hit addr = %02X%02X%02X%02X%02X%02X%02X%02X\n", SensorCount,
                 address[0], address[1], address[2], address[3], address[4], address[5],
                 address[6], address[7]);
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

  LED_ERR_OFF;
  LED_REC_OFF;
  SETSTATE(LED_CYCLE, SAP);
  xTaskCreate(
      Sample,           /* 任务函数 */
      "Sample",         /* 任务名 */
      2 * 1024,         /* 任务栈大小，根据需要自行设置*/
      NULL,             /* 参数，入参为空 */
      1,                /* 优先级 */
      &TASK_HandleOne); /* 任务句柄 */
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
    xLastflashTime = xTaskGetTickCount();
    LED_SAP_ON;
    { // 在这里采样和保存
      M1820::SampleNow();
      float temp = TempSensors[0].receiveTemperature();
      DEBUG.printf("Time:%d Temp: %.2f\n", xLastWakeTime, temp);
    }
    vTaskDelayUntil(&xLastflashTime, 250);
    LED_SAP_OFF;
  }
}

void LoadConfig()
{
  // confFile.getValue("FSD");
}