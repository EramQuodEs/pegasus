#include<HardwareSerial.h>
#include <Arduino.h>        //needed for various things
#include <ESP32CAN.h>       //needed for CAN modem
#include <CAN_config.h>     //needed for CAN modem
#include <WiFiMulti.h>
#include <Adafruit_Sensor.h> //needed for IMU
#include <TinyGPS.h>         //needed for GPS  
#include <WebSocketsClient.h>//needed for WS
#include <stdlib.h>
#include <Preferences.h>
#include <time.h>
#include "UDHttp.h"

#ifdef DEBUG_ESP_PORT
#define DEBUG_ESP_PORT Serial
#endif


#include <HTTPClient.h>
#include <HTTPUpdate.h>
WiFiClient client;

//#include "BluetoothSerial.h"
//
//#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
//#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
//#endif
//
//BluetoothSerial SerialBT;


#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "MPU9250.h"          //needed for IMU
#include "Adafruit_BMP280.h"  //needed for BAR
//#define TINY_GPS_RES_M6
#define TINY_GPS_RES_M8


//Occupied pins from the modem
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22

#define DEBUG_ESP_PORT Serial
// Set serial for AT commands (to the module)
//#define SerialAT  Serial1 !!reverted to WS
//TinyGsm modem(SerialAT); !!reverted to WS

File root;
#define CAN_TAG 0//
#define GPS_TAG 1//
#define ALP_TAG 2//
#define GYR_TAG 3//
#define MAG_TAG 4//
#define TMP_TAG 5//





WiFiMulti WiFiMulti;
Adafruit_BMP280 bmp;          //needed for BAR
MPU9250 mpu;                  //needed for IMU
TinyGPS gps;                  //needed for GPS
Preferences preferences;
WebSocketsClient webSocket;

// Your GPRS credentials
// Leave empty, if missing user or pass
//const char apn[]  = "internet.vodafone.gr";
//const char user[] = "";
//const char pass[] = "";

#define SD_CS_PIN   2
#define GPS_RX_PIN  34
#define GPS_TX_PIN  15        //TX pin not needed for GPS
#define softwareVersion  "fwvcurrent"
#define fileCounter      "fCounter"
#define server           "fortion.hopto.org"
boolean mpuReady = false;
enum SystemStates {
  ss_OTAUpdate,       //OTA Update check
  ss_DataFlush,       //Transfer orphan data
  ss_InitMeasure,     //Init meas system
  ss_Measure,         //Meas
  ss_Idle             //Check Wifi and select route
};
volatile float quart[4] = {0, 0, 0, 0};
volatile float accel[3] = {0, 0, 0};
volatile float gyros[3] = {0, 0, 0};
volatile float magne[3] = {0, 0, 0};

volatile float  fquart[4] = {0, 0, 0, 0};
volatile float  faccel[3] = {0, 0, 0};
volatile float  fgyros[3] = {0, 0, 0};
volatile float  fmagne[3] = {0, 0, 0};
float flat, flon, falt, fcrs, fspd;

volatile float fbquart[4] = {0.88, 0.88, 0.88, 0.88};
volatile float fbaccel[3] = {0.88, 0.88, 0.88};
volatile float fbgyros[3] = {0.88, 0.88, 0.88};
volatile float fbmagne[3] = {0.58, 0.58, 0.58};

volatile float pressure;   //To store the barometric pressure (Pa)
volatile float temperature;  //To store the temperature (oC)
volatile float altimeter;
volatile boolean SDcritical = false;
volatile boolean broadacstCANtowebpage = false;
int broadacstCANtowebpageCount = 0;
//------
int measurement_cnt = 0;
char measHolder[5160];///<<<!important:: Small measHolder may lead in stack smash!
//uint8_t BTPayload[2160];
//boolean BTPLready=false;

int fmc;
volatile boolean freshIMU = false;
//boolean BLEpayloadReady=false;
CAN_device_t CAN_cfg;               // CAN Config
unsigned long previousMillis = 0;   // will store last time a CAN Message was send
const int rx_queue_size = 20;

static int year = 2020;
static byte month = 11;
static byte day = 11;
static byte hour = 12;
static byte minute = 10;
static byte second = 10;
static byte hundredths = 96;
static volatile boolean IMU_Present, GPS_Present, CAN_Present, SD_Present, Lidar_Present;
TaskHandle_t HLP;
TaskHandle_t MEAS;
TaskHandle_t WBS;
TaskHandle_t BLE;
TaskHandle_t MWD; //Main watchdog and house keeping thread


static enum SystemStates espState = ss_Idle;
unsigned long fix_age;
float msl = 0;


//SD_Rec SD_pool[500];  //buffer for the SD
//int SD_pool_itc = 0;  //item count
//int SD_pool_counter = 0; //current item



int kbytes, lines;

boolean dataflush_active = false;
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;
HardwareSerial ss(2);
boolean CanReady=false;
//SoftwareSerial ss(GPS_RX_PIN, GPS_TX_PIN);

int killcount = 0;
int wifi_rssi;
volatile boolean init_finished = false;
volatile boolean HasBeenSynced = false;
int VID = 1;
int DID = 1;
tm timeinfo;
time_t now;
double lastNTPtime;
unsigned long lastEntryTime;
int CANof, GPSof, IMUof;
const char* NTP_SERVER = "ch.pool.ntp.org";
const char* TZ_INFO    = "CET-0CEST-1,M3.5.0/02:00:00,M10.5.0/03:00:00";  //Greek time zone and daylight saving settings.


//Websockets variables
boolean WS_alive = false;
int roundtriptime;
boolean WS_sent = false;  //Contains the outcome of message send function through the websocket.

long    WS_TS = 0;        //Contains the TS of the message sent through the websocket.
long    WS_TSR = 0;
boolean awaitingWS = false;
uint16_t WS_CanRequest = 0;
uint16_t WS_CanCountMessages = 0;
boolean WS_Use_IMU;//code = M    .... these codes are sent via the WS to TOGGLE the printing (in SD and WS) of
boolean WS_Use_CAN;//code = N    .... the peripheral measurements. This way they can be isolated in real time.
boolean WS_Use_GPS;//code = G    .... The use of R (aka RTR) is used in conjuction with all these. R shuts all peripherals except CAN.
boolean WS_Use_TIM;//code = L
boolean MeasActive = true;
int     WS_inQ = 0;
boolean WS_lock = false;
int     WS_throttle = 0;
int sent = 0;
int recvd = 0;
int sdwr = 0;
long SDTotalCharWritten = 0;

int WS_timeout = 0;

char Filename[12] = "/orphan.txt";
boolean fileIsOpen = false;
File dataFile;
boolean Inbound = false;
int prog = 0;
long saTS = 0;
long saChars = 0;
boolean dataflushStarted = false;
void setup() {
  Serial.begin(115200);
  delay(250);
  Serial.setDebugOutput(true);
  // Serial.setDebugOutput(false);
  espState = ss_Idle; //initial system state

  ss.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); //setup the gps serial port
  CAN_cfg.speed = CAN_SPEED_250KBPS; // setup the can speed for Truck
  //CAN_cfg.speed = CAN_SPEED_500KBPS; // setup the can speed for Citroen
  CAN_cfg.tx_pin_id = GPIO_NUM_14;   // setup the can port
  CAN_cfg.rx_pin_id = GPIO_NUM_4;    // setup the can port
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));
  SPI.begin(0, 18, 19, 25);       //setup the sd spi port
  pinMode(SD_CS_PIN, OUTPUT);     //setup the sd spi port
  digitalWrite(SD_CS_PIN, HIGH);  //setup the can spi port
  pinMode(13, OUTPUT);            //open the led pin
  Wire.begin();
  WS_Use_IMU = true;
  WS_Use_CAN = true;
  WS_Use_GPS = true;
  WS_Use_TIM = true;
  delay(200);
  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  Serial.printf("Total PSRAM: %d\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d", ESP.getFreePsram());
  if ( ESP32Can.CANInit() == 0) {
    CanReady=true;
  }
  WiFiMulti.addAP("www.ntua.gr" , "Rur@lSurv3!ng"     );//<<base station
  WiFiMulti.addAP("fylakio fortion" , "fortion2017"     );//<<base station
  WiFiMulti.addAP("fortionMobile"   , "fortion.gr.2020" );//<<on board station Wifi AP

  //Serial.setDebugOutput(true);
  Serial.setDebugOutput(true);

  Serial.println();
  Serial.println();
  Serial.println();

  for (uint8_t t = 4; t > 0; t--) {
    Serial.printf("[SETUP] BOOT WAIT %d...\n", t);
    Serial.flush();

  }
  preferences.begin("xcx-pegasus", false);

  //WiFi.disconnect();
  int tries = 0;
  while (WiFiMulti.run() != WL_CONNECTED) {
    tries++;
    delay(500);
    Serial.println("Trying " + String(tries) + " / 10 times to connect.");
    if (tries == 10) {
      tries = 0;
      Serial.println("No wifi support. Relying on SD card");
      break;
    }
  }
  Serial.println(WiFi.localIP());
  SyncDateTime();
  delay(100);
  // server address, port and URL
  const char serva[] = "fortion.hopto.org";
  webSocket.begin(serva, 8989, "/");

  // event handler
  webSocket.onEvent(webSocketEvent);

  fmc = (int) preferences.getUInt(fileCounter, 0);
  fmc++;
  Serial.printf("Last saved file: /%06d.txt\n\r", fmc);
  sprintf(Filename, "/%06d.txt", fmc);
  // use HTTP Basic Authorization this is optional remove if not needed
  //webSocket.setAuthorization("user", "Password");
  //webSocket.disconnect();
  // try ever 5000 again if connection has failed
  xTaskCreatePinnedToCore(//
    &WBS_routine, /* Function to implement the task */
    "WBS", /* Name of the task */
    16386, /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &WBS,  /* Task handle. */
    1); /* Core where the task should run */
  webSocket.setReconnectInterval(2000);
  for (int sec = 0; sec < 20; sec++) {
    delay(1000);
    if (WS_alive) {
      break;
    }
  }
    
//  if (WS_alive) {
//    webSocket.sendTXT(":: Websocket connection established! Creating main threads...");
//  }


  xTaskCreatePinnedToCore(
    &MWD_routine, /* Function to implement the task */
    "MWD", /* Name of the task */
    32772,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &MWD,  /* Task handle. */
    0); /* Core where the task should run */

  xTaskCreatePinnedToCore(
    &HLP_routine, /* Function to implement the task */
    "HLP", /* Name of the task */
    8192,  /* Stack size in words */
    NULL,  /* Task input parameter */
    0,  /* Priority of the task */
    &HLP,  /* Task handle. */
    0); /* Core where the task should run */
  //
  //  xTaskCreatePinnedToCore(
  //    &BLE_routine, /* Function to implement the task */
  //    "BLE", /* Name of the task */
  //    4096,  /* Stack size in words */
  //    NULL,  /* Task input parameter */
  //    0,  /* Priority of the task */
  //    &BLE,  /* Task handle. */
  //    1); /* Core where the task should run */


  //
  //
  //  SerialBT.begin("FortionDH"); //Bluetooth device name
  //  Serial.println("The device started, now you can pair it with bluetooth!");

  delay(2000);

}


void loop() {
  delay(10000);
}
