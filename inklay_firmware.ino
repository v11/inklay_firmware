// Config
// 1. Board Config: Selected Board: ESP32 Dev Module
// 2. Partition Scheme: Minimal SPIFFS (1.9 MB APP with OTA/190KB SPIFFS)
// 3. Upload Speed: 115200
// 4. Select Serial Port

// Config
#define CLIENT_VERSION "0.0.10"

// #define CHANNEL_BETA
#define CHANNEL_PRODUCTION

// SPI
// #define DISPLAY_75INCH_2COLOR
// #define DISPLAY_75INCH_3COLOR
// #define DISPLAY_565INCH_7COLOR
#define DISPLAY_106INCH_3COLOR

// Parallel
// #define DISPLAY_6INCH_2COLOR

// Adafruit_GFX: Draw Libary
#include <Adafruit_GFX.h>

// Adafruit_GFX: Fonts
#include <Fonts/FreeMonoBold9pt7b.h>

// GXEPD2
#include <GxEPD2.h>
#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>

// NVS
#include <nvs.h>
#include "nvs_flash.h"

// JSON: Library: Include JSON Library https://arduinojson.org
#include <ArduinoJson.h>

// Wifi: Wifi Library
#include <WiFi.h>
#include <WiFiClientSecure.h>

// Bluetooth:
// The ESP Partion Sheme has to be set to Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)
#include <string> // This library is not necessary, but prevents intelisense errors
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// Time
#include "time.h"

// Update (OTA)
#include <Update.h>

// Include Config
// Config File contains links to the servers and sensitive date
// This file will be exluded in the public repo
#include "config.h";

// Local bitmap data
// Convert images: https://javl.github.io/image2cpp/
#include "bitmap.h"

// Version
const char *clientVersion = CLIENT_VERSION;

// Channel
#ifdef CHANNEL_BETA
const char *clientChannel = "beta";
const char *updateServerHost = BETAUPDATESERVERHOST;
const char *updateServerURL = BETAUPDATESERVERURL;
const char *updateServerBinFolder = BETAUPDATESERVERBINFOLDER;
const char *firebaseDatabaseHost = BETAFIREBSEDATABASEHOST;
const char *firebaseStorageHost = BETAFIREBASESTORAGEHOST;
const char *firebaseStorageURL = BETAFIREBASESTORAGEURL;
#elif defined(CHANNEL_PRODUCTION)
const char *clientChannel = "production";
const char *updateServerHost = PRODUPDATESERVERHOST;
const char *updateServerURL = PRODUPDATESERVERURL;
const char *updateServerBinFolder = PRODUPDATESERVERBINFOLDER;
const char *firebaseDatabaseHost = PRODFIREBSEDATABASEHOST;
const char *firebaseStorageHost = PRODFIREBASESTORAGEHOST;
const char *firebaseStorageURL = PRODFIREBASESTORAGEURL;
#endif

// Display Type
#ifdef DISPLAY_75INCH_2COLOR
const char *clientDisplayType = "7in5_2color";
#elif defined(DISPLAY_6INCH_2COLOR)
const char *clientDisplayType = "6in_2color";
#elif defined(DISPLAY_75INCH_3COLOR)
const char *clientDisplayType = "7in5_3color";
#elif defined(DISPLAY_565INCH_7COLOR)
const char *clientDisplayType = "5in65_7color";
#elif defined(DISPLAY_106INCH_3COLOR)
const char *clientDisplayType = "10in6_3color";
#endif

// Startup Mode Enum
enum StartupMode
{
  startUpBatteryLow,
  startUpWelcome,
  startUpWifi,
  startUpBluetooth
};
StartupMode startUpMode;

// WiFi: Credentials
const char *ssid;
const char *passphrase;
const int wifiTimeOut = 10000;
String macAddress = "";

// Bluetooth
// Generating UUIDs: https://www.uuidgenerator.net/
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Assign GPIO Pins
// Check the Pins of the board if they support Input / Output

const gpio_num_t pinLEDRed = GPIO_NUM_25;   // LED Red
const gpio_num_t pinLEDGreen = GPIO_NUM_32; // LED Green
const gpio_num_t pinLEDBlue = GPIO_NUM_33;  // LED Blue

// const gpio_num_t pinLED = GPIO_NUM_33; // LED
// const gpio_num_t pinRST = GPIO_NUM_32; // Reset Button
const gpio_num_t pinBLE = GPIO_NUM_2; // Digital Input Bluetooth Button

#ifdef DISPLAY_75INCH_2COLOR
// 7in5 2-Color
GxEPD2_BW<GxEPD2_750_T7, GxEPD2_750_T7::HEIGHT> display(GxEPD2_750_T7(/*CS=*/0, /*DC=*/17, /*RST=*/16, /*BUSY=*/4)); // GDEW075T7 800x480
bool with_color = false;
#elif defined(DISPLAY_75INCH_3COLOR)
// 7in5b-HD 3-Color GDEH075Z90 880x528
GxEPD2_3C<GxEPD2_750c_Z90, GxEPD2_750c_Z90::HEIGHT / 8> display(GxEPD2_750c_Z90(/*CS=5*/ 0, /*DC=*/17, /*RST=*/16, /*BUSY=*/4)); // GDEH075Z90 880x52
bool with_color = true;
#elif defined(DISPLAY_565INCH_7COLOR)
// 7-Color (Lolin D32)
GxEPD2_7C<GxEPD2_565c, GxEPD2_565c::HEIGHT / 4> display(GxEPD2_565c(/*CS=5*/ 0, /*DC=*/17, /*RST=*/16, /*BUSY=*/4)); // Waveshare 5.65" 7-color
bool with_color = true;
#elif defined(DISPLAY_6INCH_2COLOR)
// 6in 2-Color
GxEPD2_BW<GxEPD2_it60_1448x1072, GxEPD2_it60_1448x1072::HEIGHT / 4> display(GxEPD2_it60_1448x1072(/*CS=5*/ 0, /*DC=*/17, /*RST=*/16, /*BUSY=*/4));
bool with_color = false;
#elif defined(DISPLAY_106INCH_3COLOR)
// 10in6 3-Color GDEH116Z91 960x640
GxEPD2_3C<GxEPD2_750c_Z90, GxEPD2_750c_Z90::HEIGHT / 8> display(GxEPD2_750c_Z90(/*CS=5*/ 0, /*DC=*/17, /*RST=*/16, /*BUSY=*/4)); // GDEH075Z90 880x52
bool with_color = true;
#endif

// Time
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 3600;
const int daylightOffset_sec = 3600;
String currentTime;
String timeStamp;

// Battery Voltage
float batteryVoltage;

// Config Data from Firebase
bool configData = false;
int deepSleepInSeconds = -1;
const char *timezone;

// Stats Data to Firebase
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR int wifiError = 0;

// Deep Sleep
// (Use the ULL (unsigned long long) modifier for longer Sleep Times)
// Deep Sleep Calculator: https: //www.geekstips.com/battery-life-calculator-sleep-mode/
unsigned long long uS_TO_S_FACTOR = 1000000ULL;
enum DeepSleepMode
{
  forever,
  config,
  restart
};
DeepSleepMode deepSleepMode;
const int deepSleepInSecondsDefault = 300;

// Bitmap Loading Constants
#ifdef DISPLAY_75INCH_2COLOR
static const uint16_t input_buffer_pixels = 800;
static const uint16_t max_row_width = 800;
#elif defined(DISPLAY_75INCH_3COLOR)
static const uint16_t input_buffer_pixels = 880;
static const uint16_t max_row_width = 880;
#elif defined(DISPLAY_565INCH_7COLOR)
static const uint16_t input_buffer_pixels = 600;
static const uint16_t max_row_width = 600;
#elif defined(DISPLAY_6INCH_2COLOR)
static const uint16_t input_buffer_pixels = 1448;
static const uint16_t max_row_width = 1448;
#elif defined(DISPLAY_106INCH_3COLOR)
static const uint16_t input_buffer_pixels = 960;
static const uint16_t max_row_width = 960;
#endif

static const uint16_t max_palette_pixels = 256;       // for depth <= 8
uint8_t input_buffer[3 * input_buffer_pixels];        // up to depth 24
uint8_t output_row_mono_buffer[max_row_width / 8];    // buffer for at least one row of b/w bits
uint8_t output_row_color_buffer[max_row_width / 8];   // buffer for at least one row of color bits
uint8_t mono_palette_buffer[max_palette_pixels / 8];  // palette buffer for depth <= 8 b/w
uint8_t color_palette_buffer[max_palette_pixels / 8]; // palette buffer for depth <= 8 c/w
uint16_t rgb_palette_buffer[max_palette_pixels];      // palette buffer for depth <= 8 for buffered graphics, needed for 7-color display

void IRAM_ATTR reset()
{

  // This function is called when an interrupt occurs on a GPIO pin.
  // The IRAM_ATTR attribute declares that the compiled code will be placed in the Internal RAM (IRAM) of the ESP32.
  // This provides real-time execution.
  // Therefore, the function can be called at any time during the setup of the programm.

  Serial.println("");
  Serial.println("reset esp");

  // Pulldown voltage (reset button input) before restart. Otherwise this function get immediately called again. This results in a loop.
  // pinMode(pinRST, INPUT_PULLDOWN);
  pinMode(pinBLE, INPUT_PULLDOWN);

  // Restart the ESP
  ESP.restart();
}

void setInterrupt()
{

  // This function assigns an Interrupt to a GPIO Pin
  // Interrupts are asynchronus (like events).
  // Interrupts can be assigned to GPIO Pins and called in real-time.
  // Therefore, a reset can be called also during controller setup and not just in the loop of the programm.

  Serial.println("");
  Serial.println("set reset interrupt");

  // Define Pin as Input
  // pinMode(pinRST, INPUT_PULLDOWN);
  pinMode(pinBLE, INPUT_PULLDOWN);

  // Attach Interupt (reset-button) and assign reset function
  // attachInterrupt(pinRST, reset, HIGH);
  attachInterrupt(pinBLE, reset, HIGH);
}

void setup()
{

  // Begin Serial
  Serial.begin(115200);

  // Setup LED
  ledcSetup(0, 5000, 8);
  ledcSetup(1, 5000, 8);
  ledcSetup(2, 5000, 8);

  ledcAttachPin(pinLEDRed, 0);
  ledcAttachPin(pinLEDGreen, 1);
  ledcAttachPin(pinLEDBlue, 2);

  // Init Display
  initDisplay();

  // Set Button Interrupt
  setInterrupt();

  // Debug Client Version
  getClientVersion();

  // Boot Count
  ++bootCount;

  switch (getStartupMode())
  {
  case startUpBatteryLow:

    // Render Wifi Undefined Page
    renderDisplayBatteryLow();

    // Deep Sleep
    setDeepSleep(forever);

    break;

  case startUpWifi:

    // Wifi Mode
    startUpMode = startUpWifi;

    // Blink LED
    getLedBlink();

    // Delete NVS
    // Serial.println("");
    // Serial.println("delete values from nvs");
    // deleteNVS();

    // Get Value by Key from NVS in Namespace
    Serial.println("");
    Serial.println("get values from nvs");
    ssid = getValueByKeyFromNVS("wifi_ssid", "storage");
    passphrase = getValueByKeyFromNVS("wifi_pw", "storage");
    Serial.print("wifi ssid: ");
    Serial.println(ssid);
    Serial.print("wifi pw: ");
    Serial.println(passphrase);

    // Connect to WiFi Network
    getWifi();

    // Get Version
    checkVersion(updateServerHost, updateServerURL);

    // Get Config Data from Firebase
    getDeviceConfig(firebaseDatabaseHost);

    // Get the time
    getTime();

    // Save Stats to Firebase
    sendDeviceStats(firebaseDatabaseHost);

    // Save Logs to Firebase
    sendDeviceLogs(firebaseDatabaseHost);

    // Load Bitmap from URL (Host, URL, Position X, Position Y)
#ifdef DISPLAY_75INCH_2COLOR
    loadBitmapFromHTTPS(firebaseStorageHost, firebaseStorageURL, 0, 0, with_color);
#elif defined(DISPLAY_6INCH_2COLOR)
    loadBitmapFromHTTPS(firebaseStorageHost, firebaseStorageURL, 0, 0, with_color);
#elif defined(DISPLAY_75INCH_3COLOR)
    loadBitmapFromHTTPS(firebaseStorageHost, firebaseStorageURL, 0, 0, with_color);
#elif defined(DISPLAY_565INCH_7COLOR)
    showBitmapFrom_HTTPS_Buffered(firebaseStorageHost, firebaseStorageURL, 0, 0);
#elif defined(DISPLAY_106INCH_3COLOR)
    loadBitmapFromHTTPS(firebaseStorageHost, firebaseStorageURL, 0, 0, with_color);
#endif

    // Deep Sleep
    setDeepSleep(config);

    break;

  case startUpWelcome:

    // Get Device Mac Address
    Serial.print("mac-address: ");
    Serial.println(WiFi.macAddress());

    // Blink LED
    getLedBlink();

    // Render Welcome Screen
    renderDisplayWelcome();

    // Deep Sleep
    setDeepSleep(forever);

    break;
  case startUpBluetooth:

    // Set WiFi to station mode and disconnect from an AP if it was previously connected
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Bluetooth Mode
    startUpMode = startUpBluetooth;

    // Activate the LED
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 255);

    // Start a Bluetooth Service to recieve data
    getBluetooth();

    // Render Debug Page
    renderDisplayBluetooth();

    break;

  default:
    break;
  }
}

void getClientVersion()
{

  Serial.println("");
  Serial.println("get client version");

  // Print Version
  Serial.print("client version: ");
  Serial.println(clientVersion);

  // Print Channel
  Serial.print("client channel: ");
  Serial.println(clientChannel);

  // Display Version
  Serial.print("display type: ");
  Serial.println(clientDisplayType);
}

enum StartupMode getStartupMode()
{

  // This function returns the startup mode (WiFi or Bluetooth).
  // To start Bluetooth Mode, the Bluetooth Button needs to be pressed during startup
  // Otherwise Wi-Fi Mode gets started

  Serial.println("");
  Serial.println("get startup mode");

  if (getBatteryPercentage() == 0)
  {
    // Battery Low Mode
    Serial.println("Battery Low mode");
    return startUpBatteryLow;
  }

  if (digitalRead(pinBLE) == HIGH)
  {
    // Bluetooth Mode
    Serial.println("bluetooth mode");
    return startUpBluetooth;
  }

  // Get Value by Key from NVS in Namespace
  const char *value = getValueByKeyFromNVS("wifi_ssid", "storage");

  if (strcmp(value, "undefined") == 0)
  {
    // Wifi Mode
    Serial.println("wifi undefined mode");
    return startUpWelcome;
  }
  else
  {
    // Wifi Mode
    Serial.println("wifi mode");
    return startUpWifi;
  }
}

void getLedBlink()
{
  // Check the Wake-Up Reason from Deep-Slep
  // And only activate LED when the User clicked the Reset-Button
  // The LED will not blink when the Controller wakes up from the Timer (to save battery).

  if (isWakeUpReasonTimer() == false)
  {
    ledcWrite(0, 135);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
    delay(50);
    ledcWrite(0, 135);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
    delay(50);
    ledcWrite(0, 135);
    ledcWrite(1, 255);
    ledcWrite(2, 0);
    delay(50);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
  }
}

class MyCallbacks : public BLECharacteristicCallbacks
{

  // This callback is listening for data which was send with a bluetooth client (iOS/Android App)
  // The data will be stored in the NVS

  void onWrite(BLECharacteristic *pCharacteristic)
  {

    Serial.println("");
    Serial.println("recieved data from bluetooth");

    std::string value = pCharacteristic->getValue();

    if (value.length() > 0)
    {
      Serial.print("value: ");
      for (int i = 0; i < value.length(); i++)
      {
        Serial.print(value[i]);
      }
      Serial.println();

      // JSON: ArduinoJson
      // JSON: Allocate the JSON document
      // JSON: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
      DynamicJsonDocument doc(96);

      // JSON: Deserialize
      deserializeJson(doc, value);

      // Get SSID and Password
      const char *ssid = doc["ssid"];
      const char *passphrase = doc["passphrase"];

      Serial.print("wifi ssid: ");
      Serial.println(ssid);
      Serial.print("wifi password: ");
      Serial.println(passphrase);

      // Save Key and Value to NVS in Namespace
      setKeyValueToNVS("wifi_ssid", ssid, "storage");
      setKeyValueToNVS("wifi_pw", passphrase, "storage");
    }

    reset();
  }

  //Added by Jitendra, to set ssid values in custom characteristic
  // This callback is setting wifi ssid list which is being listened with a bluetooth client (iOS App)
  void onRead(BLECharacteristic *pCharacteristic)
  {
    Serial.println("BLE Read Event");
    Scan_WiFi_Networks(pCharacteristic);
  }
};

void getBluetooth()
{

  // This function establishes a Bluetooth Service
  // Use the iOS App nRF Connect to connect and write utf-8 data
  // Example JSON: {"ssid": "wifi","passphrase":"123456"}

  Serial.println("");
  Serial.println("get bluetooth");

  // Init the Device
  BLEDevice::init("Inklay");

  // Create a Bluetooth Server
  BLEServer *pServer = BLEDevice::createServer();

  // Create a Service with the UUID
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Set up the Service as Read / Write
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_WRITE);

  // Set the Callback (gets called when data is recieved)
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Send data (example)
  // pCharacteristic->setValue("Hello World");

  // Start the service
  pService->start();

  // Start advertising (visibility for ble-scanners)
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
}

void Scan_WiFi_Networks(BLECharacteristic *pCharacteristic)
{
  char buf[50];
  char SSID_List[500];
  int len = 0;
  memset(buf, 0, sizeof(buf));
  memset(SSID_List, 0, sizeof(SSID_List));

  Serial.println("scan start");
  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  Serial.println("scan done");
  if (n == 0)
  {
    Serial.println("no networks found");
  }
  else
  {
    Serial.print(n);
    Serial.println(" networks found");
    len += sprintf(&SSID_List[len], "%s", "[");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      Serial.print(i + 1);
      Serial.print(": ");
      String SSID = WiFi.SSID(i);
      SSID.toCharArray(buf, SSID.length() + 1);
      len += sprintf(&SSID_List[len], "{\"ssid\":\"%s\"},", buf);
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(")");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      //  delay(10);
    }
    len += sprintf(&SSID_List[len - 1], "%s", "]");
    SSID_List[len] = 0;
  }
  Serial.println("");

  Serial.println("JSON DATA ------> ");
  Serial.println(SSID_List);
  pCharacteristic->setValue(SSID_List);
  Serial.println();
}

void initDisplay()
{

  // This function will initialize a GXEPD Display

  Serial.println("");
  Serial.println("init display");

  // Standard SPI pins, e.g. SCK(18), MISO(19), MOSI(23), SS(5)
  display.init(115200);
}

void *setKeyValueToNVS(const char *key, const char *val, char *storageNameSpace)
{

  // This function will save key value pairs to NVS

  Serial.println("");
  Serial.println("save key: " + String(key) + " with value: " + String(val) + " to nvs");

  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    // NVS partition was truncated and needs to be erased
    // Retry nvs_flash_init
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  // Open NVS Handle with Namespace
  Serial.println("opening nvs handle with namespace: " + String(storageNameSpace));
  nvs_handle my_handle;
  err = nvs_open(storageNameSpace, NVS_READWRITE, &my_handle);

  if (err != ESP_OK)
  {
    Serial.println("error opening nvs handle");
  }
  else
  {
    Serial.println("succesful opening nvs handle");

    // Write to NVS
    Serial.println("write to nvs");
    err = nvs_set_str(my_handle, key, val);

    if (err != ESP_OK)
    {
      Serial.println("write failed");
    }
    else
    {
      Serial.println("write done");
    }

    // Commit to NVS
    Serial.println("commit updates in nvs");
    err = nvs_commit(my_handle);

    if (err != ESP_OK)
    {
      Serial.println("commit failed");
    }
    else
    {
      Serial.println("commit done");
    }

    // Close NVS Handle
    nvs_close(my_handle);
  }
}

const char *getValueByKeyFromNVS(char *key, char *storageNameSpace)
{

  // This function can get values by a key from the NVS

  // Open NVS Handle with Namespace
  nvs_handle my_handle;
  esp_err_t err = nvs_open(storageNameSpace, NVS_READWRITE, &my_handle);

  if (err != ESP_OK)
  {
    Serial.println("error opening nvs handle");
    return "undefined";
  }
  else
  {

    // Read from NVS
    size_t required_size;
    err = nvs_get_str(my_handle, key, NULL, &required_size);

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
      return "undefined";
    }
    else
    {
      char *value = (char *)malloc(required_size);
      nvs_get_str(my_handle, key, value, &required_size);
      return value;
    }

    // Close NVS Handle
    nvs_close(my_handle);
  }
}

void deleteNVS()
{

  // This function will erase NVS data

  Serial.println("");
  Serial.println("erase nvs");

  esp_err_t err = nvs_flash_erase();

  if (err != ESP_OK)
  {
    Serial.println("Erased: Failed");
  }
  else
  {
    Serial.println("Erased: Done");
  }
}

void getWifi()
{

  // This function will establish a WiFi connection

  Serial.println("");
  Serial.println("get wifi");

  // Start Wifi
  WiFi.begin(ssid, passphrase);

  // Keep track of when we started our attempt to get a WiFi connection
  unsigned long startAttemptTime = millis();

  Serial.print("connecting to wifi ");

  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < wifiTimeOut)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");

  // Make sure that we're actually connected, otherwise go to deep sleep
  if (WiFi.status() != WL_CONNECTED)
  {
    wifiError++;
    Serial.println("connection failed: timeout");
    Serial.println("wifi error: " + String(wifiError));
    setError("Wi-Fi Error: Could not connect to " + String(getValueByKeyFromNVS("wifi_ssid", "storage")));
    return;
  }

  Serial.println("connected to the wifi network");

  // Get Device Mac Address
  Serial.print("mac-address: ");
  Serial.println(WiFi.macAddress());
  macAddress = WiFi.macAddress();
  macAddress.replace(":", "%3A");
}

void checkVersion(const char *host, const char *url)
{

  // This function checks the client firmware version with the server firmware version in the channel (beta/production)
  // A http connection is used. Therefore no Certificate is needed.
  // The server responds with version and file (1.0.0, inklay_production_1_0_0.bin)
  // If the server version is identical: Do nothing
  // If the sever version is highter: Start OTA update
  // If the sever version is lower: Start OTA Downgrade

  Serial.println("");
  Serial.println("check version with inklay update server");

  // Wifi Client (HTTP)
  WiFiClient client;

  // Connect to Host
  Serial.println("connect to server via port 80");

  // HTTPS Connection: Check if Secure Connection is successfull
  if (!client.connect(host, 80))
  {
    // Connection Failed
    Serial.println("connection failed");

    setError("Update Server: Connection failed.");
    return;
  }

  // HTTPS Connection: Request URL with
  Serial.print("requesting url: ");
  Serial.print(host);
  Serial.println(url);

  // Assign URL
  client.print("GET ");
  client.print(url);
  client.println(" HTTP/1.1");

  // Send the HTTP headers
  client.print("Host: ");
  client.println(host);
  client.println("User-Agent: ESP32");
  client.println("Connection: close");
  client.println();

  // HTTPS Connection: Client is connected. Read Headers
  // HTTPS Connection: Header is the first response we get from the Server
  // HTTPS Connection: Example: HTTP/1.1 200 OK: Standard response for successful HTTP requests.

  // Check for Server Timeout
  unsigned long timeout = millis();

  while (client.available() == 0)
  {
    if (millis() - timeout > 5000)
    {

      // Server not available
      Serial.println("update server is not responding");

      // Stop the client
      client.stop();

      setError("Update Server is not responding");

      return;
    }
  }

  // Check the HTTP Response

  while (client.available())
  {

    // read first line of header
    String line = client.readStringUntil('\n');

    // remove space, to check if the line is end of headers
    line.trim();

    // If the the line is empty, this is end of headers
    if (!line.length())
    {
      // Headers ended
      // Continue
      break;
    }

    if (line.startsWith("HTTP/1.1"))
    {
      if (!(line.indexOf("200") < 0))
      {
        Serial.println("http response: 200 ok");

        // Continue to read response
      }
      else
      {
        // Server not available
        Serial.println("update server is not available");

        // Stop the client
        client.stop();

        setError("Update Server is not available");

        return;
      }
    }
  }

  // HTTPS Connection: Once server sends all requested data it will disconnect, then once all received data are read, program will exit the while loop.

  // JSON Parse: ArduinoJson
  // JSON Parse: Allocate the JSON document
  // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
  StaticJsonDocument<192> doc;

  // JSON Parse: Parse JSON object and receive possible errors
  DeserializationError error = deserializeJson(doc, client);

  // Check for Deserialization Errors
  if (error)
  {
    Serial.print(("deserialization failed with error: "));
    Serial.println(error.c_str());

    setError("Update Server: Deserialization failed with error: " + String(error.c_str()));

    return;
  }
  else
  {
    Serial.println("serialization successful");
  }

  // JSON Parse: Assign the data
  const char *serverVersion = doc[clientChannel]["version"];
  const char *serverFilename = doc[clientChannel]["file"];

  Serial.print("received: server version: ");
  Serial.println(serverVersion);

  Serial.print("received: server file name: ");
  Serial.println(serverFilename);

  // Check if server version is identical, bigger or lower than the client version
  String X[3] = {"", "", ""};
  String Y[3] = {"", "", ""};

  int m = 0;
  for (int i = 0; serverVersion[i] != '\0'; i++)
  {
    if (serverVersion[i] == '.')
    {
      m++;
      continue;
    }
    X[m].concat(serverVersion[i]);
  }

  m = 0;
  for (int i = 0; clientVersion[i] != '\0'; i++)
  {
    if (clientVersion[i] == '.')
    {
      m++;
      continue;
    }
    Y[m].concat(clientVersion[i]);
  }

  if ((X[0].toInt() == Y[0].toInt()) && (X[1].toInt() == Y[1].toInt()) && (X[2].toInt() == Y[2].toInt()))
  {
    // identical
    Serial.print("server version identical - keep client version: ");
    Serial.println(clientVersion);
  }

  else if ((X[0].toInt() > Y[0].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() > Y[1].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() == Y[1].toInt() && X[2].toInt() > Y[2].toInt()))
  {
    // Bigger, Start OTA Upgrade
    Serial.print("server version higher - start ota update to ");
    Serial.print(clientChannel);
    Serial.print("/");
    Serial.print(clientDisplayType);
    Serial.print("/");
    Serial.println(serverFilename);
    execOTA(String(clientChannel), String(serverFilename), String(serverVersion));
  }

  else if ((X[0].toInt() < Y[0].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() < Y[1].toInt()) || (X[0].toInt() == Y[0].toInt() && X[1].toInt() == Y[1].toInt() && X[2].toInt() < Y[2].toInt()))
  {
    // Lower, Start OTA Downgrade
    Serial.print("server version lower - start ota downgrade to ");
    Serial.print(clientChannel);
    Serial.print("/");
    Serial.print(clientDisplayType);
    Serial.print("/");
    Serial.println(serverFilename);
    execOTA(String(clientChannel), String(serverFilename), String(serverVersion));
  }

  // close
  client.stop();
}

// Utility to extract header value from headers
String getHeaderValue(String header, String headerName)
{
  return header.substring(strlen(headerName.c_str()));
}

void execOTA(String serverPath, String filename, String serverVersion)
{

  // This function performs an over the air update (HTTP).
  // The binary on the server will be downloaded and installed

  // The url structure to the binary: [host]/[path]/[channel]/[displayType]/[filename]
  // Example XXX

  Serial.println("");
  Serial.println("exec ota");

  long contentLength = 0;
  bool isValidContentType = false;

  // Host
  const char *host = updateServerHost;

  // URL to the binary folder
  String path = updateServerBinFolder;

  // Activate the LED
  ledcWrite(0, 255);
  ledcWrite(1, 255);
  ledcWrite(2, 255);

  // Render Debug Page
  renderDisplayOTA(serverVersion);

  // Wifi Client (HTTP)
  WiFiClient client;

  Serial.println("connect to server via port 80");

  // Connect the update
  if (client.connect(host, 80))
  {

    Serial.print("requesting url: ");
    Serial.println(host + path + clientChannel + "/" + clientDisplayType + "/" + filename);

    // Get the contents of the bin file
    client.print(String("GET ") + path + clientChannel + "/" + clientDisplayType + "/" + filename + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();

    while (client.available() == 0)
    {
      if (millis() - timeout > 5000)
      {

        // Server not available
        Serial.println("update server timeout!");

        // Show error page
        setError("Update failed. Server not responding.");

        // Stop the client
        client.stop();
        return;
      }
    }

    while (client.available())
    {

      // read headers
      String line = client.readStringUntil('\n');

      // remove space, to check if the line is end of headers
      line.trim();

      // If the the line is empty, this is end of headers
      // break the while and feed the remaining "client" to the Update.writeStream();
      if (!line.length())
      {
        // Headers ended
        // Get the OTA started
        break;
      }

      // Check if the HTTP Response is 200 (OK)
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1"))
      {
        if (line.indexOf("200") < 0)
        {
          Serial.println("update failed: server not responding");

          // Show error page
          setError("Update failed. Server not responding.");

          break;
        }
      }

      // extract headers: Get content length
      if (line.startsWith("Content-Length: "))
      {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("got " + String(contentLength) + " bytes from server");
      }

      // extract headers: Get content type
      if (line.startsWith("Content-Type: "))
      {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("got " + contentType + " payload");
        if (contentType == "application/octet-stream")
        {
          isValidContentType = true;
        }
      }
    }
  }
  else
  {

    // Server not available
    Serial.println("connection to " + String(host) + " failed.");

    // Show error page
    setError("Update failed. Server not responding.");
  }

  // Check what is the contentLength and if content type is "application/octet-stream"
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType)
  {
    // Check if there is enough space to OTA update
    bool canBegin = Update.begin(contentLength);

    if (canBegin)
    {
      Serial.println("begin update. this may take 2-5 mins to complete. please wait.");

      // No activity would appear on the Serial monitor
      size_t written = Update.writeStream(client);

      if (written == contentLength)
      {
        Serial.println("update sucessfull: written : " + String(written));
      }
      else
      {
        Serial.println("update failed: written only : " + String(written) + "/" + String(contentLength));

        // Show error page
        setError("Update failed.");
      }

      if (Update.end())
      {

        if (Update.isFinished())
        {
          // Sucessfull Update
          Serial.println("update successfull: rebooting");
          reset();
        }
        else
        {
          Serial.println("update failed: something went wrong");
        }
      }
      else
      {
        Serial.println("update failed: error #: " + String(Update.getError()));
      }
    }
    else
    {

      // Not enough space to begin OTA
      Serial.println("Not enough space to begin OTA");

      // Show error page
      setError("Update failed. Not enough space to begin OTA");

      client.flush();
    }
  }
  else
  {
    Serial.println("there was no content in the response");

    // Show error page
    setError("Update failed. There was no content in the response");

    client.flush();
  }
}

void getDeviceConfig(const char *host)
{

  // This function loads the device config data from Firebase (Channel, Display Type, Deep Sleep in Seconds, GMT Time Zone Offset in Seconds)
  // If the channel version is different: A version check (checkVersion) will be performed again and later the OTA process will be started.
  // If the display type is different: A version check (checkVersion) will be performed again and later the OTA process will be started.

  Serial.println("");
  Serial.println("get device config from firebase");

  // Wifi Client for secure connection (HTTPS)
  WiFiClientSecure client;

  // Add Certificate
  client.setInsecure();

  // Connect to Host
  Serial.println("connect to server via port 443");

  // HTTPS Connection: Check if Secure Connection is successfull
  if (!client.connect(host, 443))
  {
    // Connection Failed
    char err_buf[100];

    if (client.lastError(err_buf, 100) < 0)
    {

      Serial.print("connection failed with error: ");
      Serial.println(err_buf);

      setError("Firebase Database: Connection failed with error: " + String(err_buf));
    }
    else
    {
      Serial.println("connection failed");

      setError("Firebase Database: Connection failed.");
    }

    return;
  }

  // HTTPS Connection: Request URL with
  String url = "/devices/" + macAddress + "/config.json";
  Serial.print("requesting url: ");
  Serial.println(host + url);

  // Assign URL
  client.println("GET " + url + " HTTP/1.1");

  // Send the HTTP headers
  client.print("Host: ");
  client.println(host);
  client.println("User-Agent: ESP32");
  client.println("Connection: close");
  client.println();

  // HTTPS Connection: Client is connected. Read Headers
  // HTTPS Connection: Header is the first response we get from the Server
  // HTTPS Connection: Example: HTTP/1.1 200 OK: Standard response for successful HTTP requests.

  // Check for Server Timeout
  unsigned long timeout = millis();

  while (client.available() == 0)
  {
    if (millis() - timeout > 5000)
    {

      // Server not available
      Serial.println("firebase database is not responding");

      // Stop the client
      client.stop();

      setError("Firebase Database is not responding");

      return;
    }
  }

  // Check the HTTP Response
  bool httpError = false;

  while (client.available())
  {

    // read first line of header
    String line = client.readStringUntil('\n');

    // remove space, to check if the line is end of headers
    line.trim();

    // If the the line is empty, this is end of headers
    if (!line.length())
    {
      // Headers ended
      // Continue
      break;
    }

    if (line.startsWith("HTTP/1.1"))
    {
      if (!(line.indexOf("200") < 0))
      {
        Serial.println("http response: 200 ok");

        httpError = false;

        // Continue to read response
      }
      else
      {
        Serial.println("http response: error");

        httpError = true;

        // Continue to read response
      }
    }
  }

  // HTTPS Connection: Once server sends all requested data it will disconnect, then once all received data are read, program will exit the while loop.

  // JSON Parse: ArduinoJson
  // JSON Parse: Allocate the JSON document
  // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
  DynamicJsonDocument doc(384);

  // JSON Parse: Parse JSON object and receive possible errors
  DeserializationError error = deserializeJson(doc, client, DeserializationOption::NestingLimit(2));

  // Check for Deserialization Errors
  if (error)
  {
    Serial.print(("deserialization failed with error: "));
    Serial.println(error.c_str());

    setError("Firebase Database: Deserialization failed with error: " + String(error.c_str()));

    return;
  }
  else
  {
    Serial.println("serialization successful");
  }

  // Check if the JSON Doc is empty
  if (doc.isNull())
  {

    Serial.println("this device has not been setup.");

    // Display not available
    renderDisplayNotAvailable();

    // Deep Sleep
    setDeepSleep(forever);

    return;
  }

  // Read error response
  if (httpError)
  {

    String error = doc["error"];
    Serial.println(error);

    setError("Firebase Database: " + error);
    return;
  }

  // JSON Parse: Get the data
  const char *serverChannel = doc["channel"];
  const char *configDisplayType = doc["displayType"];
  deepSleepInSeconds = doc["deepSleepInSeconds"];
  const char *timezoneTmp = doc["timezone"];

  // Assign Timezone
  timezone = timezoneTmp;

  // JSON Parse: Print the result
  Serial.print("received: channel: ");
  Serial.println(serverChannel);
  Serial.print("received: display type: ");
  Serial.println(configDisplayType);
  Serial.print("received: timezone: ");
  Serial.println(timezoneTmp);
  Serial.print("received: seep sleep in seconds: ");
  Serial.println(deepSleepInSeconds);

  configData = true;

  // close
  client.stop();

  // Check Channel
  // Start OTA update, if config channel and client channel are not identical

  Serial.println("check client channel with server channel");

  if (strcmp(serverChannel, clientChannel) != 0)
  {

    Serial.print("server channel (");
    Serial.print(serverChannel);
    Serial.print(") is different to client channel (");
    Serial.print(clientChannel);
    Serial.println(")");

    // Force Update
    clientVersion = "0";

    // Overwite the client channel
    clientChannel = serverChannel;

    // Check if display type also has changed
    if (strcmp(configDisplayType, clientDisplayType) != 0)
    {
      Serial.print("server display type (");
      Serial.print(configDisplayType);
      Serial.print(") is different to client display type (");
      Serial.print(clientDisplayType);
      Serial.println(")");

      // Overwite the client display type
      clientDisplayType = configDisplayType;
    }

    // Check Version again
    checkVersion(updateServerHost, updateServerURL);
  }

  // Identical channel
  Serial.print("server channel identical - keep client channel: ");
  Serial.println(clientChannel);

  Serial.println("check display type with server display type");

  if (strcmp(configDisplayType, clientDisplayType) != 0)
  {

    Serial.print("server display type (");
    Serial.print(configDisplayType);
    Serial.print(") is different to client display type (");
    Serial.print(clientDisplayType);
    Serial.println(")");

    // Force Update
    clientVersion = "0";

    // Overwite the client display type
    clientDisplayType = configDisplayType;

    // Check Version again
    checkVersion(updateServerHost, updateServerURL);
  }

  // Identical display type
  Serial.print("server display type identical - keep client display type: ");
  Serial.println(clientDisplayType);
}

void getTime()
{

  // This function gets the current time for the device
  // The timezone is stored in Firebase and recievecv by getDeviceConfig
  // Example timezone: CET-1CEST,M3.5.0,M10.5.0/3
  // Complete list: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv

  Serial.println("");
  Serial.println("get time");

  // Check if Timezone is available from Display Config (Firebase)
  if (configData)
  {
    // Configure Time
    configTime(0, 0, ntpServer);

    // Set with config timezone
    setenv("TZ", timezone, 1);
  }
  else
  {
    // Configure Time
    configTime(0, 0, ntpServer);

    // Set with default timezone
    setenv("TZ", "GMT0", 1);
  }

  // Create timeinfo
  struct tm timeinfo;

  // Check if time is available
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("failed to obtain time");
    setError("Get Time: Connection failed");
    return;
  }

  // Get Unix TimeStamp
  time_t now;
  time(&now);
  timeStamp = now;

  Serial.println(timeStamp);

  // Sunday
  char timeWeekDay[10];
  strftime(timeWeekDay, 10, "%A", &timeinfo);

  // December
  char timeMonth[10];
  strftime(timeMonth, 10, "%B", &timeinfo);

  // 27
  char timeMonthDay[3];
  strftime(timeMonthDay, 3, "%d", &timeinfo);

  // 2020
  char timeYear[5];
  strftime(timeYear, 5, "%Y", &timeinfo);

  // 21
  char timeHour[3];
  strftime(timeHour, 3, "%H", &timeinfo);

  // 47
  char timeMinutes[3];
  strftime(timeMinutes, 3, "%M", &timeinfo);

  // 44
  char timeSeconds[3];
  strftime(timeSeconds, 3, "%S", &timeinfo);

  // Sunday, December 27 2020 21 : 47 : 44
  currentTime = String(timeWeekDay) + ", " + String(timeMonthDay) + ". " + String(timeMonth) + " " + String(timeYear) + " " + String(timeHour) + ":" + String(timeMinutes) + ":" + String(timeSeconds);

  Serial.println(currentTime);
}

float getBatteryVoltage()
{

  // This function gets the battery voltage (Lolin D32)

  return analogRead(35) / 4096.0 * 7.23;
}

float getBatteryPercentage()
{

  // This function calculates the remaining battery percentage
  // https://github.com/G6EJD/LiPo_Battery_Capacity_Estimator/blob/master/ReadBatteryCapacity_LiFePO4.ino

  uint8_t percentage = 100;

  // Get Voltage
  float voltage = getBatteryVoltage();

  // Calculate Percentage
  percentage = 4984.6172 * pow(voltage, 3) - 48287.7528 * pow(voltage, 2) + 155490.3329 * voltage - 166375.8607;

  if (voltage > 3.4)
  {
    percentage = 100;
  }
  else if (voltage <= 3.00)
  {
    percentage = 0;
  }

  // Return battery percentage
  return percentage;
}

void sendDeviceStats(const char *host)
{

  // This function sends device stats to Firebase (battery voltage, battery percentage)
  // Note: For sucessful PUT / POST request it is necessary to listen and wait for the response of the Server

  Serial.println("");
  Serial.println("send device stats");

  // Wifi Client for secure connection (HTTPS)
  WiFiClientSecure client;

  // Add Certificate to client
  client.setInsecure();

  // Connect to Host
  Serial.println("connect to server via port 443");

  // HTTPS Connection: Check if Secure Connection is successfull
  if (!client.connect(host, 443))
  {
    // Connection Failed
    char err_buf[100];

    if (client.lastError(err_buf, 100) < 0)
    {

      Serial.print("connection failed with error: ");
      Serial.println(err_buf);

      setError("Firebase Database: Connection failed with error: " + String(err_buf));
    }
    else
    {
      Serial.println("connection failed");

      setError("Firebase Database: Connection failed.");
    }

    return;
  }

  // JSON Parse: ArduinoJson
  // JSON Parse: Allocate the JSON document
  // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
  DynamicJsonDocument doc(256);

  float batteryPercentage = getBatteryPercentage();
  float batteryVoltage = getBatteryVoltage();

  // Asign the data
  doc["channel"] = clientChannel;
  doc["version"] = clientVersion;
  doc["displayType"] = clientDisplayType;
  doc["bootCount"] = String(bootCount);
  doc["lastUpdate"] = currentTime;
  doc["wifiError"] = wifiError;
  doc["batteryPercentage"] = batteryPercentage;
  doc["batteryVoltage"] = batteryVoltage;

  // HTTPS Connection: Request URL with
  String url = "/devices/" + macAddress + "/stats.json";
  Serial.print("requesting url: ");
  Serial.println(host + url);

  Serial.print("sent: client channel: ");
  Serial.println(clientChannel);
  Serial.print("sent: client version: ");
  Serial.println(clientVersion);
  Serial.print("sent: client display type: ");
  Serial.println(clientDisplayType);
  Serial.print("sent: boot count: ");
  Serial.println(bootCount);
  Serial.print("sent: last update: ");
  Serial.println(currentTime);
  Serial.print("sent: wifi error: ");
  Serial.println(wifiError);
  Serial.print("sent: battery percentage: ");
  Serial.println(batteryPercentage);
  Serial.print("sent: battery voltage: ");
  Serial.println(batteryVoltage);

  // Assign URL
  client.println("PUT " + url + " HTTP/1.1");

  // Send the HTTP headers
  client.print("Host: ");
  client.println(host);
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(measureJson(doc));
  client.println("Content-Type: application/json");
  client.println();

  // Send JSON document in body
  serializeJson(doc, client);

  // Wait for response
  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }

  // Read response from Server
  // It seems that the server cancels the requests if the client prematurely closes the connection.
  // This explains why you need to wait for the complete response before calling client.stop().
  while (client.available())
  {
    char c = client.read();
    // Serial.write(c);
  }
  // Serial.println("");

  client.stop();
}

void sendDeviceLogs(const char *host)
{
  // This function sends device logs to Firebase
  // Note: For sucessful PUT / POST request it is necessary to listen and wait for the response of the Server

  Serial.println("");
  Serial.println("send device logs");

  // Wifi Client for secure connection (HTTPS)
  WiFiClientSecure client;

  // Add Certificate to client
  client.setInsecure();

  // Connect to Host
  Serial.println("connect to server via port 443");

  // HTTPS Connection: Check if Secure Connection is successfull
  if (!client.connect(host, 443))
  {
    // Connection Failed
    char err_buf[100];

    if (client.lastError(err_buf, 100) < 0)
    {

      Serial.print("connection failed with error: ");
      Serial.println(err_buf);

      setError("Firebase Database: Connection failed with error: " + String(err_buf));
    }
    else
    {
      Serial.println("connection failed");

      setError("Firebase Database: Connection failed.");
    }

    return;
  }

  // JSON Parse: ArduinoJson
  // JSON Parse: Allocate the JSON document
  // JSON Parse: Use https://arduinojson.org/v6/assistant/ to calculate the capacity
  DynamicJsonDocument doc(256);

  float batteryPercentage = getBatteryPercentage();
  float batteryVoltage = getBatteryVoltage();

  // Asign the data
  doc["timeStamp"] = timeStamp;
  doc["batteryVoltage"] = batteryVoltage;

  // HTTPS Connection: Request URL with
  String url = "/devices/" + macAddress + "/logs.json";
  Serial.print("requesting url: ");
  Serial.println(host + url);

  Serial.print("sent unix time stamp: ");
  Serial.println(timeStamp);
  Serial.print("sent: battery voltage: ");
  Serial.println(batteryVoltage);

  // Assign URL
  client.println("POST " + url + " HTTP/1.1");

  // Send the HTTP headers
  client.print("Host: ");
  client.println(host);
  client.println("Connection: close");
  client.print("Content-Length: ");
  client.println(measureJson(doc));
  client.println("Content-Type: application/json");
  client.println();

  // Send JSON document in body
  serializeJson(doc, client);

  // Wait for response
  while (client.connected())
  {
    String line = client.readStringUntil('\n');
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }

  // Read response from Server
  // It seems that the server cancels the requests if the client prematurely closes the connection.
  // This explains why you need to wait for the complete response before calling client.stop().
  while (client.available())
  {
    char c = client.read();
    // Serial.write(c);
  }
  // Serial.println("");

  client.stop();
}

void setDeepSleep(DeepSleepMode deepSleepMode)
{

  // This function puts the device into deep-sleep with low power consumption

  Serial.println("");
  Serial.println("init deep sleep");

  // Disconnect wifi before deep-sleep (prevents connecting errors)
  WiFi.disconnect();

  // Defince pin as wakeup source
  // Only RTC IO can be used as a source for external wake source (pins: 0, 2, 4, 12-15, 25-27, 32-39)
  // esp_sleep_enable_ext0_wakeup(pinRST, 1); //1 = High, 0 = Low
  esp_sleep_enable_ext0_wakeup(pinBLE, 1); //1 = High, 0 = Low

  switch (deepSleepMode)
  {
  case forever:

    Serial.println("deep sleep mode: forever");

    // Do not provide wake up source: esp will sleep forever
    Serial.println("set deep sleep forerver");

    break;
  case config:

    Serial.println("deep sleep mode: config");

    // Check if the value could be loaded from Firebase
    if (deepSleepInSeconds < 0)
    {
      // Set the Timer to a default value of 10 Seconds
      deepSleepInSeconds = deepSleepInSecondsDefault;
    }

    // Set wake up source: calculate deep sleep in micro seconds
    esp_sleep_enable_timer_wakeup(deepSleepInSeconds * uS_TO_S_FACTOR);
    Serial.println("set deep sleep for " + String(deepSleepInSeconds) + " seconds!");

    break;

  case restart:

    Serial.println("deep sleep mode: restart");

    // Set the Timer to a default value of 10 Seconds
    deepSleepInSeconds = deepSleepInSecondsDefault;

    // Set wake up source: calculate deep sleep in micro seconds
    esp_sleep_enable_timer_wakeup(deepSleepInSeconds * uS_TO_S_FACTOR);
    Serial.println("set deep sleep for " + String(deepSleepInSeconds) + " seconds!");

    break;

  default:
    break;
  }

  // Start Deep Sleep
  Serial.flush();
  esp_deep_sleep_start();
}

bool isWakeUpReasonTimer()
{

  // This function returns the reason for the wake up from deep sleep.
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("wakeup caused by external signal using rtc_io");
    return false;
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("wakeup caused by external signal using rtc_cntl");
    return false;
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("wakeup caused by timer");
    return true;
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("wakeup caused by touchpad");
    return false;
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("wakeup caused by ulp program");
    return false;
    break;
  default:
    Serial.printf("wakeup was not caused by deep sleep: %d\n", wakeup_reason);
    return false;
    break;
  }
}

void setError(String msg)
{

  // Print Error Message
  renderDisplayError(msg);

  // Put the device into temporary Deep-Sleep
  setDeepSleep(restart);
}

void loadBitmapFromHTTPS(const char *host, const char *url, int16_t posX, int16_t posY, bool with_color)
{

  Serial.println("");
  Serial.println("load bitmap from firebase storage");

  // Client Connection Flag
  bool connection_ok = false;

  // Check if Bitmap Data is valid for drawing
  bool valid = false;

  // Microsoft bitmaps are usually stored from bottom to top.
  // When the height in the bitmap header is negative then it means that the image is stored
  // from top to bottom, and its real height is abs(height specified in the header).
  bool flip = true;

  uint32_t startTime = millis();

  // Setup Wifi Client for secure connection (HTTPS)
  WiFiClientSecure client;

  // Add Certificate
  client.setInsecure();

  // Connect to Host
  Serial.println("connect to server via port 443");

  // HTTPS Connection: Check if Secure Connection is successfull
  if (!client.connect(host, 443))
  {
    // Connection Failed
    char err_buf[100];

    if (client.lastError(err_buf, 100) < 0)
    {

      Serial.print("connection failed with error: ");
      Serial.println(err_buf);

      setError("Firebase Storage: Connection failed with error: " + String(err_buf));
    }
    else
    {
      Serial.println("connection failed");

      setError("Firebase Storage: Connection failed.");
    }

    return;
  }

  // Successfull Connection
  Serial.print("requesting url: ");
  Serial.println("https://" + String(host) + url + "/o/devices%2F" + macAddress + "%2Fimage.bmp?alt=media");

  // Send HTTP Request
  client.print(String("GET ") + "https://" + host + url + "/o/devices%2F" + macAddress + "%2Fimage.bmp?alt=media" + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "User-Agent: ESP32\r\n" + "Connection: close\r\n\r\n");

  while (client.connected())
  {
    // Read header from the stream line per line
    // The header contains info about Content-Type (image/bmp), Content-Length (in bytes) and more.
    String line = client.readStringUntil('\n');

    if (!connection_ok)
    {
      // Set connection_ok flag if response is 200 OK
      connection_ok = line.startsWith("HTTP/1.1 200 OK");
    }

    // Stop reading headers
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }

  // Stop execution when connection error
  if (!connection_ok)
  {
    setError("Could not download Image from Firebase Storage.");
    return;
  }

  // Parse BMP header
  // Serial.println("Debug Read16: " + read16(client));
  // The header field used to identify the BMP and DIB file is 0x42 0x4D
  // https://en.wikipedia.org/wiki/BMP_file_format
  if (read16(client) == 0x4D42) // BMP signature
  {

    uint32_t fileSize = read32(client);     // BMP Header: Size of the BMP file (54 bytes header + 16 bytes data)
    uint32_t creatorBytes = read32(client); // BMP Header: Application specific
    uint32_t imageOffset = read32(client);  // BMP Header: Offset where the pixel array (bitmap data) can be found
    uint32_t headerSize = read32(client);   // DIB Header: Number of bytes in the DIB header (from this point)
    uint32_t width = read32(client);        // DIB Header: Width of the bitmap in pixels
    uint32_t height = read32(client);       // DIB Header: Height of the bitmap in pixels. Positive for bottom to top pixel order.
    uint16_t planes = read16(client);       // DIB Header: Number of color planes being used
    uint16_t depth = read16(client);        // DIB Header: the number of bits per pixel, which is the color depth of the image. Typical values are 1, 4, 8, 16, 24 and 32.
    uint32_t format = read32(client);       // DIB Header: Compression method, Most common is 0 (BI_RGB)
    uint32_t bytes_read = 7 * 4 + 3 * 2;    // read so far

    // Only uncompressed (format == 0) is handled
    if ((planes == 1) && ((format == 0) || (format == 3)))
    {
      Serial.print("file size: ");
      Serial.println(fileSize);
      Serial.print("planes: ");
      Serial.println(planes);
      Serial.print("image offset: ");
      Serial.println(imageOffset);
      Serial.print("header size: ");
      Serial.println(headerSize);
      Serial.print("image size: ");
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      Serial.print('planes');
      Serial.println(planes);
      Serial.print("bit depth: ");
      Serial.println(depth);
      Serial.print("compression method: ");
      Serial.println(format);

      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;

      if (depth < 8)
      {
        rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;
      }

      // Microsoft bitmaps are usually stored from bottom to top.
      // When the height in the bitmap header is negative then it means that the image is stored
      // from top to bottom, and its real height is abs(height specified in the header).
      if (height < 0)
      {
        height = -height;
        flip = false;
      }

      // Get the position
      int16_t x = posX;
      int16_t y = posY;

      uint16_t w = width;
      uint16_t h = height;

      // Crop the image
      if ((x + w - 1) >= display.epd2.WIDTH)
      {
        w = display.epd2.WIDTH - x;
      }
      if ((y + h - 1) >= display.epd2.HEIGHT)
      {
        h = display.epd2.HEIGHT - y;
      }

      // Width needs to be smaller than max-row width to be valid
      if (w <= max_row_width)
      {
        valid = true;

        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue;

        bool whitish, colored;

        if (depth == 1)
        {
          with_color = false;
        }

        if (depth <= 8)
        {
          if (depth < 8)
          {
            bitmask >>= depth;
          }

          // Skip additional DIB Header Data (Print resolution of the image, Number of colors in the palette)
          //bytes_read += skip(client, 54 - bytes_read); //palette is always @ 54
          bytes_read += skip(client, imageOffset - (4 << depth) - bytes_read); // 54 for regular, diff for colorsimportant

          // Loop is running 2-times with 1-depth images
          // Create Full Buffer
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {

            blue = client.read();
            green = client.read();
            red = client.read();
            client.read();
            bytes_read += 4;
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?

            if (0 == pn % 8)
            {
              mono_palette_buffer[pn / 8] = 0;
            }
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            if (0 == pn % 8)
            {
              color_palette_buffer[pn / 8] = 0;
            }
            color_palette_buffer[pn / 8] |= colored << pn % 8;
          }
        }

        // Clear the Screen first
        display.clearScreen();

        // ??
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        // Serial.print("skip ");
        // Serial.println(rowPosition - bytes_read);

        // Skip additional DIB Header Data (Print resolution of the image, Number of colors in the palette)
        bytes_read += skip(client, rowPosition - bytes_read);

        // Start Drawing Line per Line (479 times if height is 480)
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {

          // Check connection
          if (!connection_ok || !(client.connected() || client.available()))
          {
            break;
          }

          // yield() to avoid WDT
          delay(1);
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0;           // for depth <= 8
          uint8_t in_bits = 0;           // for depth <= 8
          uint8_t out_byte = 0xFF;       // white (for w%8!=0 border)
          uint8_t out_color_byte = 0xFF; // white (for w%8!=0 border)
          uint32_t out_idx = 0;

          // Start Drawing Pixel per Pixel for one Line
          for (uint16_t col = 0; col < w; col++)
          {
            yield();

            // Check connection
            if (!connection_ok || !(client.connected() || client.available()))
            {
              break;
            }

            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              uint32_t get = in_remain > sizeof(input_buffer) ? sizeof(input_buffer) : in_remain;
              uint32_t got = read(client, input_buffer, get);
              while ((got < get) && connection_ok)
              {
                Serial.print("got ");
                Serial.print(got);
                Serial.print(" < ");
                Serial.print(get);
                Serial.print(" @ ");
                Serial.println(bytes_read);
                // if ((get - got) > client.available())
                //   delay(200); // does improve? yes, if >= 200
                uint32_t gotmore = read(client, input_buffer + got, get - got);
                got += gotmore;
                connection_ok = gotmore > 0;
              }
              in_bytes = got;
              in_remain -= got;
              bytes_read += got;
            }
            if (!connection_ok)
            {
              Serial.print("error: got no more after ");
              Serial.print(bytes_read);
              Serial.println(" bytes read!");
              break;
            }
            switch (depth)
            {
            case 24:
              // Serial.println("Lukas: Switch Case 24");
              blue = input_buffer[in_idx++];
              green = input_buffer[in_idx++];
              red = input_buffer[in_idx++];
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
              break;
            case 16:
            {
              uint8_t lsb = input_buffer[in_idx++];
              uint8_t msb = input_buffer[in_idx++];
              if (format == 0) // 555
              {
                blue = (lsb & 0x1F) << 3;
                green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                red = (msb & 0x7C) << 1;
              }
              else // 565
              {
                blue = (lsb & 0x1F) << 3;
                green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                red = (msb & 0xF8);
              }
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
            }
            break;
            case 1:
            case 4:
            case 8:
            {
              if (0 == in_bits)
              {
                in_byte = input_buffer[in_idx++];
                in_bits = 8;
              }
              uint16_t pn = (in_byte >> bitshift) & bitmask;
              whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
              colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
              in_byte <<= depth;
              in_bits -= depth;
            }
            break;
            }
            if (whitish)
            {
              // keep white
            }
            else if (colored && with_color)
            {
              // Serial.println("REEED");
              out_color_byte &= ~(0x80 >> col % 8); // colored
            }
            else
            {
              out_byte &= ~(0x80 >> col % 8); // black
            }
            if ((7 == col % 8) || (col == w - 1)) // write that last byte! (for w%8!=0 border)
            {
              output_row_color_buffer[out_idx] = out_color_byte;
              output_row_mono_buffer[out_idx++] = out_byte;
              out_byte = 0xFF;       // white (for w%8!=0 border)
              out_color_byte = 0xFF; // white (for w%8!=0 border)
            }
          } // end pixel
          int16_t yrow = y + (flip ? h - row - 1 : row);
          // display.writeImage(output_row_mono_buffer, output_row_color_buffer, x, yrow, w, 1);
          display.writeImage(output_row_mono_buffer, output_row_color_buffer, x, yrow, w, 1);
        } // end line
        Serial.print("downloaded in ");
        Serial.print(millis() - startTime);
        Serial.println(" ms");
        Serial.print("bytes read ");
        Serial.println(bytes_read);

        // Refresh the Screen
        display.refresh();
      }
    }

    if (!valid)
    {
      Serial.println("bitmap format not handled.");
    }
  }
}

void showBitmapFrom_HTTPS_Buffered(const char *host, const char *url, int16_t x, int16_t y)
{
  Serial.println();
  Serial.print("downloading file \"");
  Serial.print(url);
  Serial.println("\"");
  display.setFullWindow();
  display.firstPage();
  do
  {
    drawBitmapFrom_HTTPS_ToBuffer(host, url, x, y, with_color);
  } while (display.nextPage());
}

void drawBitmapFrom_HTTPS_ToBuffer(const char *host, const char *url, int16_t x, int16_t y, bool with_color)
{
  // Client Connection Flag
  bool connection_ok = false;

  // Check if Bitmap Data is valid for drawing
  bool valid = false;

  // Microsoft bitmaps are usually stored from bottom to top.
  // When the height in the bitmap header is negative then it means that the image is stored
  // from top to bottom, and its real height is abs(height specified in the header).
  bool flip = true;

  bool has_multicolors = display.epd2.panel == GxEPD2::ACeP565;

  uint32_t startTime = millis();

  // Setup Wifi Client for secure connection (HTTPS)
  WiFiClientSecure client;

  // Add Certificate
  client.setInsecure();

  // Connect to Host
  Serial.println("Connect to server via port 443");

  // Fill Screen
  display.fillScreen(GxEPD_WHITE);

  Serial.print("connecting to ");
  Serial.println(host);

  // HTTPS Connection: Check if Secure Connection is successfull
  if (!client.connect(host, 443))
  {
    // Connection Failed
    char err_buf[100];

    if (client.lastError(err_buf, 100) < 0)
    {

      Serial.print("connection failed with error: ");
      Serial.println(err_buf);

      setError("Firebase Storage: Connection failed with error: " + String(err_buf));
    }
    else
    {
      Serial.println("connection failed");

      setError("Firebase Storage: Connection failed.");
    }

    return;
  }

  // Successfull Connection
  Serial.println("Connected to server!");

  Serial.print("requesting url: ");
  Serial.println("https://" + String(host) + url + "/o/devices%2F" + macAddress + "%2Fimage.bmp?alt=media");

  // Send HTTP Request
  client.print(String("GET ") + "https://" + host + url + "/o/devices%2F" + macAddress + "%2Fimage.bmp?alt=media" + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "User-Agent: ESP32\r\n" + "Connection: close\r\n\r\n");

  // Wating
  Serial.println("Waiting for response ...");

  while (client.connected())
  {
    // Read header from the stream line per line
    // The header contains info about Content-Type (image/bmp), Content-Length (in bytes) and more.
    String line = client.readStringUntil('\n');

    if (!connection_ok)
    {
      // Set connection_ok flag if response is 200 OK
      connection_ok = line.startsWith("HTTP/1.1 200 OK");
      if (connection_ok)
      {
        Serial.println("valid connection: " + line);
      }
    }
    if (!connection_ok)
    {
      Serial.println("connection error: " + line);
    }

    // Stop reading headers
    if (line == "\r")
    {
      Serial.println("headers received");
      break;
    }
  }

  if (!connection_ok)
  {
    setError("Could not download Image from Firebase Storage.");
    return;
  }

  // Parse BMP header
  if (read16(client) == 0x4D42) // BMP signature
  {
    uint32_t fileSize = read32(client);
    uint32_t creatorBytes = read32(client);
    uint32_t imageOffset = read32(client); // Start of image data
    uint32_t headerSize = read32(client);
    uint32_t width = read32(client);
    uint32_t height = read32(client);
    uint16_t planes = read16(client);
    uint16_t depth = read16(client); // bits per pixel
    uint32_t format = read32(client);
    uint32_t bytes_read = 7 * 4 + 3 * 2;                   // read so far
    if ((planes == 1) && ((format == 0) || (format == 3))) // uncompressed is handled, 565 also
    {
      Serial.print("File size: ");
      Serial.println(fileSize);
      Serial.print("Image Offset: ");
      Serial.println(imageOffset);
      Serial.print("Header size: ");
      Serial.println(headerSize);
      Serial.print("Bit Depth: ");
      Serial.println(depth);
      Serial.print("Image size: ");
      Serial.print(width);
      Serial.print('x');
      Serial.println(height);
      // BMP rows are padded (if needed) to 4-byte boundary
      uint32_t rowSize = (width * depth / 8 + 3) & ~3;
      if (depth < 8)
        rowSize = ((width * depth + 8 - depth) / 8 + 3) & ~3;
      if (height < 0)
      {
        height = -height;
        flip = false;
      }
      uint16_t w = width;
      uint16_t h = height;
      if ((x + w - 1) >= display.width())
        w = display.width() - x;
      if ((y + h - 1) >= display.height())
        h = display.height() - y;
      //if (w <= max_row_width) // handle with direct drawing
      {
        valid = true;
        uint8_t bitmask = 0xFF;
        uint8_t bitshift = 8 - depth;
        uint16_t red, green, blue;
        bool whitish, colored;
        if (depth == 1)
          with_color = false;
        if (depth <= 8)
        {
          if (depth < 8)
            bitmask >>= depth;
          //bytes_read += skip(client, 54 - bytes_read); //palette is always @ 54
          bytes_read += skip(client, imageOffset - (4 << depth) - bytes_read); // 54 for regular, diff for colorsimportant
          for (uint16_t pn = 0; pn < (1 << depth); pn++)
          {
            blue = client.read();
            green = client.read();
            red = client.read();
            client.read();
            bytes_read += 4;
            whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
            colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
            if (0 == pn % 8)
              mono_palette_buffer[pn / 8] = 0;
            mono_palette_buffer[pn / 8] |= whitish << pn % 8;
            if (0 == pn % 8)
              color_palette_buffer[pn / 8] = 0;
            color_palette_buffer[pn / 8] |= colored << pn % 8;
            //Serial.print("0x00"); Serial.print(red, HEX); Serial.print(green, HEX); Serial.print(blue, HEX);
            //Serial.print(" : "); Serial.print(whitish); Serial.print(", "); Serial.println(colored);
            rgb_palette_buffer[pn] = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
          }
        }
        uint32_t rowPosition = flip ? imageOffset + (height - h) * rowSize : imageOffset;
        //Serial.print("skip "); Serial.println(rowPosition - bytes_read);
        bytes_read += skip(client, rowPosition - bytes_read);
        for (uint16_t row = 0; row < h; row++, rowPosition += rowSize) // for each line
        {
          if (!connection_ok || !(client.connected() || client.available()))
            break;
          delay(1); // yield() to avoid WDT
          uint32_t in_remain = rowSize;
          uint32_t in_idx = 0;
          uint32_t in_bytes = 0;
          uint8_t in_byte = 0; // for depth <= 8
          uint8_t in_bits = 0; // for depth <= 8
          uint16_t color = GxEPD_WHITE;
          for (uint16_t col = 0; col < w; col++) // for each pixel
          {
            yield();
            if (!connection_ok || !(client.connected() || client.available()))
              break;
            // Time to read more pixel data?
            if (in_idx >= in_bytes) // ok, exact match for 24bit also (size IS multiple of 3)
            {
              uint32_t get = in_remain > sizeof(input_buffer) ? sizeof(input_buffer) : in_remain;
              uint32_t got = read8n(client, input_buffer, get);
              while ((got < get) && connection_ok)
              {
                //Serial.print("got "); Serial.print(got); Serial.print(" < "); Serial.print(get); Serial.print(" @ "); Serial.println(bytes_read);
                uint32_t gotmore = read8n(client, input_buffer + got, get - got);
                got += gotmore;
                connection_ok = gotmore > 0;
              }
              in_bytes = got;
              in_remain -= got;
              bytes_read += got;
            }
            if (!connection_ok)
            {
              Serial.print("Error: got no more after ");
              Serial.print(bytes_read);
              Serial.println(" bytes read!");
              break;
            }
            switch (depth)
            {
            case 24:
              blue = input_buffer[in_idx++];
              green = input_buffer[in_idx++];
              red = input_buffer[in_idx++];
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
              color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
              break;
            case 16:
            {
              uint8_t lsb = input_buffer[in_idx++];
              uint8_t msb = input_buffer[in_idx++];
              if (format == 0) // 555
              {
                blue = (lsb & 0x1F) << 3;
                green = ((msb & 0x03) << 6) | ((lsb & 0xE0) >> 2);
                red = (msb & 0x7C) << 1;
                color = ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | ((blue & 0xF8) >> 3);
              }
              else // 565
              {
                blue = (lsb & 0x1F) << 3;
                green = ((msb & 0x07) << 5) | ((lsb & 0xE0) >> 3);
                red = (msb & 0xF8);
                color = (msb << 8) | lsb;
              }
              whitish = with_color ? ((red > 0x80) && (green > 0x80) && (blue > 0x80)) : ((red + green + blue) > 3 * 0x80); // whitish
              colored = (red > 0xF0) || ((green > 0xF0) && (blue > 0xF0));                                                  // reddish or yellowish?
            }
            break;
            case 1:
            case 4:
            case 8:
            {
              if (0 == in_bits)
              {
                in_byte = input_buffer[in_idx++];
                in_bits = 8;
              }
              uint16_t pn = (in_byte >> bitshift) & bitmask;
              whitish = mono_palette_buffer[pn / 8] & (0x1 << pn % 8);
              colored = color_palette_buffer[pn / 8] & (0x1 << pn % 8);
              in_byte <<= depth;
              in_bits -= depth;
              color = rgb_palette_buffer[pn];
            }
            break;
            }
            if (with_color && has_multicolors)
            {
              // keep color
            }
            else if (whitish)
            {
              color = GxEPD_WHITE;
            }
            else if (colored && with_color)
            {
              color = GxEPD_COLORED;
            }
            else
            {
              color = GxEPD_BLACK;
            }
            uint16_t yrow = y + (flip ? h - row - 1 : row);
            display.drawPixel(x + col, yrow, color);
          } // end pixel
        }   // end line
      }
      Serial.print("bytes read ");
      Serial.println(bytes_read);
    }
  }
  Serial.print("loaded in ");
  Serial.print(millis() - startTime);
  Serial.println(" ms");
  client.stop();
  if (!valid)
  {
    Serial.println("bitmap format not handled.");
  }
}

uint32_t skip(WiFiClientSecure &client, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while ((client.connected() || client.available()) && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = client.read();
      remain--;
    }
    else
      delay(1);
    if (millis() - start > 2000)
      break; // don't hang forever
  }
  return bytes - remain;
}

uint32_t read(WiFiClientSecure &client, uint8_t *buffer, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while ((client.connected() || client.available()) && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = client.read();
      *buffer++ = uint8_t(v);
      remain--;
    }
    else
      delay(1);
    if (millis() - start > 2000)
      break; // don't hang forever
  }
  return bytes - remain;
}

uint16_t read16(WiFiClientSecure &client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint16_t result;
  ((uint8_t *)&result)[0] = client.read(); // LSB
  ((uint8_t *)&result)[1] = client.read(); // MSB
  return result;
}

uint32_t read32(WiFiClientSecure &client)
{
  // BMP data is stored little-endian, same as Arduino.
  uint32_t result;
  ((uint8_t *)&result)[0] = client.read(); // LSB
  ((uint8_t *)&result)[1] = client.read();
  ((uint8_t *)&result)[2] = client.read();
  ((uint8_t *)&result)[3] = client.read(); // MSB
  return result;
}

uint32_t read8n(WiFiClientSecure &client, uint8_t *buffer, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while ((client.connected() || client.available()) && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = client.read();
      *buffer++ = uint8_t(v);
      remain--;
    }
    else
      delay(1);
    if (millis() - start > 2000)
      break; // don't hang forever
  }
  return bytes - remain;
}

uint32_t read8n(WiFiClient &client, uint8_t *buffer, int32_t bytes)
{
  int32_t remain = bytes;
  uint32_t start = millis();
  while ((client.connected() || client.available()) && (remain > 0))
  {
    if (client.available())
    {
      int16_t v = client.read();
      *buffer++ = uint8_t(v);
      remain--;
    }
    else
      delay(1);
    if (millis() - start > 2000)
      break; // don't hang forever
  }
  return bytes - remain;
}

void renderDisplayBluetooth()
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("Bluetooth: Inklay is ready to pair!");
    display.setCursor(40, 120);
    display.print("Channel: " + String(clientChannel));
    display.setCursor(40, 160);
    display.print("Version: " + String(clientVersion));
    display.setCursor(40, 200);
    display.print("Device ID: " + WiFi.macAddress());
    display.setCursor(40, 240);
    display.print("Wi-Fi: " + String(getValueByKeyFromNVS("wifi_ssid", "storage")));
  } while (display.nextPage());
}

void renderDisplayWifiError()
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("Could not connect to " + String(getValueByKeyFromNVS("wifi_ssid", "storage")) + ". Restart in " + String(deepSleepInSecondsDefault) + " Seconds");
  } while (display.nextPage());
}

void renderDisplayWelcome()
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("Welcome!");
    display.setCursor(40, 80);
    display.print("Download the Inklay App from the App-Store and get started!");
    display.drawInvertedBitmap(40, 120, qrcode, 100, 100, GxEPD_BLACK);
  } while (display.nextPage());
}

void renderDisplayNotAvailable()
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("This device has not yet been set up.");
    display.setCursor(40, 80);
    display.print("Go to inklay.vision11.ch, sign up and add this device.");
    display.setCursor(40, 120);
    display.print("Device ID: " + WiFi.macAddress());
  } while (display.nextPage());
}

void renderDisplayBatteryLow()
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("Battery Low. Please charge.");
  } while (display.nextPage());
}

void renderDisplayOTA(String filename)
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("Updating Inklay to " + String(clientChannel) + " " + filename);
    display.setCursor(40, 80);
    display.print("Please wait a few minutes.");
  } while (display.nextPage());
}

void renderDisplayError(String msg)
{
  display.setRotation(0);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.setFont(&FreeMonoBold9pt7b);
    display.setTextColor(GxEPD_BLACK);
    display.setCursor(40, 40);
    display.print("Error: " + msg);
    display.setCursor(40, 80);
    display.print("Restart in 5 Minutes");
  } while (display.nextPage());
}

void loop()
{
}
