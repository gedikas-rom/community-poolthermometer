#define ENABLE_GxEPD2_GFX 0
#include <esp_now.h>
#include <WiFi.h>
#include <OneWireESP32.h>
#include <GxEPD2_BW.h>
// #include <GxEPD2_3C.h>
#include <GxEPD2_GFX.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/TomThumb.h>
#include <imagedata.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Button.h>
#include <driver/rtc_io.h>
#include <Preferences.h>
#include <time.h>
#include "sensor_message.h"

#define VOLTAGE_PIN 5  // Battery Voltage Pin
#define BUTTON1_PIN GPIO_NUM_0  // Button 1 Pin
#define BUTTON2_PIN GPIO_NUM_1  // Button 2 Pin
#define BUTTON3_PIN GPIO_NUM_2  // Button 3 Pin
#define TEMP_PIN 22    // DS18B20 data line for air temperature
#define CS_PIN 23    // Display CS Pin
#define BUSY_PIN 16  // Display BUSY Pin
#define RST_PIN 17   // Display RST Pin
#define DC_PIN 20    // Display DC Pin

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */
#define BUTTON_PIN_BITMASK (1ULL << BUTTON1_PIN) | (1ULL << BUTTON2_PIN) | (1ULL << BUTTON3_PIN)
//| (1ULL << BUTTON3_PIN) // GPIO 0 bitmask for ext1

const char* firmware = "0.6.2";
Button btnLeft(BUTTON1_PIN);
Button btnMiddle(BUTTON2_PIN);
Button btnRight(BUTTON3_PIN);

// E-Paper config
// ESP32-C3 Supermini CS(SS)=7,SCL(SCK)=4,SDA(MOSI)=6,BUSY=3,RES(RST)=2,DC=1
// XIAO ESP32-C6 CS(SS)=7,SCL(SCK)=19,SDA(MOSI)=18,BUSY=3,RES(RST)=2,DC=1
// 2.9'' EPD Module
//GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ 7, /*DC=*/ 1, /*RES=*/ 2, /*BUSY=*/ 3)); // DEPG0290BS 128x296, SSD1680
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ CS_PIN, /*DC=*/ DC_PIN, /*RES=*/ RST_PIN, /*BUSY=*/ BUSY_PIN)); // DEPG0290BS 128x296, SSD1680
//GxEPD2_3C<GxEPD2_290_C90c, GxEPD2_290_C90c::HEIGHT> display(GxEPD2_290_C90c(/*CS=5*/ 7, /*DC=*/ 1, /*RES=*/ 2, /*BUSY=*/ 3)); // GDEM029C90 128x296, SSD1680

enum ValveState {
  UNDEFINED,
  OPENING,
  OPEN,
  CLOSING,
  CLOSED
};

enum Mode {
  AUTO,   // automatic mode
  ON,     // Solar heating on, pump full power
  OFF,          // Solar heating off, pump in eco mode (lowest level)
  CLEANING     // Solar heating off, pump full power for max cleaning
};

// ESP-NOW config
// Define data structure
typedef struct struct_message_receive {
  float averageTempWater;
  float averageTempAir;
  float targetTemp;
  Mode mode;
  ValveState currentValveState;
  int currentPumpState;
  tm lastUpdate; // Last update time
} struct_message_receive;

struct_message_receive myDataReceive;
sensor_message outMsg;
sensor_message inMsg;
volatile bool responseReceived = false;

OneWire32 ds(TEMP_PIN);
const uint8_t maxDevices = 2;  // max devices per bus
uint8_t foundDevices = 0;
uint64_t addr[maxDevices];
float currTemp[maxDevices];
bool bNoSleep = false; // Flag to prevent deep sleep
int batteryLevel;
float localTemperature;

// ESP-NOW peer discovery
static uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t bridgeMac[6] = {0};
static bool bridgeKnown = false;
static unsigned long lastDiscoverMs = 0;
static const unsigned long DISCOVER_INTERVAL_MS = 2000;
static const unsigned long DISCOVER_INTERVAL_MAX_MS = 30000;
static unsigned long discoverIntervalMs = DISCOVER_INTERVAL_MS;

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void updateDisplay_PoolControlValues();
void updateDisplay_ButtonHints();
void updateDisplay_LastUpdate();
void updateDisplay_Temperature();
void updateDisplay_BatteryState();
bool isValidTemperatureAir(float temp);
void measureTemperature();
void setPoolControlMode(String mode);
void setPoolControlMode(Mode mode);
void measureVoltage();
void getPoolControlValues();
void prepareDisplay();
void setSensorData();
void sendMessage(String id, String payload);
void btnTaskHandling(void *parameter);
void checkButtons();
void checkButtons(int wakeupBtnPin);
void startDeepSleep();

static void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  esp_err_t err = esp_now_add_peer(&peerInfo);
  if (err != ESP_OK) {
    Serial.println("Failed to add peer");
  }
}

static bool macEquals(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

static void storeBridgeMac(const uint8_t *mac) {
  Preferences prefs;
  if (!prefs.begin("bridge", false)) return;
  prefs.putBytes("mac", mac, 6);
  prefs.end();
}

static void setBridgeMac(const uint8_t *mac) {
  if (bridgeKnown && macEquals(bridgeMac, mac)) return;
  memcpy(bridgeMac, mac, 6);
  bridgeKnown = true;
  addPeer(bridgeMac);
  discoverIntervalMs = DISCOVER_INTERVAL_MS;
  storeBridgeMac(bridgeMac);
  Serial.printf("[NODE] Bridge MAC learned: %02X:%02X:%02X:%02X:%02X:%02X\n",
                bridgeMac[0], bridgeMac[1], bridgeMac[2],
                bridgeMac[3], bridgeMac[4], bridgeMac[5]);
}

static bool loadBridgeMac() {
  Preferences prefs;
  if (!prefs.begin("bridge", true)) return false;
  size_t len = prefs.getBytesLength("mac");
  if (len != 6) {
    prefs.end();
    return false;
  }
  uint8_t mac[6] = {0};
  prefs.getBytes("mac", mac, 6);
  prefs.end();
  setBridgeMac(mac);
  return true;
}

static void sendDiscover() {
  sensor_message msg = {};
  strncpy(msg.id, "bridge/discover", sizeof(msg.id) - 1);
  msg.payload[0] = '\0';
  addPeer(broadcastMac);
  esp_err_t result = esp_now_send(broadcastMac, (uint8_t *)&msg, sizeof(msg));
  lastDiscoverMs = millis();
  if (discoverIntervalMs < DISCOVER_INTERVAL_MAX_MS) {
    discoverIntervalMs = min(discoverIntervalMs * 2, DISCOVER_INTERVAL_MAX_MS);
  }
  Serial.printf("[NODE] Discover sent result=%s\n", result == ESP_OK ? "OK" : "ERROR");
}

static bool parseLastUpdateFromJson(const JsonVariantConst& v, tm& outTm) {
  if (v.is<long>()) {
    time_t ts = static_cast<time_t>(v.as<long>());
    localtime_r(&ts, &outTm);
    return true;
  }

  if (v.is<const char*>()) {
    const char* s = v.as<const char*>();
    int year, month, day, hour, minute, second;
    if (sscanf(s, "%d-%d-%d %d:%d:%d", &year, &month, &day, &hour, &minute, &second) == 6) {
      memset(&outTm, 0, sizeof(outTm));
      outTm.tm_year = year - 1900;
      outTm.tm_mon = month - 1;
      outTm.tm_mday = day;
      outTm.tm_hour = hour;
      outTm.tm_min = minute;
      outTm.tm_sec = second;
      return true;
    }
  }

  return false;
}

static void formatDateTimeGerman(const tm& value, char* out, size_t outSize) {
  static const char* monthsDe[] = {
    "Januar", "Februar", "März", "April", "Mai", "Juni",
    "Juli", "August", "September", "Oktober", "November", "Dezember"
  };

  const int mon = value.tm_mon;
  const char* month = (mon >= 0 && mon < 12) ? monthsDe[mon] : "???";
  snprintf(out, outSize, "%02d.%02d.%02d %02d:%02d:%02d",
           value.tm_mday, value.tm_mon + 1, (value.tm_year + 1900) % 100,
           value.tm_hour, value.tm_min, value.tm_sec);
}

static void getStatusRowLayout(int16_t& batteryX, int16_t& batteryY, uint16_t& batteryW, uint16_t& batteryH) {
  batteryW = 25;
  batteryH = 25;
  const int16_t midLineY = (int16_t)(display.height() * 2 / 3);
  batteryX = (int16_t)display.width() - (int16_t)batteryW;
  batteryY = midLineY + 2; // directly below the middle line
}

RTC_DATA_ATTR int bootCount = 0;
volatile int pendingModeCommand = -1; // -1 none, otherwise Mode enum value

void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

void setup() {
  Serial.begin(115200);
  // Pin config
  pinMode(VOLTAGE_PIN, INPUT);         // Configure A5 as ADC input

  // Wifi config
  WiFi.mode(WIFI_STA);
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  addPeer(broadcastMac);
  if (!loadBridgeMac()) {
    sendDiscover();
  }

  if (bootCount == 0)
  {
    // first boot
    display.init(115200,true,50,false);
    prepareDisplay();
    configTime(0, 0, "poolthermometer.local", "poolthermometer.local");
    //time_t rtc =  1751640850; // 1751640850 = 4 July 2025 14:54:28 MEZ+1
    //timeval tv = { rtc, 0 };
    //settimeofday(&tv, nullptr);
  } else {
    // wake up from deep sleep
    display.init(115200,false,50,false);
    checkButtons(__builtin_ffsll(esp_sleep_get_ext1_wakeup_status())-1);
  }
  
    //to find addresses for temp bus
	foundDevices = ds.search(addr, maxDevices);
	for (uint8_t j = 0; j < foundDevices; j += 1) {
		Serial.printf("Temp %d: 0x%llx,\n", j, addr[j]);
	}

  xTaskCreate(btnTaskHandling, "btnTaskHandling", 2048, NULL, 1, NULL);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
}

void loop() {
  if (!bridgeKnown && (millis() - lastDiscoverMs > discoverIntervalMs)) {
    sendDiscover();
  }

  if (pendingModeCommand >= 0) {
    const Mode requestedMode = static_cast<Mode>(pendingModeCommand);
    pendingModeCommand = -1;
    setPoolControlMode(requestedMode);
  }

  measureTemperature();
  measureVoltage();
  setSensorData();
  delay(3000);
  //getPoolControlValues();
  //delay(2000);
  //updateDisplay_LastUpdate();
  // check middle button to prevent deep sleep
  bNoSleep = (btnMiddle.checkBtn() == 2); // long press on middle button
  Serial.printf("bNoSleep: %d, %d\n", bNoSleep, btnMiddle.checkBtn());  
  if (bNoSleep) {
    Serial.println("No sleep requested, waiting for button press...");
    bNoSleep = false; // reset flag
  } else {
    //Serial.println("Going to deep sleep...");
    //startDeepSleep();
  }
  delay(7000);
}

void startDeepSleep(){
  display.hibernate();
  // Prepare wakeup
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_LOW);
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  memcpy(&myDataReceive, incomingData, sizeof(myDataReceive));

  if (len != sizeof(sensor_message)) return;

  memcpy(&inMsg, incomingData, sizeof(sensor_message));
  inMsg.id[sizeof(inMsg.id) - 1]           = '\0';
  inMsg.payload[sizeof(inMsg.payload) - 1] = '\0';

  if (strcmp(inMsg.id, "bridge/announce") == 0) {
    setBridgeMac(esp_now_info->src_addr);
    return;
  }

  // Response to get-values?
  if (strcmp(inMsg.id, "values") == 0) {
    responseReceived = true;
    Serial.printf("[NODE] Incoming response: %s\n", inMsg.payload);
    // Parse and use payload, e.g. for control logic:
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, inMsg.payload);
    if (error) {
      Serial.print("Error parsing JSON: ");
      Serial.println(error.c_str());
      return;
    }
    // Example: extract values
    if (doc["averageTempWater"].is<float>())
      myDataReceive.averageTempWater = doc["averageTempWater"];
    if (doc["averageTempAir"].is<float>())
      myDataReceive.averageTempAir = doc["averageTempAir"];
    if (doc["targetTemp"].is<float>())
      myDataReceive.targetTemp = doc["targetTemp"];
    if (doc["mode"].is<int>())
      myDataReceive.mode = static_cast<Mode>(doc["mode"].as<int>());
    if (doc["currentValveState"].is<int>())
      myDataReceive.currentValveState = static_cast<ValveState>(doc["currentValveState"].as<int>());
    if (doc["currentPumpState"].is<int>())
      myDataReceive.currentPumpState = doc["currentPumpState"];
    parseLastUpdateFromJson(doc["lastUpdate"], myDataReceive.lastUpdate);
      
    updateDisplay_PoolControlValues();
    updateDisplay_LastUpdate();
 
  } else {
    Serial.printf("[NODE] Incoming message with unknown id: %s\n", inMsg.id);
  }
}

void btnTaskHandling(void *parameter){
  while(1){
    checkButtons();
    vTaskDelay(pdMS_TO_TICKS(250)); // 250ms
  }
}

void checkButtons(int wakeupBtnPin)
{
  int btnL = (wakeupBtnPin < 0 ? btnLeft.checkBtn() : (wakeupBtnPin == BUTTON1_PIN ? 1 : 0));
  int btnM = (wakeupBtnPin < 0 ? btnMiddle.checkBtn() : (wakeupBtnPin == BUTTON2_PIN ? 1 : 0));
  int btnR = (wakeupBtnPin < 0 ? btnRight.checkBtn() : (wakeupBtnPin == BUTTON3_PIN ? 1 : 0));

  if (btnL > 0) {
    Serial.println("Button L pressed");
    pendingModeCommand = AUTO;
  }
  if (btnM > 0) {
    Serial.println("Button M pressed");
    pendingModeCommand = ON;
  }
  if (btnR > 0) {
    Serial.println("Button R pressed");
    pendingModeCommand = OFF;
  }
}

void checkButtons(){
  checkButtons(-1); // -1 = no wakeup button = check all buttons
}

void measureVoltage(){
  
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(VOLTAGE_PIN); // Read and accumulate ADC voltage
  }
  float voltage = 2 * Vbatt / 16;     // Adjust for 1:2 divider and convert to volts
  Serial.println(voltage, 3);                  // Output voltage to 3 decimal places
  
  voltage = voltage > 4000 ? 4000 : voltage; // limit to 4.1V
  voltage = voltage < 3400 ? 3400 : voltage; // limit to 3.4V 
  //float voltage = random(3412, 4095); // analogRead(VOLTAGE_PIN);
  batteryLevel = map(voltage, 3400, 4000, 0, 100);
  updateDisplay_BatteryState();
}

bool isValidTemperatureAir(float temp){
  return temp >= 0 && temp <= 50;
}

void measureTemperature(){
  // measure 
  int validMeasures = 0;
  localTemperature = 0;
  ds.request();
  // read
  for(byte j = 0; j < foundDevices; j++){
    uint8_t err = ds.getTemp(addr[j], currTemp[j]);
    if(err){
      const char *errt[] = {"", "CRC", "BAD","DC","DRV"};
      Serial.print(j); Serial.print("-Temp: "); Serial.println(errt[err]);
    }
		else{
      if (isValidTemperatureAir(currTemp[j]))
      {
        localTemperature = localTemperature + currTemp[j];
        validMeasures++;
      }
		}
	}
  
  if (validMeasures>0)
    localTemperature = localTemperature/validMeasures;
  else
    localTemperature = random(1200, 2900)/100;
  
  Serial.printf("Local Temp: %.1f\n", localTemperature);
  updateDisplay_Temperature();
}

void sendMessage(String id, String payload) {
  if (!bridgeKnown && id != "bridge/discover") {
    sendDiscover();
    return;
  }

  Serial.println("Sending message: " + payload);
  Serial.println(WiFi.macAddress());

  memset(&outMsg, 0, sizeof(outMsg));
  strncpy(outMsg.id, id.c_str(), sizeof(outMsg.id) - 1);

  strncpy(outMsg.payload, payload.c_str(), sizeof(outMsg.payload) - 1);

  // outMsg.payload[0] = '\0';

  responseReceived = false;
  esp_err_t result = esp_now_send(bridgeMac, (uint8_t *)&outMsg, sizeof(outMsg));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void getPoolControlValues() {
  sendMessage("get-values", "");
}

void setPoolControlMode(String mode) {
  sendMessage("set-mode", "{\"mode\":\"" + mode + "\"}");
}

void setPoolControlMode(Mode mode) {
  switch (mode) {
    case AUTO: setPoolControlMode("AUTO"); break;
    case ON: setPoolControlMode("ON"); break;
    case OFF: setPoolControlMode("OFF"); break;
    case CLEANING: setPoolControlMode("CLEANING"); break;
    default: break;
  }
}

void setSensorData()
{
  // Send message via ESP-NOW
  JsonDocument doc;

  // Add values in the document
  doc["sensor"] = "temperature";
  doc["location"] = "pavilion";
  doc["value"] = localTemperature;
  doc["battery"] = batteryLevel;
  doc["firmware"] = firmware;
  doc["return"] = "get-values"; // request values after setting sensor data
	String payload = "";

  // Generate the minified JSON and send it to the Serial port
  serializeJson(doc,payload);
  sendMessage("set-sensordata", payload);
}

void prepareDisplay(){
  display.setRotation(3);
  display.setFont(&FreeSansBold24pt7b);
  display.setTextColor(GxEPD_BLACK);
  uint16_t x = 5; 
  uint16_t y = 67;
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
    display.drawFastHLine(0, display.height()*2/3, display.width(), GxEPD_BLACK);
    display.drawFastVLine(display.width()/3, 0, display.height()*2/3, GxEPD_BLACK);
    display.drawFastVLine(display.width()*2/3, 0, display.height()*2/3, GxEPD_BLACK);
    
    display.drawBitmap(display.width()/3-22, y-18,epd_bitmap_celsius, 20, 20, GxEPD_WHITE, 0);
    display.drawBitmap(2*display.width()/3-22, y-18,epd_bitmap_celsius, 20, 20, GxEPD_WHITE, 0);
    display.drawBitmap(display.width()-22, y-18,epd_bitmap_celsius, 20, 20, GxEPD_WHITE, 0);

    display.drawBitmap(display.width()/3-33, y-60,epd_bitmap_pool, 25, 25, GxEPD_WHITE, 0);
    display.drawBitmap(2*display.width()/3-33, y-60,epd_bitmap_solar, 25, 25, GxEPD_WHITE,0);
    display.drawBitmap(display.width()-33, y-60,epd_bitmap_pavilion, 25, 25, GxEPD_WHITE,0);
  
    //display.drawBitmap(x, display.height()-19,epd_bitmap_full_battery, 25, 25, GxEPD_WHITE,0);
  }
  while (display.nextPage());

  updateDisplay_ButtonHints();
}

void updateDisplay_ButtonHints() {
  const int16_t box_x = 0;
  const uint16_t box_w = display.width();
  const uint16_t box_h = 13;
  const int16_t box_y = display.height() - box_h;

  display.setRotation(3);
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);

    const int16_t slotW = box_w / 3;
    const int16_t cy = box_y + (box_h / 2) - 1;
    const int16_t r = 6;

    const int16_t autoCx = box_x + (slotW / 2) + 10;
    const int16_t onCx = box_x + slotW + (slotW / 2);
    const int16_t offCx = box_x + 2 * slotW + (slotW / 2) - 8;

    // AUTO: circle + A
    display.drawCircle(autoCx, cy, r, GxEPD_BLACK);
    display.setCursor(autoCx - 2, cy - 3);
    display.print("A");

    // ON: circle + play symbol
    display.drawCircle(onCx, cy, r, GxEPD_BLACK);
    display.fillTriangle(onCx - 2, cy - 2, onCx - 2, cy + 2, onCx + 2, cy, GxEPD_BLACK);

    // OFF: circle + stop symbol
    display.drawCircle(offCx, cy, r, GxEPD_BLACK);
    display.fillRect(offCx - 2, cy - 2, 4, 4, GxEPD_BLACK);
  }
  while (display.nextPage());
}

void updateDisplay_PoolControlValues(){
  uint16_t x = 5;
  uint16_t y = 67; 
  uint16_t box_w = 73;
  uint16_t box_h = 36;
  uint16_t box_x = x;
  uint16_t box_y = y-box_h+2; 

  // temperatures
  // water 
  display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    //display.drawRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    display.setCursor(x, y);
    display.setFont(&FreeSansBold24pt7b);
    display.printf("%.0f", floor(myDataReceive.averageTempWater));
    display.setFont(&FreeSansBold12pt7b);
    display.printf(".%.0f\n", (myDataReceive.averageTempWater-(int)myDataReceive.averageTempWater)*10);
  }
  while (display.nextPage());

  // air 
  display.setPartialWindow(box_x+display.width()/3, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    //display.fillRect(box_x+display.width()/3, box_y, box_w, box_h, GxEPD_BLACK);
    display.fillRect(box_x+display.width()/3, box_y, box_w, box_h, GxEPD_WHITE);
    display.setCursor(box_x+display.width()/3, y);
    display.setFont(&FreeSansBold24pt7b);
    display.printf("%.0f", floor(myDataReceive.averageTempAir));
    display.setFont(&FreeSansBold12pt7b);
    display.printf(".%.0f\n", (myDataReceive.averageTempAir-(int)myDataReceive.averageTempAir)*10);
  }
  while (display.nextPage());
  
  // mode
  box_w = 160;
  box_h = 16;
  box_y = display.height()-20-box_h+2;
  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    display.setCursor(x, display.height()-20);
    display.setFont(&FreeSans9pt7b);
    switch (myDataReceive.mode) {
      case ON: display.printf("Manuell - AN"); break;
      case OFF: display.printf("Manuell - AUS"); break;
      case CLEANING: display.printf("Manuell - REINIGUNG"); break;
      case AUTO: display.printf("AUTO"); break;
    }
   }
  while (display.nextPage());

  updateDisplay_LastUpdate();
  updateDisplay_ButtonHints();
}

void updateDisplay_Temperature(){
  uint16_t x = 5;
  uint16_t y = 67; 
  uint16_t box_w = 73;
  uint16_t box_h = 36;
  uint16_t box_x = x;
  uint16_t box_y = y-box_h+2; 

  // local temperature pavilion 
  display.setRotation(3);
  display.setFont(&FreeSansBold24pt7b);
  display.setTextColor(GxEPD_BLACK);
  display.setPartialWindow(box_x+2*display.width()/3, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    display.fillRect(box_x+2*display.width()/3, box_y, box_w, box_h, GxEPD_WHITE);
    display.setCursor(box_x+2*display.width()/3, y);
    display.setFont(&FreeSansBold24pt7b);
    display.printf("%.0f", floor(localTemperature));
    display.setFont(&FreeSansBold12pt7b);
    display.printf(".%.0f\n", (localTemperature-(int)localTemperature)*10);
    Serial.printf("local Temp: %.1f", localTemperature);
  }
  while (display.nextPage());

}

void updateDisplay_LastUpdate(){
  // return;
  if (myDataReceive.lastUpdate.tm_year == 0)
  {
    // no last update time available
    Serial.println("No last update time available");
    return;
  }

  char lastUpdateText[48];
  formatDateTimeGerman(myDataReceive.lastUpdate, lastUpdateText, sizeof(lastUpdateText));
  Serial.println(lastUpdateText);

  // last update
  display.setRotation(3);
  display.setFont(nullptr);
  display.setTextSize(1);
  display.setTextColor(GxEPD_BLACK);
  int16_t x1, y1;
  uint16_t textW, textH;
  display.getTextBounds(lastUpdateText, 0, 0, &x1, &y1, &textW, &textH);
  int16_t batteryX, batteryY;
  uint16_t batteryW, batteryH;
  getStatusRowLayout(batteryX, batteryY, batteryW, batteryH);

  const int16_t gap = 4;
  int16_t textX = batteryX - gap - (int16_t)textW;
  if (textX < 0) textX = 0;
  int16_t textBaselineY = batteryY + ((int16_t)batteryH - (int16_t)textH) / 2 - 1;
  if (textBaselineY > (int16_t)display.height()) textBaselineY = (int16_t)display.height();

  int16_t box_x = textX;
  int16_t box_y = batteryY;
  uint16_t box_w = (uint16_t)(batteryX - textX);
  uint16_t box_h = batteryH;

  display.setPartialWindow(box_x, box_y+1, box_w, box_h-1);
  display.firstPage();
  do
  {
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    //display.drawRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
    display.setCursor(textX, textBaselineY);
    display.println(lastUpdateText);
    Serial.println(lastUpdateText);
  }
  while (display.nextPage());

}

void updateDisplay_BatteryState(){
  int16_t iconX, iconY;
  uint16_t box_w, box_h;
  getStatusRowLayout(iconX, iconY, box_w, box_h);
  int16_t box_x = iconX;
  int16_t box_y = iconY;

  display.setRotation(3);
  display.setPartialWindow(box_x, box_y+1, box_w, box_h);
  display.firstPage();
  do
  {
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    //display.drawRect(iconX, iconY, 25, 25, GxEPD_BLACK);
    //display.drawRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);

    Serial.printf("Battery-Level:%d, Map:%d\n", batteryLevel, map(batteryLevel, 0, 100, 4, 0));
    display.drawBitmap(iconX, iconY-2, epd_bitmap_batteryArray[map(batteryLevel, 0, 100, 4, 0)], 25, 25, GxEPD_WHITE, 0);
  }
  while (display.nextPage());
}
