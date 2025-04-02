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
#include <imagedata.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Button.h>
#include <driver/rtc_io.h>

#define VOLTAGE_PIN 0  // Battery Voltage Pin
#define BUTTON1_PIN GPIO_NUM_1  // Button 1 Pin
#define BUTTON2_PIN GPIO_NUM_2  // Button 2 Pin
#define BUTTON3_PIN GPIO_NUM_21  // Button 3 Pin
#define TEMP_PIN 22    // DS18B20 Datenleitung Lufttemperatur
#define CS_PIN 23    // Display CS Pin
#define BUSY_PIN 16  // Display BUSY Pin
#define RST_PIN 17   // Display RST Pin
#define DC_PIN 20    // Display DC Pin

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */
#define BUTTON_PIN_BITMASK (1ULL << BUTTON1_PIN) | (1ULL << BUTTON2_PIN) 
//| (1ULL << BUTTON3_PIN) // GPIO 0 bitmask for ext1

const char* firmware = "0.2.5";
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
  OFF     // Solar heating off, pump in eco mode (lowest level)
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
} struct_message_receive;

struct_message_receive myDataReceive;

OneWire32 ds(TEMP_PIN);
const uint8_t maxDevices = 2;  // max devices per bus
uint8_t foundDevices = 0;
uint64_t addr[maxDevices];
float currTemp[maxDevices];

int batteryLevel;
float localTemperature;

// REPLACE WITH Pool Controller MAC Address
// 54:32:04:11:D4:FC
uint8_t broadcastAddress[] = {0x54, 0x32, 0x04, 0x11, 0xD4, 0xFC};

void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void updateDisplay_PoolControlValues();
void updateDisplay_Temperature();
void updateDisplay_BatteryState();
bool isValidTemperatureAir(float temp);
void measureTemperature();
void setPoolControlMode(String mode);
void measureVoltage();
void getPoolControlValues();
void prepareDisplay();
void setSensorData();
void sendMessage(String payload);
void btnTaskHandling(void *parameter);
void checkButtons();
void checkButtons(int wakeupBtnPin);
void startDeepSleep();

RTC_DATA_ATTR int bootCount = 0;

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
  pinMode(VOLTAGE_PIN, INPUT);         // Configure A0 as ADC input

  // Wifi config
  WiFi.mode(WIFI_STA);
  Serial.print("MAC-Address: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false; 
 
  esp_err_t err = esp_now_add_peer(&peerInfo);
  if (err != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }

  if (bootCount == 0)
  {
    // first boot
    display.init(115200,true,50,false);
    prepareDisplay();
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

  xTaskCreate(btnTaskHandling, "btnTaskHandling", 1024, NULL, 1, NULL);
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();
}

void loop() {
  measureTemperature();
  measureVoltage();
  setSensorData();

  getPoolControlValues();
  delay(3000);
  
  startDeepSleep();
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
  Serial.printf("%.1f°C", myDataReceive.averageTempWater);
  Serial.println("Data received...refresh values");
  updateDisplay_PoolControlValues();
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

  if (btnL > 1) {
    Serial.println("Button L pressed");
    setPoolControlMode("AUTO");
  }
  if (btnM > 1) {
    Serial.println("Button M pressed");
    setPoolControlMode("ON");
  }
  if (btnR > 1) {
    Serial.println("Button R pressed");
    setPoolControlMode("OFF");
  }
}

void checkButtons(){
  checkButtons(-1); // -1 = no wakeup button = check all buttons
}

void measureVoltage(){
  /*
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt += analogReadMilliVolts(VOLTAGE_PIN); // Read and accumulate ADC voltage
  }
  float voltage = 2 * Vbatt / 16 / 1000.0;     // Adjust for 1:2 divider and convert to volts
  Serial.println(voltage, 3);                  // Output voltage to 3 decimal places
  */
  float voltage = random(3412, 4095); // analogRead(VOLTAGE_PIN);
  batteryLevel = map(voltage, 3412, 4095, 0, 100);
  updateDisplay_BatteryState();
}

bool isValidTemperatureAir(float temp){
  return temp >= 0 && temp <= 50;
}

void measureTemperature(){
  // measure 
  int validMeasures = 0;
  ds.request();
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
    localTemperature = validMeasures/validMeasures;
  else
    localTemperature = random(1200, 2900)/100;
  
  Serial.printf("Local Temp: %.1f\n", localTemperature);
  updateDisplay_Temperature();
}

void sendMessage(String payload)
{
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)payload.c_str(), payload.length());
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void getPoolControlValues() {
  JsonDocument doc;

  // Add values in the document
  doc["cmd"] = "get-values";

  String payload = "";

  // Generate the minified JSON and send it to the Serial port
  serializeJson(doc,payload);
  sendMessage(payload);
}

void setPoolControlMode(String mode) {
  // Send message via ESP-NOW
  JsonDocument doc;

  // Add values in the document
  doc["cmd"] = "set-mode";
  doc["mode"] = mode.c_str();

  String payload = "";

  // Generate the minified JSON and send it to the Serial port
  serializeJson(doc,payload);
  sendMessage(payload);
}

void setSensorData()
{
  // Send message via ESP-NOW
  JsonDocument doc;

  // Add values in the document
  doc["cmd"] = "set-sensordata";
  doc["sensor"] = "temperature";
  doc["location"] = "pavilion";
  doc["value"] = localTemperature;
  doc["battery"] = batteryLevel;
  doc["firmware"] = firmware;
	String payload = "";

  // Generate the minified JSON and send it to the Serial port
  serializeJson(doc,payload);
  sendMessage(payload);
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
  
    display.drawBitmap(x, display.height()-19,epd_bitmap_full_battery, 25, 25, GxEPD_WHITE,0);

    display.setCursor(x, display.height()-20);
    display.setFont(&FreeSans9pt7b);
    display.printf("Aktualisiere...");
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
  box_w = 120;
  box_h = 16;
  box_y = display.height()-20-box_h+2;
  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    //display.drawRect(box_x, box_y, box_w, box_h, GxEPD_BLACK);
    display.fillRect(box_x, box_y, box_w, box_h, GxEPD_WHITE);
    display.setCursor(x, display.height()-20);
    display.setFont(&FreeSans9pt7b);
    switch (myDataReceive.mode) {
      case ON: display.printf("Manuell - AN"); break;
      case OFF: display.printf("Manuell - AUS"); break;
      case AUTO: display.printf("AUTO"); break;
    }
   }
  while (display.nextPage());

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

void updateDisplay_BatteryState(){
  uint16_t x = 5;
  uint16_t y = 67; 
  uint16_t box_w = 25;
  uint16_t box_h = 19;
  uint16_t box_x = x;
  uint16_t box_y = display.height()-10; 

  display.setRotation(3);
  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    Serial.printf("Battery-Level:%d, Map:%d\n", batteryLevel, map(batteryLevel, 0, 100, 4, 0));
    display.drawBitmap(x, display.height()-19,epd_bitmap_batteryArray[map(batteryLevel, 0, 100, 4, 0)], 25, 25, GxEPD_WHITE,0);
  }
  while (display.nextPage());
}
