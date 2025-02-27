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

#define TEMP_PIN 22    // DS18B20 Datenleitung Lufttemperatur

// E-Paper config
// ESP32-C3 CS(SS)=7,SCL(SCK)=4,SDA(MOSI)=6,BUSY=3,RES(RST)=2,DC=1
// 2.9'' EPD Module
GxEPD2_BW<GxEPD2_290_BS, GxEPD2_290_BS::HEIGHT> display(GxEPD2_290_BS(/*CS=5*/ 7, /*DC=*/ 1, /*RES=*/ 2, /*BUSY=*/ 3)); // DEPG0290BS 128x296, SSD1680
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

typedef struct struct_message_send {
  char command[32];
  char parameter[32];
} struct_message_send;

struct_message_send myDataSend;
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

void setup() {
  Serial.begin(115200);

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

  display.init(115200,true,50,false);
  prepareDisplay();

    //to find addresses for temp bus
	foundDevices = ds.search(addr, maxDevices);
	for (uint8_t j = 0; j < foundDevices; j += 1) {
		Serial.printf("Temp %d: 0x%llx,\n", j, addr[j]);
	}
}

void loop() {
  measureTemperature();
  setPoolControlMode("ON");
  measureVoltage();
  getPoolControlValues();
  delay(10000);
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

void measureVoltage(){
  float voltage = random(3412, 4095); // analogRead(33)
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

void getPoolControlValues() {
  // Send message via ESP-NOW
  char cmd[] = "get-values";
  memcpy(&myDataSend.command, cmd, sizeof(cmd));
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataSend, sizeof(myDataSend));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}

void setPoolControlMode(String mode) {
  // Send message via ESP-NOW
  char cmd[] = "set-mode";
  char parameter[mode.length()+1];
  strcpy(parameter, mode.c_str());
  memcpy(&myDataSend.command, cmd, sizeof(cmd));
  memcpy(&myDataSend.parameter, parameter, sizeof(parameter));
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myDataSend, sizeof(myDataSend));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
}


void prepareDisplay(){
  display.setRotation(1);
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

  display.setPartialWindow(box_x, box_y, box_w, box_h);
  display.firstPage();
  do
  {
    Serial.printf("Battery-Level:%d, Map:%d\n", batteryLevel, map(batteryLevel, 0, 100, 4, 0));
    display.drawBitmap(x, display.height()-19,epd_bitmap_batteryArray[map(batteryLevel, 0, 100, 4, 0)], 25, 25, GxEPD_WHITE,0);
  }
  while (display.nextPage());
}
