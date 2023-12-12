#include <LiquidCrystal_I2C.h>
#include <Adafruit_Fingerprint.h>
#include <ArduinoJson.h>
#include <MFRC522.h>
#include <Keypad.h>
#include "esp_log.h"
#include <AsyncMqttClient.h>
#include <WiFi.h>
extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

/******Khai báo cho Wifi và MQTT******/

#define WIFI_SSID "308A C9 2.4G"
#define WIFI_PASSWORD "12345678"

// #define WIFI_SSID "VIETTEL_MAI PHUONG"
// #define WIFI_PASSWORD "23452345"

// BROKER
#define MQTT_HOST IPAddress(192, 168, 1, 27)
#define MQTT_PORT 1883

// MQTT Publish Topics
#define MQTT_PUB_RFID "esp/door/register/RFID"
#define MQTT_PUB_KEYPAD "esp/door/register/keypad"
#define MQTT_PUB_FINGERPRINT "esp/door/register/fingerprint"

// MQTT check Topics
#define MQTT_PUB_CHECK_RFID "esp/door/check/RFID"
#define MQTT_PUB_CHECK_KEYPAD "esp/door/check/keypad"
#define MQTT_PUB_CHECK_FINGERPRINT "esp/door/check/fingerprint"


AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
unsigned long previousMillis = 0; // Stores last time a message was published
const long interval = 10000;      // Interval at which to publish values

/****************************Khai báo cho LCD******************************************/
LiquidCrystal_I2C lcd(0x27,20,4); 

/***************************Khai báo finger print*************************************/
#define mySerial Serial2
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);
uint8_t id; // id này nhận số từ keypad phục vụ cho finger print
uint8_t sumID; // tổng số ID cho finger print
/******************************biến phục vụ cho hai hàm fingerprint*/
uint8_t global_p;
int global_id;
// Định nghĩa cấu trúc JSON lưu dữ liệu vân tay để publish
typedef struct {
  int id;
  int confidence; 
  int area;
  float scale;
  char type[10];
} FingerprintData;
FingerprintData fingerInfo; 

StaticJsonDocument<512> doc;
/*Trong đó:
StaticJsonDocument là class để tạo và chứa 1 JSON document trong ArduinoJson.
<200> định nghĩa dung lượng tối đa mà JSON document có thể chứa là 200 bytes.
Mục đích của nó là để cấp phát sẵn bộ nhớ cho JSON document thay vì cấp phát động. 
Nhờ đó, tối ưu hoá hiệu năng và giảm việc fragment memory.
Khai báo như trên tương đương với:
char jsonBuffer[200];
DynamicJsonDocument doc(200);
Trong đó:
Dùng mảng char[200] thay vì con trỏ để cấp phát tĩnh bộ nhớ cho JSON.
Dung lượng cũng được định nghĩa là 200 bytes.
Ưu điểm của cách 1 so với cách 2:
Code ngắn gọn hơn, Không cần quản lý vùng nhớ tĩnh jsonBuffer
Tuy nhiên vẫn có thể dùng cách thứ 2 nếu muốn tách biết buffer và đối tượng JSON cho gọn.

/*****************************khai báo nút bấm keypad event menu*/
uint8_t eventMenu;
int flagPassword = 0;
int flagFingerprint = 0;
int flagRFID = 0;


/***************************Khai báo cho key pad**************************************/
const byte ROWS = 4; // four rows
const byte COLS = 4; // four columns
// define the cymbols on the buttons of the keypads
char hexaKeys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {26, 25, 33, 32}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {13, 12, 14, 15}; // connect to the column pinouts of the keypad
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);




/*****************Khai báo password cho keypad*******************************************/
String password; // chuỗi này để lưu mật khẩu người dùng tạo
String input_password; // chuỗi này để lưu tạm thời mật khẩu người dùng tạo

/*******************************Khai báo cho RFID*****************************************/

#define SS_PIN 5   // ESP32 pin GPIO5
#define RST_PIN 27 // ESP32 pin GPIO27
MFRC522 rfid(SS_PIN, RST_PIN);

byte keyTagUID[10][4];
uint8_t numTags;

/************Function cho MQTT***********/
void connectToWifi(){
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}
void connectToMqtt(){
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}
void WiFiEvent(WiFiEvent_t event){
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event){
  case SYSTEM_EVENT_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case SYSTEM_EVENT_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}
void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}
void onMqttSubscribe(uint16_t packetId, uint8_t qos)
{
  Serial.println("Subscribe acknowledged.");
  Serial.print(" packetId: ");
  Serial.println(packetId);
  Serial.print(" qos: ");
  Serial.println(qos);
}
void onMqttUnsubscribe(uint16_t packetId)
{
  Serial.println("Unsubscribe acknowledged.");
  Serial.print(" packetId: ");
  Serial.println(packetId);
}
void onMqttMessage(char *topic, char *payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total)
{
  // Do whatever you want when you receive a message
  // Save the message in a variable
  String receivedMessage;
  for (int i = 0; i < len; i++)
  {
    Serial.println((char)payload[i]);
    receivedMessage += (char)payload[i];
  }
  // Save topic in a String variable
  String receivedTopic = String(topic);
  Serial.print("Received Topic: ");
  Serial.println(receivedTopic);
  // Check which GPIO we want to control
  int stringLen = receivedTopic.length();
  // Get the index of the last slash
  int lastSlash = receivedTopic.lastIndexOf("/");
  // Get the GPIO number (it's after the last slash "/")
  // esp/digital/GPIO
  String gpio = receivedTopic.substring(lastSlash + 1, stringLen);
  Serial.print("DIGITAL GPIO: ");
  Serial.println(gpio);
  Serial.print("STATE: ");
  Serial.println(receivedMessage);
  // Check if it is DIGITAL
  if (receivedTopic.indexOf("digital") > 0)
  {
    // Set the specified GPIO as output
    pinMode(gpio.toInt(), OUTPUT);
    // Control the GPIO
    if (receivedMessage == "true")
    {
      digitalWrite(gpio.toInt(), HIGH);
    }
    else
    {
      digitalWrite(gpio.toInt(), LOW);
    }
  }
  Serial.println("Publish received.");
  Serial.print(" topic: ");
  Serial.println(topic);
  Serial.print(" qos: ");
  Serial.println(properties.qos);
  Serial.print(" dup: ");
  Serial.println(properties.dup);
  Serial.print(" retain: ");
  Serial.println(properties.retain);
  Serial.print(" len: ");
  Serial.println(len);
  Serial.print(" index: ");
  Serial.println(index);
  Serial.print(" total: ");
  Serial.println(total);
}
void onMqttPublish(uint16_t packetId)
{
  Serial.println("Publish acknowledged.");
  Serial.print(" packetId: ");
  Serial.println(packetId);
}



/**************Prototype Function cho finger print***************/
void setUpFingerprint(void){
  while (!Serial)
    ; // For Yun/Leo/Micro/Zero/...
  delay(100);
  Serial.println("\n\nAdafruit Fingerprint sensor enrollment");

  // set the data rate for the sensor serial port
  finger.begin(57600);

  if (finger.verifyPassword()){
    Serial.println("Found fingerprint sensor!");
  }
  else{
    Serial.println("Did not find fingerprint sensor :(");
    while (1){
      delay(1);
    }
  }
  finger.emptyDatabase();
  Serial.println(F("Empty Database of sensor"));
  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x"));
  Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x"));
  Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: "));
  Serial.println(finger.capacity);
  Serial.print(F("Security level: "));
  Serial.println(finger.security_level);
  Serial.print(F("Device address: "));
  Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: "));
  Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: "));
  Serial.println(finger.baud_rate);
}

void EnrollFingerprint(void);
uint8_t readNumberKeypad(void);
uint8_t getFingerprintEnroll();
void getFingerprintID1(void *pvParameter);
void printHex(int num, int precision);

/*******************Prototype Function cho keypad**********************/
void registerPassword(void);
void checkKeyPad(void *pvParameter);
// Hàm đặc biệt, read from keypad
// tương tự như readNumberKeypad nhưng tôi lười đổi :))
uint8_t readnumber(void);
/*******************Prototype Function cho RFID**********************/
void taskCheckRFID(void *pvParameter);
void registerRFID();
void checkRFID(void *pvParameter);

/*****************************Màn hình welcome*******************************************/

void displayMenu(void){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Welcome Smartkey !!"); 
  Serial.println("Welcome Smartkey !!"); 
  lcd.setCursor(0,1);
  lcd.print("1. Register Pass"); 
  Serial.println("1. Register Pass"); 
  lcd.setCursor(0,2);
  lcd.print("2. Register Finger");
  Serial.println("2. Register Finger");
  lcd.setCursor(0,3);
  lcd.print("3. Register RFIDTag");
  Serial.println("3. Register RFIDTag");
}
void setup()
{
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  setUpFingerprint();
  SPI.begin();     // init SPI bus
  rfid.PCD_Init(); // init MFRC522

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE,
                                    (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE,
                                    (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
  WiFi.onEvent(WiFiEvent);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  //mqttClient.setCredentials(BROKER_USER, BROKER_PASS);
  connectToWifi();



  while (flagPassword == 0 || flagFingerprint == 0|| flagRFID == 0){
    displayMenu();
    eventMenu = readNumberKeypad();
    switch (eventMenu)
    {
    case 1:
      registerPassword();
      flagPassword = 1;
      Serial.print("flagPassword:");
      Serial.println(flagPassword);
      break;
    case 2:
      EnrollFingerprint();
      flagFingerprint = 1;
      Serial.print("flagFingerprint:");
      Serial.println(flagFingerprint);
      break;
    case 3:
      registerRFID();
      flagRFID = 1;
      Serial.print("flagRFID:");
      Serial.println(flagRFID);      
      break;
    }
  }
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Register done!!!");

  xTaskCreate(taskCheckRFID, "Task RFID", 4096, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(checkKeyPad, "Task Keypad", 4096, NULL, tskIDLE_PRIORITY, NULL);
  xTaskCreate(getFingerprintID1, "Task Get FingerprintID", 4096, NULL, tskIDLE_PRIORITY, NULL);

}
void loop(){

}

/**************Function cho finger print***************/

void EnrollFingerprint(void){
  Serial.println("Ready to enroll a fingerprint!");
  Serial.println("Please type in the ID # (from 1 to 127) you want to save this finger as...");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Ready to enroll");
  lcd.setCursor(2,1);
  lcd.print("fingerprint !!");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Number SUM ID: ");
  lcd.setCursor(0, 1);
  lcd.print("for finger register");
  // (from 1 to 127) 
  sumID = readnumber();
  if (sumID == 0) {// ID #0 not allowed, try again!
     return;
  }
  lcd.setCursor(16, 0);
  lcd.print(sumID);
  Serial.print("Sum ID #");
  Serial.println(sumID);
  delay(2000);
  for(id = 1; id <= sumID;id ++){
    while (!getFingerprintEnroll() );
    //downloadFingerprintTemplate(id);
  }
  Serial.println("Register done!!!");
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Register done!!!");
}
uint8_t readNumberKeypad(void){
  uint8_t num = 0;
  while (num == 0)
  {
    char key = customKeypad.waitForKey();
    num = (uint8_t)key - 48;
    lcd.setCursor(0, 1);
    lcd.print(num);
    key = NO_KEY;
  }
  return num;
}
uint8_t getFingerprintEnroll() {

  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); 
  Serial.println(id);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wait valid finger");
  lcd.setCursor(0, 1);
  lcd.print("to enroll as #"); 
  lcd.setCursor(15, 1) ;
  lcd.print(id);
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    Serial.print("p=");
    Serial.print(p < 0x10 ? " 0" : " ");
    Serial.println(p, HEX);
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      lcd.clear();
      lcd.setCursor(3, 1);
      lcd.print("Image taken");
      delay(2000);
      break;
    case FINGERPRINT_NOFINGER:
      Serial.println(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!
  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  Serial.println("Remove finger");
  lcd.clear();
  lcd.setCursor(3, 1);
  lcd.print("Remove finger");
  delay(2000);
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.println(id);
  p = -1;
  Serial.println("Place same finger again");
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Place finger again");
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image taken");
      lcd.clear();
      lcd.setCursor(3, 1);
      lcd.print("Image taken"); 
      delay(2000);     
      break;
    case FINGERPRINT_NOFINGER:
      Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      break;
    case FINGERPRINT_IMAGEFAIL:
      Serial.println("Imaging error");
      break;
    default:
      Serial.println("Unknown error");
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      Serial.println("Image converted");
      break;
    case FINGERPRINT_IMAGEMESS:
      Serial.println("Image too messy");
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      Serial.println("Communication error");
      return p;
    case FINGERPRINT_FEATUREFAIL:
      Serial.println("Could not find fingerprint features");
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      Serial.println("Could not find fingerprint features");
      return p;
    default:
      Serial.println("Unknown error");
      return p;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    Serial.println("Prints matched!");
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    Serial.println("Fingerprints did not match");
    return p;
  } else {
    Serial.println("Unknown error");
    return p;
  }

  Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("Stored!");
    uint8_t p1 = finger.loadModel(id);
    Serial.print("Template "); Serial.print(id); Serial.println(" loaded");
    p1 = finger.getModel();
    switch (p1)
    {
      case FINGERPRINT_OK:
        Serial.print("Template ");
        Serial.print(id);
        Serial.println(" transferring:");
        break;
      default:
        Serial.print("Unknown error ");
        Serial.println(p1);
        return p1;
    }
    // one data packet is 267 bytes. in one data packet, 11 bytes are 'usesless' :D
  uint8_t bytesReceived[534]; // 2 data packets
  memset(bytesReceived, 0xff, 534);

  uint32_t starttime = millis();
  int i = 0;
  while (i < 534 && (millis() - starttime) < 20000) {
    if (mySerial.available()) {
      bytesReceived[i++] = mySerial.read();
    }
  }
  Serial.print(i); Serial.println(" bytes read.");
  Serial.println("Decoding packet...");

  uint8_t fingerTemplate[512]; // the real template
  memset(fingerTemplate, 0xff, 512);

  // filtering only the data packets
  int uindx = 9, index = 0;
  memcpy(fingerTemplate + index, bytesReceived + uindx, 256);   // first 256 bytes
  uindx += 256;       // skip data
  uindx += 2;         // skip checksum
  uindx += 9;         // skip next header
  index += 256;       // advance pointer
  memcpy(fingerTemplate + index, bytesReceived + uindx, 256);   // second 256 bytes

  for (int i = 0; i < 512; ++i) {
    //Serial.print("0x");
    printHex(fingerTemplate[i], 2);
    //Serial.print(", ");
  }
  Serial.println("\ndone.");
    fingerInfo.id = id;
    fingerInfo.area = finger.image2Tz(2); 
    /*
      Hàm finger.image2Tz() trong thư viện Adafruit Fingerprint dùng để 
      chuyển đổi ảnh vân tay (image) sang dạng mẫu vân tay (template - tz).
      Tham số truyền vào hàm image2Tz() có ý nghĩa:
          0: Sử dụng ảnh vân tay đã lưu trong bộ nhớ đệm.
          1: Sử dụng ảnh vân tay đầu tiên trong quá trình enroll.
          2: Sử dụng ảnh vân tay thứ 2 trong quá trình enroll.          
    */
    fingerInfo.confidence = finger.confidence; 
    
    
    // Chuyển struct sang JSON
    doc["id"] = id;
    for (int i = 0; i < 512; ++i) {
      doc["template"][i] = fingerTemplate[i];
    }
    String jsonString;
    serializeJson(doc, jsonString);
    uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_FINGERPRINT, 1, true, jsonString.c_str());
  } 
  else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } 
  else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not store in that location");
    return p;
  } 
  else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    return p;
  } 
  else {
    Serial.println("Unknown error");
    return p;
  }
  return true;
}

/********/
// Function cho check finger
void getFingerprintID1(void *pvParameter) {
  while(1){
    // Loop until a finger is detected
    while(1) {
      global_p = finger.getImage();
      switch (global_p) {
        case FINGERPRINT_OK:
          Serial.println("Image taken");
          break; // Break out of the loop
        case FINGERPRINT_NOFINGER:
          Serial.println("No finger detected");
          break;
        case FINGERPRINT_PACKETRECIEVEERR:
          Serial.println("Communication error");
          break;
        case FINGERPRINT_IMAGEFAIL:
          Serial.println("Imaging error");
          break;
        default:
          Serial.println("Unknown error");
          break;
      }
      // Check if the finger is detected
      if (global_p == FINGERPRINT_OK) {
        break; // Break out of the loop
      }
      // Add a delay to avoid polling too frequently
      delay(100);
    }

    // OK success!

    global_p = finger.image2Tz();
    switch (global_p) {
      case FINGERPRINT_OK:
        Serial.println("Image converted");
        break;
      case FINGERPRINT_IMAGEMESS:
        Serial.println("Image too messy");
        break;
      case FINGERPRINT_PACKETRECIEVEERR:
        Serial.println("Communication error");
        break;
      case FINGERPRINT_FEATUREFAIL:
        Serial.println("Could not find fingerprint features");
        break;
      case FINGERPRINT_INVALIDIMAGE:
        Serial.println("Could not find fingerprint features");
        break;
      default:
        Serial.println("Unknown error");
        break;
    }

    // OK converted!
    global_p = finger.fingerSearch();
    if (global_p == FINGERPRINT_OK) {
      Serial.println("Found a print match!");
    } else if (global_p == FINGERPRINT_PACKETRECIEVEERR) {
      Serial.println("Communication error");
    } else if (global_p == FINGERPRINT_NOTFOUND) {
      Serial.println("Did not find a match");
    } else {
      Serial.println("Unknown error");
    }

    // found a match!
    Serial.print("Found ID #"); Serial.print(finger.fingerID);
    Serial.print(" with confidence of "); Serial.println(finger.confidence);
    if(0<finger.fingerID&&finger.fingerID<=sumID&&finger.confidence>50){
      lcd.clear();
      lcd.setCursor(2, 1);
      lcd.print("ACCESS GRANTED!");
      fingerInfo.id = id;
      fingerInfo.area = finger.image2Tz(2); 
      /*
        Hàm finger.image2Tz() trong thư viện Adafruit Fingerprint dùng để 
        chuyển đổi ảnh vân tay (image) sang dạng mẫu vân tay (template - tz).
        Tham số truyền vào hàm image2Tz() có ý nghĩa:
            0: Sử dụng ảnh vân tay đã lưu trong bộ nhớ đệm.
            1: Sử dụng ảnh vân tay đầu tiên trong quá trình enroll.
            2: Sử dụng ảnh vân tay thứ 2 trong quá trình enroll.          
      */
      fingerInfo.scale = 0.2;
      fingerInfo.confidence = finger.confidence; 
      strcpy(fingerInfo.type, "Optical");
      
      char fingerprintBuffer[256];
      // Chuyển struct sang JSON
      doc["id"] = finger.fingerID;
      doc["confidence"] = finger.confidence;
      doc["area"]= finger.image2Tz(2); 

      String jsonString;
      serializeJson(doc, jsonString);
      uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_CHECK_FINGERPRINT, 1, true, jsonString.c_str());
    }
    else{
      lcd.clear();
      lcd.setCursor(2, 1);
      lcd.print("ACCESS DENIDED!");
    }
    // Store the ID in a global variable
    global_id = finger.fingerID;
  }
}



void printHex(int num, int precision) {
  char tmp[16];
  char format[128];

  sprintf(format, "%%.%dX", precision);

  sprintf(tmp, format, num);
  Serial.print(tmp);
}
/*******************Function cho keypad**********************/
void registerPassword(void){
  lcd.clear();
  String input_passwordset;
  Serial.println("Create Password");
  lcd.setCursor(0, 0);
  lcd.print("Create Password");
  char key = '1';
  while (key!='#')
  {
    key = customKeypad.waitForKey();
    if(key!='#'){
      Serial.print(key);
      lcd.setCursor(input_passwordset.length(), 1);
      lcd.print(key);
      input_passwordset += key;
    }
  }
  password = input_passwordset;
  Serial.println();
  Serial.println("Create Done!!!");
  lcd.setCursor(0, 0);
  lcd.clear();
  lcd.print("Create Done!!!");
  Serial.print("Password: ");
  Serial.println(password);
  lcd.setCursor(0, 1);
  lcd.print(password);
  lcd.clear();
  lcd.setCursor(1, 1);
  lcd.print("Register done!!!");
  // Publish an MQTT message on topic esp32/BME2800/temperature
  uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_KEYPAD, 1, true, password.c_str());
  Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_KEYPAD, packetIdPub1);
  Serial.printf("Password publish:");
  Serial.printf("Password: %s\n", password.c_str());
}
void checkKeyPad(void *pvParameter)
{
  while (1)
  {
    char key = customKeypad.waitForKey();
    if (key){
      if (key != '#'){
        Serial.print(key);
        lcd.setCursor(input_password.length(), 0);
      }

      if (key == '*'){
        input_password = ""; // clear input password
      }
      else if (key == '#'){
        if (password == input_password)
        {
          Serial.println();
          Serial.println("The password is correct, ACCESS GRANTED!");
          // DO WORK
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("ACCESS GRANTED!");
          // 1. Open Door
          // 2. Publish mes to broker
          String notification = "Correct Password";
          notification= input_password + notification;
          uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_CHECK_KEYPAD, 1, true, notification.c_str());
        }
        else{
          Serial.println();
          Serial.println("The password is incorrect, ACCESS DENIED!");
          lcd.clear();
          lcd.setCursor(2, 1);
          lcd.print("ACCESS DENIED!");
          String notification = "Incorrect Password";
          notification= input_password + notification;
          uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_CHECK_KEYPAD, 1, true, notification.c_str());
        }

        input_password = ""; // clear input password
      }
      else{
        input_password += key; // append new character to input password string
      }
    }
  }
}


// Hàm đặc biệt, read from keypad
uint8_t readnumber(void){
  uint8_t num = 0;
  while (num == 0)
  {
    char key = customKeypad.waitForKey();
    num = (uint8_t)key - 48;
    lcd.setCursor(0, 1);
    lcd.print(num);
    key = NO_KEY;
  }
  return num;
}
/*******************Function cho RFID**********************/
void taskCheckRFID(void *pvParameter){

  while (1) {

    // Đọc UID thẻ
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      // Duyệt mảng keyTagUID để tìm UID trùng khớp
      bool accessGranted = false;
      for(int i=0; i<10; i++){
        if(rfid.uid.uidByte[0] == keyTagUID[i][0] && rfid.uid.uidByte[1] == keyTagUID[i][1] &&
           rfid.uid.uidByte[2] == keyTagUID[i][2] && rfid.uid.uidByte[3] == keyTagUID[i][3]) 
        {
           accessGranted = true;
           break;
        }
      }

      if(accessGranted){
        // Cho phép truy cập 
        Serial.println("Access granted");
        lcd.clear();
        lcd.setCursor(2,1);
        lcd.print("ACCESS GRANTED");
        char uidString_char[30];
        sprintf(uidString_char, "%x%x%x%x", rfid.uid.uidByte[0], rfid.uid.uidByte[1], rfid.uid.uidByte[2], rfid.uid.uidByte[3]);
        char notification[]= "Correct UID TAG";
        strcat(notification, uidString_char);
        uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_CHECK_RFID, 1, true, notification);
        
      }
      else {
        // Từ chối truy cập
        Serial.print("Access denied");
        for (int i = 0; i < rfid.uid.size; i++) {
          Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");
          Serial.print(rfid.uid.uidByte[i], HEX);
        }
        lcd.clear();
        lcd.setCursor(2,1); 
        lcd.print("ACCESS DENIED");
        char uidString_char[30];
        sprintf(uidString_char, "%x%x%x%x", rfid.uid.uidByte[0], rfid.uid.uidByte[1], rfid.uid.uidByte[2], rfid.uid.uidByte[3]);
        char notification[]= "Incorrect UID TAG";
        strcat(notification, uidString_char);
        uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_CHECK_RFID, 1, true, notification);
      }

      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
  }
}
void registerRFID(){
  Serial.println("Number RFID you enroll?");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Number RFID:");
  numTags = readnumber();
  Serial.println(numTags);
  lcd.setCursor(14, 0);
  lcd.print(numTags);
  vTaskDelay(2000/portTICK_PERIOD_MS);

  for (int i =0;i<numTags;i++){
    lcd.clear();
    lcd.setCursor(0,0);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    lcd.print("Scan RFID to Reg");
    Serial.println("Scan RFID to Reg");
    lcd.setCursor(17,0);
    lcd.print(i+1);
    Serial.println(i+1);
    vTaskDelay(1000/portTICK_PERIOD_MS);
    while (keyTagUID[i][0] == 0 || keyTagUID[i][1] == 0 || keyTagUID[i][2] == 0 || keyTagUID[i][3] == 0)
    { // Block everything while waiting for a RFID TAG to register
      if (rfid.PICC_IsNewCardPresent()){  //new tag is available
        if (rfid.PICC_ReadCardSerial()){  //NUID has been readed
          MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
          keyTagUID[i][0] = rfid.uid.uidByte[0]; Serial.print(keyTagUID[i][0] < 0x10 ? " 0" : " "); Serial.print(keyTagUID[i][0], HEX);
          keyTagUID[i][1] = rfid.uid.uidByte[1]; Serial.print(keyTagUID[i][1] < 0x10 ? " 0" : " "); Serial.print(keyTagUID[i][1], HEX);
          keyTagUID[i][2] = rfid.uid.uidByte[2]; Serial.print(keyTagUID[i][2] < 0x10 ? " 0" : " "); Serial.print(keyTagUID[i][2], HEX);
          keyTagUID[i][3] = rfid.uid.uidByte[3]; Serial.print(keyTagUID[i][3] < 0x10 ? " 0" : " "); Serial.print(keyTagUID[i][3], HEX);
          // Display UID TAG on LCD 20x4
          lcd.setCursor(0, 2);
          lcd.print("UID:");

          lcd.setCursor(5, 2);
          lcd.print(keyTagUID[i][0] < 0x10 ? " 0" : " ");
          lcd.print(keyTagUID[i][0], HEX);

          lcd.setCursor(8, 2);
          lcd.print(keyTagUID[i][1] < 0x10 ? " 0" : " ");
          lcd.print(keyTagUID[i][1], HEX);

          lcd.setCursor(11, 2);
          lcd.print(keyTagUID[i][2] < 0x10 ? " 0" : " ");
          lcd.print(keyTagUID[i][2], HEX);

          lcd.setCursor(14, 2);
          lcd.print(keyTagUID[i][3] < 0x10 ? " 0" : " ");
          lcd.print(keyTagUID[i][3], HEX);      
          lcd.setCursor(0, 3);
          lcd.print("Save UID TAG");
          lcd.setCursor(13, 3);
          lcd.print(i+1);
          Serial.println();
          vTaskDelay(5000/portTICK_PERIOD_MS);
        }
        rfid.PICC_HaltA();       //halt PICC
        rfid.PCD_StopCrypto1();  //stop encryption on PCD
      }
    }
    String uidString;//để publish lên mqtt
    uidString = "";
    uidString += keyTagUID[i][0] < 0x10 ? " 0" : " ";
    uidString += String(keyTagUID[i][0], HEX);
    
    uidString += keyTagUID[i][1] < 0x10 ? " 0" : " ";
    uidString += String(keyTagUID[i][1], HEX);
    
    uidString += keyTagUID[i][2] < 0x10 ? " 0" : " ";
    uidString += String(keyTagUID[i][2], HEX);
    
    uidString += keyTagUID[i][3] < 0x10 ? " 0" : " "; 
    uidString += String(keyTagUID[i][3], HEX);
    //mqttClient.publish(MQTT_PUB_RFID, uidString.c_str());

    char uidString_char[30];
    sprintf(uidString_char, "%x%x%x%x", keyTagUID[i][0], keyTagUID[i][1], keyTagUID[i][2], keyTagUID[i][3]);
    uint16_t packetIdPubRFID = mqttClient.publish(MQTT_PUB_RFID, 1, true, uidString_char);
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i", MQTT_PUB_RFID, packetIdPubRFID);
    Serial.printf("RFID TAG Register:");
    Serial.printf(uidString_char);
    if(i<(numTags-1)){
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Next UID Register");
      vTaskDelay(3000/portTICK_PERIOD_MS);
    }
  }
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Register UID Done !");


  vTaskDelay(5000/portTICK_PERIOD_MS);
}