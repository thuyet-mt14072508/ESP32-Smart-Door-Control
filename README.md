ESP32 Smart Door Lock\n
This project implements a smart door lock system using an ESP32 board. It supports three methods for access control - password via keypad, fingerprint scanner, and RFID tags.

Features
Register and store passwords
Enroll fingerprints
Register RFID tags
Grant/deny access via keypad password
Grant/deny access via fingerprint match
Grant/deny access via registered RFID tag
Publish access notifications to MQTT broker
LCD screen for status and notifications
Components
ESP32 DevKit Board
LCD 20x4 I2C display
Matrix membrane keypad
Optical fingerprint sensor (GT-521F32)
MFRC522 RFID reader
Connected to local WiFi and MQTT broker
Usage
On startup, the device allows registration of passwords, fingerprints, and RFID tags. After registrations are complete, it monitors the keypad, fingerprint scanner, and RFID reader to grant/deny access and publish notifications to the MQTT broker.

The LCD screen displays status messages and access notifications.

See the code comments for more implementation details.

Libraries Used
LiquidCrystal I2C
Adafruit Fingerprint Sensor
MFRC522 RFID
Arduino Json
AsyncMqttClient
To Do
Add servo to simulate door lock control
Save registered data to SPIFFS or EEPROM
Implement BLE instead of WiFi+MQTT
3D print enclosure
Let me know if you have any other questions!
