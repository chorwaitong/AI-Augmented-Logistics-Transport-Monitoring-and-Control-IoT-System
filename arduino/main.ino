#include "ArduinoGraphics.h"           // Required for LED drawing/text
#include "Arduino_LED_Matrix.h"
#include "DHT.h"
#include "Servo.h"
#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>


#define PN532_SS   (10)
#define PN532_MOSI (11)
#define PN532_MISO (12)
#define PN532_SCK  (13)

#define DHTPIN 2        // Digital pin connected to the DHT22
#define DHTTYPE DHT22   // DHT 22 (AM2302)
#define TRIG_PIN 3                    // Pin for trigger signal
#define ECHO_PIN 4                    // Pin for echo signal
#define SERVO_PIN 6
#define SERVO_OPEN 0
#define SERVO_CLOSE 90

// WiFi credentials (replace with your own)
const char* ssid = "SSID here";
const char* password = "Password here";

// MQTT settings
const char* mqtt_broker = "broker.emqx.io";
const int mqtt_port = 1883;
const char* mqtt_topic = "display/char";  // Topic to subscribe to for character display
const char* mqtt_topic_temp = "r4_cwt/temperature";    // Topic for temperature
const char* mqtt_topic_hum = "r4_cwt/humidity";        // Topic for humidity
const char* mqtt_topic_dist = "r4_cwt/dist";        // Topic for ultrasonic distance
const char* mqtt_topic_nfc = "r4_cwt/nfc";        // Topic for NFC
const char* mqtt_topic_servo = "r4_cwt/servo"; 

DHT dht(DHTPIN, DHTTYPE);
ArduinoLEDMatrix matrix;
Servo servo;
WiFiClient espClient;
MqttClient mqttClient(espClient);
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

int servoAngle = 0;                 // Current servo angle
int servoStep = 5;                  // Step size for servo movement
unsigned long previousMillis = 0;
const long interval = 2000;         // 2 seconds scan time
const long resetTimeNFC = 2000;     // 2s reset time for NFC reader
unsigned long lastNFCScanTime = 0;  // Time when last card was read
String lastUID = "";                // Stores last detected UID
unsigned long lastMqttPoll = 0;
const unsigned long mqttPollInterval = 500;  // Check MQTT every 100ms

void setup() {
  pinMode(TRIG_PIN, OUTPUT);         // Set trigger pin as output
  pinMode(ECHO_PIN, INPUT);          // Set echo pin as input
  servo.attach(SERVO_PIN);
  servo.write(SERVO_CLOSE);

  Serial.begin(115200);  // Match Flask baud rate
  dht.begin();
  matrix.begin();                      // Initialize LED matrix
  nfc.begin();

  while (!Serial) {
    delay(10);  // Wait for serial to initialize
  }
  Serial.println("Arduino UNO R4 WiFi Program Started");

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
  }else{
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  }
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    delay(2000);
  }
  Serial.println("\nConnected to WiFi");
  connectToMQTT();
}

void loop() {
  //Check NFC
  checkNFC();

  //Check for MQTT messages (servo)
  checkMqtt();

  // Log temperature and humidity every 2 seconds
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    logSensorData();
  }

}
void checkNFC() {
  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 }; // UID array
  uint8_t uidLength;

  unsigned long currentMillis = millis();
  if (currentMillis - lastNFCScanTime < resetTimeNFC){
    return;
  }else{
    lastUID = "";
  }

  success = nfc.inListPassiveTarget();
  if (success) {
    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength)) {
      String uidString = "";
      for (uint8_t i = 0; i < uidLength; i++) {
          uidString += String(uid[i], HEX) + " ";
          if (i < uidLength - 1) uidString += " ";
      }
      lastNFCScanTime = currentMillis;
      if (uidString != lastUID){
        lastUID = uidString;        
        Serial.print("NFC Card UID: ");
        Serial.println(uidString);
        mqttClient.beginMessage(mqtt_topic_nfc);
        mqttClient.print(uidString);
        mqttClient.endMessage();
      }
    }
  }
}

void connectToMQTT() {
  Serial.print("Connecting to MQTT...");
  String clientId = "ArduinoR4WiFi";
  mqttClient.subscribe(mqtt_topic_servo);
  //mqttClient.onMessage(onMessage);
  while (!mqttClient.connect(mqtt_broker, mqtt_port)) {
    Serial.print("\nMQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    Serial.print("\nReattempt in 5s");
    delay(5000);
  }
  if (mqttClient.subscribe(mqtt_topic_servo)) {
      Serial.println("Subscribed to topic: " + String(mqtt_topic_servo));
  } else {
      Serial.println("Failed to subscribe to: " + String(mqtt_topic_servo));
  }
  Serial.print("MQTT Connected!");
}

void checkMqtt() {
    if (millis() - lastMqttPoll >= mqttPollInterval) {
        lastMqttPoll = millis();  // Update the last check time

        if (mqttClient.parseMessage()) {  // Check if a new message is available
            String message = mqttClient.readString();
            Serial.println("Received MQTT message: " + message);
            servo.write(message.toInt()); 
            /*if (message == "1") {
                servo.write(SERVO_OPEN);  // Open servo
                Serial.println("Servo opened");
            } else if (message == "0") {
                servo.write(SERVO_CLOSE);  // Close servo
                Serial.println("Servo closed");
            }*/
        }
    }
}


void onMessage(int messageSize) {
  String message = "";
  
  // Read the message payload
  while (mqttClient.available()) {
    message += (char)mqttClient.read();
  }

  Serial.print("Received message: ");
  Serial.println(message);
  //servo.write(message.toInt()); 
  servo.write(SERVO_OPEN);
  /*if (message == "1") {
    // Open the servo (move to open position)
    servo.write(SERVO_OPEN);  // Move servo to open position (0 degrees)
  } else if (message == "0") {
    Serial.println("Closing the servo");
    servo.write(SERVO_CLOSE);  // Move servo to close position (90 degrees)
  } else {
    Serial.println("Invalid command received. Servo not moved.");
  }*/
}

void logSensorData() {
  //Ultrasonic sensor measurement
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);       // Send a 10µs pulse to trigger
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);  // Time in microseconds
  float distance_cm = duration * 0.034 / 2; // Convert to cm

  // Read temperature and humidity
  char tempStr[8];
  char humStr[8];
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  // Check for errors
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    dtostrf(0, 6, 2, tempStr);  // 6 chars total, 2 after decimal
    dtostrf(0, 6, 2, humStr);
    mqttClient.beginMessage(mqtt_topic_temp);
    mqttClient.print("ERROR");
    mqttClient.endMessage();

    mqttClient.beginMessage(mqtt_topic_hum);
    mqttClient.print("ERROR");
    mqttClient.endMessage();
  }else{
    dtostrf(t, 6, 2, tempStr);  // 6 chars total, 2 after decimal
    dtostrf(h, 6, 2, humStr);
    mqttClient.beginMessage(mqtt_topic_temp);
    mqttClient.print(tempStr);
    mqttClient.endMessage();
    delay(100);
    mqttClient.beginMessage(mqtt_topic_hum);
    mqttClient.print(humStr);
    mqttClient.endMessage();
    delay(100);
  }

  char distStr[8];
  dtostrf(distance_cm, 6, 2, distStr);
  mqttClient.beginMessage(mqtt_topic_dist);
  mqttClient.print(distStr);
  mqttClient.endMessage();

  // Log to Serial Monitor
  Serial.print("Temperature: ");
  Serial.print(tempStr);
  Serial.print(" °C, Humidity: ");
  Serial.print(humStr);
  Serial.print(" %, Distance: ");
  Serial.print(distStr);
  Serial.println(" cm");
}
