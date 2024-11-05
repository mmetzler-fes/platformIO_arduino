#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Adafruit_MPU6050.h>

// Taster Pins
const int taster1_pin = 10;
const int taster2_pin = 11;

// WIFI
//const char* ssid = "dummy";
//const char* password = "dummy";
const char* ssid = "TP-Link_E635"; 
const char* password = "35189213"; 

// MQTT
const char* mqttServer = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttTopic = "mmetzler/sensor/mpu";

// MPU6050
Adafruit_MPU6050 mpu;

WiFiClient espClient;
PubSubClient client(espClient);

void connect() {
  while (!client.connect("esp32MQTT")) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected!");
  client.subscribe("nodemcu");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Verbinden mit ");
  Serial.println(ssid);

  // Verbindung herstellen
  WiFi.begin(ssid, password);

  // WIFI prüfen 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi verbunden..!");
  Serial.print("IP= ");  Serial.println(WiFi.localIP());

  // MQTT
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    connect();
  }
  
  Wire.begin(8,9);

  // Try to initialize!
  while (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    delay(1000);
  }
  Serial.println("MPU6050 Found!");
  
  // Setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  // Taster-Pins konfigurieren
  pinMode(taster1_pin, INPUT_PULLDOWN); 
  pinMode(taster2_pin, INPUT_PULLDOWN);
}

void loop() {
  // init JsonBuffer
  DynamicJsonDocument doc(1024);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
    
  doc["Ax"] = a.acceleration.x;
  doc["Ay"] = a.acceleration.y;
  doc["Az"] = a.acceleration.z;
  doc["T"] = temp.temperature;
  doc["Gx"] = g.gyro.x;
  doc["Gy"] = g.gyro.y;
  doc["Gz"] = g.gyro.z;
  doc["player"] = 1;
  doc["taster1"] = digitalRead(taster1_pin);
  doc["taster2"] = digitalRead(taster2_pin);

  Serial.print("Ax: "); Serial.print(a.acceleration.x);
  Serial.print(" Ay: "); Serial.print(a.acceleration.y);
  Serial.print(" Az: "); Serial.print(a.acceleration.z);
  Serial.print(" T: "); Serial.print(temp.temperature);
  Serial.print(" Gx: "); Serial.print(g.gyro.x);
  Serial.print(" Gy: "); Serial.print(g.gyro.y);
  Serial.print(" Gz: "); Serial.println(g.gyro.z);

  char out[512];
  serializeJson(doc, out);
  
  if (client.publish(mqttTopic, out)) {
    Serial.println("Success sending message");
  } else {
    Serial.println("Error sending message");
  }
  
  client.loop();
  delay(100);
}
