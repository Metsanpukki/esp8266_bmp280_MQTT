
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>

// Update these with values suitable for your network.

const char* ssid = "YOURSSID";
const char* password = "YOURPASSWORD";
const char* mqtt_server = "YOURSERVER";
const unsigned int mqtt_server_port = 11883; //default 1883
const char* mqtt_topic = "/garage/downstairs/sensors/bmp280/1";

WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_BMP280 bmp;

unsigned int updateInterval = 60000; //update interval in milliseconds
long lastMsg = 0;
char msg[255];
int value = 0;
int blinkSpeed = 2046;
long lastBlink = 0;
boolean ledStatus = true;

float temperature;
float pressure;
float altitude;

void setup() {
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  pinMode(D3, OUTPUT);
  digitalWrite(D3, LOW); 
  delay(1000);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_server_port);
  client.setCallback(callback);
  client.subscribe("garage/downstairs/lights");
    if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor");
    while (1);
  }
}

void readSensors()
{
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude();  
}

void setup_wifi() {

  delay(10);
  // connect
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if ((char)payload[10] == '1') {
    digitalWrite(D3, HIGH);
  }
    else
    {
      digitalWrite(D3,LOW);
    }
  } 


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > updateInterval) {
    lastMsg = now;
    readSensors();
    snprintf (msg, 255, "{\"temperature\":%.1f, \"pressure\":%.1f, \"altitude\":%.1f}", temperature,pressure/100,altitude);
    Serial.print("Publish message: ");
    Serial.println(msg);
    client.publish(mqtt_topic,msg);
    client.subscribe("garage/downstairs/lights");
    
  }
  
}
