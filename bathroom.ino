#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <DHTesp.h>

#ifndef STASSID
#define STASSID "freekitties"
#define STAPSK  "kalogataki"
#endif

#define VERSION_MESSAGE F("Bathroom Console v0.12 28/08/19")

#define LEAK_PIN A0
#define MOTION_SENSOR_PIN D8
#define DHT_SENSOR_PIN D2
#define BUTTON0_PIN D3

#define AIO_SERVER      "192.168.2.20"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"
#define WILL_FEED AIO_USERNAME "/feeds/nodes.bathroom"
#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5
#define MQTT_PING_INTERVAL_MS 10000
#define LEAK_SENSOR_READ_INTERVAL_MS 5000
#define DHT_SENSOR_READ_INTERVAL_MS 30000

const char* ssid     = STASSID;
const char* password = STAPSK;

byte mac[] = {0xBE, 0xBD, 0xFA, 0xAB, 0xCD, 0xEF};

uint32_t lastPing = 0; // timestamp
uint32_t connectedSince = 0; // timestamp
uint32_t now = 0; // timestamp
uint32_t nextConnectionAttempt = 0; // timestamp
uint32_t failedConnectionAttempts = 0;
uint32_t lastSensorRead = 0;
uint32_t lastDhtSensorRead = 0;
int lastState[20] = {0};
int lastSensor;

DHTesp dht;

WiFiServer server(SERVER_LISTEN_PORT);
WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);
Adafruit_MQTT_Publish leak = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/leak.bathroom");
Adafruit_MQTT_Publish motion = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bathroom.motion");
Adafruit_MQTT_Publish temp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bathroom.temperature");
Adafruit_MQTT_Publish humid = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bathroom.humidity");
Adafruit_MQTT_Publish button0 = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/bathroom.button0");

Adafruit_MQTT_Subscribe bathroomlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.bathroomlight");

#define halt(s) { Serial.println(F( s )); while(1);  }

void(* __resetFunc) (void) = 0; //declare reset function @ address 0

void resetFunc(const __FlashStringHelper* msg, unsigned long delayMs) {
  Serial.println(msg);
  Serial.print(F("Resetting in "));
  Serial.print(delayMs / 1000);
  Serial.println(F("s"));
  delay(delayMs);
  __resetFunc();
}

void setup() {
  Serial.begin(115200);

  // We start by connecting to a WiFi network

  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
     would try to act as both a client and an access-point and could cause
     network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt.will(WILL_FEED, "0");

  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(BUTTON0_PIN, INPUT_PULLUP);
  dht.setup(DHT_SENSOR_PIN, DHTesp::DHT22);

  lastSensor = analogRead(LEAK_PIN);
  lastSensorRead = millis();
}

void readLeakSensor(bool force = false) {
  //Serial.println(now - lastSensorRead);
  if (force || now - lastSensorRead > LEAK_SENSOR_READ_INTERVAL_MS) {
    int sensor = analogRead(LEAK_PIN);
    bool sensorChanged = abs(lastSensor - sensor) > 100;

    lastSensorRead = now;

    if (sensorChanged || force) {
      Serial.println("Publishing Leak Sensor Value");
      leak.publish(sensor);
    }
  }
}

void readDht() {
  if (now - lastDhtSensorRead > DHT_SENSOR_READ_INTERVAL_MS) {
    float h = dht.getHumidity();
    float t = dht.getTemperature();
    temp.publish(t);
    humid.publish(h);


    Serial.print("{\"humidity\": ");
    Serial.print(h);
    Serial.print(", \"temp\": ");
    Serial.print(t);
    Serial.print("}\n");

    lastDhtSensorRead = now;
  }
}


void loop() {
  now = millis();
  //Ethernet.maintain();
  connectMqtt();

  mqtt.process(100);

  detectEdge(MOTION_SENSOR_PIN, &motion);
  detectEdge(BUTTON0_PIN, &button0);

  pingMqtt();

  readLeakSensor();
  readDht();
  
  delay(100);
}

void detectEdge(int pin, Adafruit_MQTT_Publish* feed) {
  int state = digitalRead(pin);
  if (state != lastState[pin]) {
    Serial.print("Publishing state change on pin ");
    Serial.print(pin);
    if (state == HIGH) {
      Serial.println(" high");
      feed->publish("1");
    } else {
      Serial.println(" low");
      feed->publish("0");
    }
  }

  lastState[pin] = state;
}

void onPing(bool result) {

  readLeakSensor(true);
  lastwill.publish(now);
}

void pingMqtt() {
  if (!mqtt.connected()) {
    return;
  }

  if (now - lastPing > MQTT_PING_INTERVAL_MS) {
    Serial.println(F("Ping"));
    lastPing = now;
    mqtt.pingAsync(&onPing);
  }
}

void connectMqtt() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  if (nextConnectionAttempt < now) {
    Serial.print(F("Connecting to MQTT... "));

    int delaySecs = (2 << failedConnectionAttempts); // Delay for 2, 4, 8 .. seconds
    if (ret = mqtt.connect() != 0) {
      Serial.print(F("Failed: "));
      Serial.println(mqtt.connectErrorString(ret));
      //mqtt.disconnect();

      nextConnectionAttempt = now + delaySecs * 1000;
      ++failedConnectionAttempts;
    }
  
    if (0 == ret) {
      connectedSince = millis();
      failedConnectionAttempts = 0;
      Serial.println(F("Connected!"));
    } else if (failedConnectionAttempts > MQTT_CONNECT_RETRY_MAX) {
      connectedSince = 0;
      resetFunc(F("Max retries exhausted!"), 2000); // Reset and try again
    } else {
      Serial.print(F("Retrying in "));

      Serial.print(delaySecs);
      Serial.println(F("s"));
    }
  }
}
