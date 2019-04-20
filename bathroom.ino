#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#ifndef STASSID
#define STASSID "freekitties"
#define STAPSK  "kalogataki"
#endif

const char* ssid     = STASSID;
const char* password = STAPSK;

#define VERSION_MESSAGE F("Bathroom Console v0.10 18/04/19")

#define LEAK_PIN A0

#define AIO_SERVER      "192.168.2.20"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "mosquitto"
#define AIO_KEY         "qq211"
#define WILL_FEED AIO_USERNAME "/feeds/nodes.bathroom"
#define SERVER_LISTEN_PORT 80
#define MQTT_CONNECT_RETRY_MAX 5
#define MQTT_PING_INTERVAL_MS 60000
#define SENSOR_READ_INTERVAL_MS 5000

byte mac[] = {0xBE, 0xBD, 0xFA, 0xAB, 0xCD, 0xEF};
//IPAddress iotIP (192, 168, 0, 103);

uint32_t lastPing = 0; // timestamp
uint32_t connectedSince = 0; // timestamp
uint32_t now = 0; // timestamp
uint32_t nextConnectionAttempt = 0; // timestamp
uint32_t failedConnectionAttempts = 0;
uint32_t lastSensorRead = 0;

int lastState[20] = {0};


WiFiServer server(SERVER_LISTEN_PORT);
// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;
// or... use WiFiFlientSecure for SSL
//WiFiClientSecure client;

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);
Adafruit_MQTT_Publish lastwill = Adafruit_MQTT_Publish(&mqtt, WILL_FEED);
Adafruit_MQTT_Publish leak = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/leak.bathroom");

Adafruit_MQTT_Subscribe bathroomlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.bathroomlight");
//Adafruit_MQTT_Subscribe shedlight = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/toggle.shedlight");

/*************************** Sketch Code ************************************/

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

int lastSensor;

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

  lastSensor = analogRead(LEAK_PIN);
}

// the loop function runs over and over again forever
void loop() {
  now = millis();
  //Ethernet.maintain();
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &bathroomlight) {
      Serial.print(F("Got: "));
      Serial.println((char *)bathroomlight.lastread);
    }
  }

  /*
  // Now we can publish stuff!
  Serial.print(F("\nSending photocell val "));
  Serial.print(x);
  Serial.print("...");
  if (! photocell.publish(x++)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }*/

  

  bool pinged = MQTT_ping();

  if (lastSensorRead + SENSOR_READ_INTERVAL_MS < now) {
    int sensor = analogRead(LEAK_PIN);
    bool sensorChanged = abs(lastSensor - sensor) > 100;

    lastSensorRead = now;

    if (pinged || sensorChanged) {
      leak.publish(sensor);
    }
  }
  
  delay(1000);
}

bool MQTT_ping() {

  if (!mqtt.connected()) {
    return false;
  }

  if (lastPing + MQTT_PING_INTERVAL_MS < now) {
    Serial.println(F("Ping"));
    lastPing = now;
    if (!mqtt.ping()) {
      Serial.println(F("Failed to ping"));
      mqtt.disconnect();
    } else {
      lastwill.publish(now);
      return true;
    }
  }

  return false;
}

void MQTT_connect() {
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
