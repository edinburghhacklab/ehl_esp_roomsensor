#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

extern "C" {
#include "user_interface.h"
}

#include "config.h"

#define MAX_TOPIC_LENGTH 50
#define MAX_PAYLOAD_LENGTH 10

const int onewire_pin = 5;
const int pir_pin = 14;

char my_id[40];
char mqtt_id[24];
char mqtt_topic[50];

WiFiClient client;
PubSubClient mqtt(client);
OneWire onewire(onewire_pin);
DallasTemperature sensors(&onewire);
DeviceAddress device_address;
int device_count = 0;

unsigned long last_temperature_read = 0;
unsigned long last_pir_send = 0;

void hexlify(uint8_t bytes[], uint8_t len, char *buffer)
{
  for (int i=0; i<len; i++) {
    int ms = bytes[i] >> 4;
    int ls = bytes[i] & 0x0f;
    char a, b;
    if (ms < 10)
      a = '0' + ms;
    else
      a = 'a' + ms - 10;
    if (ls < 10)
      b = '0' + ls;
    else
      b = 'a' + ls - 10;
    //printf("byte %d is %02x (%x %x %c%c)\n", i, bytes[i], ms, ls, a, b);
    buffer[i*2] = a;
    buffer[(i*2)+1] = b;
  }
  buffer[(len*2)] = 0;
}


void callback(const MQTT::Publish& pub)
{
  mqtt.publish("meta/mqtt-agents/reply", my_id);
}

void setup()
{
  char device_address_string[17];

  if (pir_active_low) {
    pinMode(pir_pin, INPUT_PULLUP);
  } else {
    pinMode(pir_pin, INPUT);
  }

  snprintf(my_id, sizeof(my_id), "roomsensor-%06x-%s", ESP.getChipId(), room_id);
  snprintf(mqtt_id, sizeof(mqtt_id), "%08x", ESP.getChipId());

  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.print(my_id);
  Serial.print(" ");
  Serial.println(ESP.getSketchMD5());

  if (ota_enabled) {
    Serial.println("Enabling OTA updates");
    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(my_id);
    if (strlen(ota_password) > 0) {
      Serial.print("OTA password: ");
      Serial.println(ota_password);
      ArduinoOTA.setPassword(ota_password);
    }
    ArduinoOTA.onStart([]() {
      Serial.println("OTA Start");
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nOTA End");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("OTA Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    ArduinoOTA.begin();
  }

  sensors.begin();
  device_count = sensors.getDeviceCount();
  if (device_count > 0) {
    for (int i=0; i<device_count; i++) {
      if (sensors.getAddress(device_address, i)) {
        Serial.print("found sensor ");
        hexlify(device_address, 8, device_address_string);
        Serial.println(device_address_string);
        sensors.setResolution(device_address, 10);
      }
    }
  } else {
    Serial.println("no sensors found");
  }

  wifi_station_set_hostname(my_id);
  WiFi.mode(WIFI_STA);

  mqtt.set_server(mqtt_server, mqtt_port);
  mqtt.set_callback(callback);
}

void loop()
{

  if (ota_enabled) {
    ArduinoOTA.handle();
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to ");
    Serial.print(ssid);
    Serial.println("...");
    WiFi.begin(ssid, password);

    unsigned long begin_started = millis();
    while (WiFi.status() != WL_CONNECTED) {
      delay(10);
      if (millis() - begin_started > 60000) {
        ESP.restart();
      }
    }
    Serial.println("WiFi connected");
  }

  if (!mqtt.connected()) {
    if (mqtt.connect(mqtt_id)) {
      Serial.println("MQTT connected");
      mqtt.subscribe("meta/mqtt-agents/poll");
    } else {
      Serial.println("MQTT connection failed");
      delay(2000);
      return;
    }
  }

  mqtt.loop();

  if (last_temperature_read == 0 || millis() - last_temperature_read > temperature_interval) {
    last_temperature_read = millis();
    //Serial.println("Reading temperatures...");
    sensors.requestTemperatures();
    for(int i=0; i<device_count; i++) {
      if (sensors.getAddress(device_address, i)) {
        char device_address_string[17];
        char mqtt_topic[MAX_TOPIC_LENGTH];
        char mqtt_payload[MAX_PAYLOAD_LENGTH];
        float temperature = sensors.getTempC(device_address);
        hexlify(device_address, 8, device_address_string);
        snprintf(mqtt_topic, sizeof(mqtt_topic), mqtt_temperature_topic, device_address_string);
        dtostrf(temperature, 0, 2, mqtt_payload);
        Serial.print(mqtt_topic);
        Serial.print(" ");
        Serial.println(mqtt_payload);
        mqtt.publish(mqtt_topic, (const uint8_t*)mqtt_payload, strlen(mqtt_payload), true);
      }
    }
  }

  if ((!pir_active_low && digitalRead(pir_pin) == HIGH) || (pir_active_low && digitalRead(pir_pin) == LOW)) {
    if (millis() - last_pir_send > pir_interval) {
      Serial.println(mqtt_pir_topic);
      mqtt.publish(mqtt_pir_topic, "");
      last_pir_send = millis();
    }
  }

}
