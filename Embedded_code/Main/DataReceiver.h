#ifndef DATARECEIVER_H
#define DATARECEIVER_H

#include <Arduino.h>
#include <ArduinoJson.h>

const size_t capacity = JSON_OBJECT_SIZE(2) + 30; // Adjusted for both "key" and "value"
char incomingData[1024]; // Buffer to store incoming serial data
bool dataComplete = false;
byte bufferIndex = 0;

struct KeyMap {
  const char* key;
  float* ptr;
};

const int mapSize = 14;
KeyMap keyMap[mapSize]; 
uint8_t keyMapCount = 0;

void registerAdress(float* data, const char* key) {
  if (keyMapCount < mapSize) {
    keyMap[keyMapCount++] = { key, data };
  } 
  else {
    Serial.println("Key map full!");
  }
}

void updateKeyValue(const char* key, float value) {
  for (uint8_t i = 0; i < keyMapCount; i++) {
    if (strcmp(keyMap[i].key, key) == 0) {
      *keyMap[i].ptr = value;
      Serial.print("Updated: ");
      Serial.print(key);
      Serial.print(" -> ");
      Serial.println(value);
      return;
    }
  }
  Serial.print("Unknown key: ");
  Serial.println(key);
}

void receiveLoop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { // End of JSON message
      incomingData[bufferIndex] = '\0'; // Null-terminate the string
      dataComplete = true;
      bufferIndex = 0;
    } else {
      if (bufferIndex < sizeof(incomingData) - 1) {
        incomingData[bufferIndex++] = ch;
      }
    }
  }

  // If a complete message is received, parse it
  if (!dataComplete) return;
  dataComplete = false;

  StaticJsonDocument<capacity> doc;  // adjust capacity as needed
  DeserializationError err = deserializeJson(doc, incomingData);
  if (err) {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  if (doc.is<JsonObject>()) {
    const char* key = doc["key"];
    float value     = doc["value"];
    updateKeyValue(key, value);
  }
  else if (doc.is<JsonArray>()) {
    for (JsonObject elem : doc.as<JsonArray>()) {
      const char* key = elem["key"];
      float value     = elem["value"];
      updateKeyValue(key, value);
    }
  }
  else {
    Serial.println("Unexpected JSON format (not an object or array).");
  }
}

#endif