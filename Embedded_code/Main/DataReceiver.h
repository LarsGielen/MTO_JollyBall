#ifndef DATARECEIVER_H
#define DATARECEIVER_H

#include <Arduino.h>
#include <ArduinoJson.h>

const size_t capacity = JSON_OBJECT_SIZE(2) + 30; // Adjusted for both "key" and "value"
char incomingData[128]; // Buffer to store incoming serial data
bool dataComplete = false;
byte bufferIndex = 0;

struct KeyMap {
  const char* key;
  float* ptr;
};

KeyMap keyMap[10]; // Allow up to 10 variables to be registered
uint8_t keyMapCount = 0;

void registerAdress(float* data, const char* key) {
  if (keyMapCount < 10) {
    keyMap[keyMapCount++] = { key, data };
  } 
  else {
    Serial.println("Key map full!");
  }
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
  if (dataComplete) {
    StaticJsonDocument<capacity> doc;
    DeserializationError error = deserializeJson(doc, incomingData);

    if (error) {
      Serial.print("Deserialization failed: ");
      Serial.println(error.c_str());
    } else {
      const char* key = doc["key"];
      float value = doc["value"];

      // change the registered adress data that is linked with the key to the new value
      for (uint8_t i = 0; i < keyMapCount; i++) {
        if (strcmp(keyMap[i].key, key) == 0) {
          *keyMap[i].ptr = value;
        }
      }
    }

    dataComplete = false; // Ready for next message
  }
}

#endif