#include <ArduinoJson.h>

const size_t capacity = JSON_OBJECT_SIZE(2) + 30; // Adjusted for both "key" and "value"
char incomingData[128]; // Buffer to store incoming serial data
bool dataComplete = false;
byte index = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); // Enable built-in LED control
  Serial.println("Ready to receive JSON...");
}

void loop() {
  // Read serial data into buffer
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { // End of JSON message
      incomingData[index] = '\0'; // Null-terminate the string
      dataComplete = true;
      index = 0;
    } else {
      if (index < sizeof(incomingData) - 1) {
        incomingData[index++] = ch;
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
      const char* value = doc["value"];

      if (strcmp(key, "led") == 0) {
        if (strcmp(value, "on") == 0) {
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("LED turned ON");
        } else if (strcmp(value, "off") == 0) {
          digitalWrite(LED_BUILTIN, LOW);
          Serial.println("LED turned OFF");
        } else {
          Serial.println("Unknown LED value");
        }
      } else {
        Serial.println("Unknown key");
      }
    }

    dataComplete = false; // Ready for next message
  }
}
