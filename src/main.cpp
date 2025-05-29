#include <Arduino.h>
#include "appsetup.h" 


void setup() {
   setupApp() ; 
}

void loop() {
  delay(1000);
  Serial.println("Hello World");
}
