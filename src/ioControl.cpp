#include "app.h" 


void cycleRGBOnce() {


 for (int i = 0; i <= 255; i += 5) {
    analogWrite(LED_RED_PIN, 255 - i);
    analogWrite(LED_GREEN_PIN, i);
    analogWrite(LED_BLUE_PIN, 0);
    delay(200);
  }

  for (int i = 0; i <= 255; i += 5) {
    analogWrite(LED_RED_PIN,  0 );
    analogWrite(LED_GREEN_PIN, 255 - i );
    analogWrite(LED_BLUE_PIN, i );
    delay(200);
  }

  for (int i = 0; i <= 255; i += 5) {
    analogWrite(LED_RED_PIN, i);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 255-i );
    delay(200);
  }

    analogWrite(LED_RED_PIN, 0);
    analogWrite(LED_GREEN_PIN, 0);
    analogWrite(LED_BLUE_PIN, 0);

}

void ledRGB()
{
   pinMode(LED_RED_PIN , OUTPUT) ; 
   pinMode(LED_GREEN_PIN , OUTPUT) ;  
   pinMode(LED_BLUE_PIN , OUTPUT) ; 
   pinMode(MOTOR_EN_PIN , OUTPUT) ; 

   digitalWrite(MOTOR_EN_PIN ,HIGH) ; 
   delay(500) ; 
   digitalWrite(MOTOR_EN_PIN, LOW); 

   cycleRGBOnce() ; 
}


