/*
 * https://circuits4you.com
 * ESP32 LED Blink Example
 * Board ESP23 DEVKIT V1
 * 
 * ON Board LED GPIO 2
 */

#include "Arduino.h"

#define LED 4
#define START_PIN   5
#define STOP_PIN    6

void setup() {
    // Set pin mode
    pinMode(1,OUTPUT);
    // pinMode(2,OUTPUT);
    // pinMode(3,OUTPUT);
    // pinMode(4,OUTPUT);
    // int i = 0;
    // for (i=0; i <= 3; ++i)
    // {
    //     pinMode(i,OUTPUT);
    // }
}

void loop() {
//   int i = 0;
//   for (i=START_PIN; i <= STOP_PIN; ++i)
//   {
//     delay(100);
//     digitalWrite(i,HIGH);
//     delay(700);
//     digitalWrite(i,LOW);

//   }
    digitalWrite(1, HIGH);
    delay(2000);
    digitalWrite(1, LOW);
    delay(200);

}