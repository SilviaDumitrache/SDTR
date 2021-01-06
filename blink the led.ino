#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#define LED 13 //the led in connected to digital pin 13

void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT); //digital pin set as output

  xTaskCreate (red, "RED", 100, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  // idle task

}

void red(void *pvParameters)
{
  while(1)
  {
    digitalWrite(LED, HIGH); //led on
    delay(1000);             //wait 1s
    digitalWrite(LED, LOW);  //led off
    delay(1000);
  }
}