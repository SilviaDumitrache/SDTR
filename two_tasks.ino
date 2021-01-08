#include <Arduino_FreeRTOS.h>

//define two tasks: blink & analog read
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);

//setup for reset/power on the board
 void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); //serial communication initialized at 9600 bits per seconds
  while(!Serial) {
    ;//wait for serial port to connect
  }

//setup tasks to run independently
  xTaskCreate( 
                TaskBlink,      //task function
                "Blink",        //name of the task
                128,            //stack size (words)
                NULL,           //parameter -> input of the task
                2,              //priority of task
                NULL);          //task handle (reference the task from ouside the execution
              


  xTaskCreate( 
                TaskAnalogRead,      //task function
                "Read",              //name of the task
                128,                 //stack size (words)
                NULL,                //parameter -> input of the task
                1,                   //priority of task
                NULL);               //task handle (reference the task from ouside the execution
              
}

//The task scheduler is automatically started

void loop() {
  //is empty => things are done in tasks
}



/*---------------------- Tasks ---------------------*/
void TaskBlink( void *pvParameters) {
  (void) pvParameters;

  //initialize led on pin 13
  #define LED 13 //the led in connected to digital pin 13
  pinMode(LED, OUTPUT);
  for(;;) //never return or exit
  {
    digitalWrite(LED,HIGH);                 //led on
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1sec
    digitalWrite(LED,LOW);                  //led off
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1sec
  }
}


void TaskAnalogRead(void *pvParameters) {
  (void) pvParameters;

  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);

    // print out the value you read:
    Serial.println(sensorValue);

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
