#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
BlynkTimer timer;

char auth[] = "HmjJ7MVLfcpCdEl9-TwEiFVn0qsWpKE7";
char ssid[] = "DIGI_066b80";
char pass[] = "aa21f994";

/*---------------------- Variables ---------------------*/
const int PulseWire = 34;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED16 = 16;          // The on-board Arduino LED, close to PIN 13.
//int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.

int sensorValue = 0;         // the sensor value
int sensorMin = 1023;        // minimum sensor value
int sensorMax = 0;           // maximum sensor value

//declaration of a mutex semaphore handle

/*---------------------- Tasks definition---------------------*/
//define two tasks: blink & analog read
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);

SemaphoreHandle_t xSerialSemaphore;   //used to ensure that only one task is accessing the resource at any time

/*---------------------- Setup ---------------------*/
//setup for reset/power on the board
 void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //serial communication initialized at 9600 bits per seconds
  Blynk.begin(auth, ssid, pass);
  pinMode(LED16, OUTPUT);
  pinMode(PulseWire, INPUT);
  
  while(!Serial) {
    ;//wait for serial port to connect
  }

 if (xSerialSemaphore == NULL)                   //check that the serial semaphore has not already been created
  {
    xSerialSemaphore = xSemaphoreCreateMutex();   //create a mutex semaphore, used to manage serial port
    if ((xSerialSemaphore) != NULL)
      xSemaphoreGive ((xSerialSemaphore));        //serial port available to use
  }

  //Configuration of the PulseSensor (variables are assigned)
//  pulseSensor.analogInput(PulseWire);   
//  pulseSensor.blinkOnPulse(LED16);       //auto-magically blink Arduino's LED with heartbeat.
//  pulseSensor.setThreshold(Threshold);     

//Check if the pulse sensor object was created. If yes => we see a signal. 
//   if (pulseSensor.begin()) {
//    Serial.println("PulseSensor Object successfuly created!");  //This prints one time at Arduino power-up,  or on Arduino reset.  
//  }

  while (millis() < 5000) {
    sensorValue = analogRead(PulseWire);

    // record the maximum sensor value
    if (sensorValue > sensorMax) {
      sensorMax = sensorValue;
    }

    // record the minimum sensor value
    if (sensorValue < sensorMin) {
      sensorMin = sensorValue;
    }
  }

//setup tasks to run independently
  xTaskCreatePinnedToCore( 
                TaskBlink,      //task function
                "Blink",        //name of the task
                1024,            //stack size (words)
                NULL,           //parameter -> input of the task
                1,              //priority of task
                NULL,
                ARDUINO_RUNNING_CORE);          //task handle (reference the task from ouside the execution



  xTaskCreatePinnedToCore( 
                TaskAnalogRead,      //task function
                "Read",              //name of the task
                10000,                 //stack size (words)
                NULL,                //parameter -> input of the task
                2,                   //priority of task
                NULL,
                ARDUINO_RUNNING_CORE);               //task handle (reference the task from ouside the execution

}

//The task scheduler is automatically started

void loop() {
  //is empty => things are done in tasks
}



/*---------------------- Tasks ---------------------*/
void TaskBlink( void *pvParameters) {
  (void) pvParameters;

  //initialize led on pin 16
  #define LED 16 //the led in connected to digital pin 16
  pinMode(LED, OUTPUT);
  for(;;) //never return or exit
  {
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    Serial.println("Led blinking"); 
    digitalWrite(LED,HIGH);                 //led on
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1sec
    digitalWrite(LED,LOW);                  //led off
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1sec
    }
    xSemaphoreGive(xSerialSemaphore);       //give serial port to others
    vTaskDelay(10);    // one tick delay (15ms) in between reads for stability
  }
}


void TaskAnalogRead(void *pvParameters) {
  (void) pvParameters;
    for (;;)
  {
if (xSemaphoreTake (xSerialSemaphore, (TickType_t) 100) == pdTRUE){
    int sensorValue = analogRead(PulseWire);
    sensorValue = map(sensorValue, sensorMin, sensorMax, 0, 255);
    // in case the sensor value is outside the range seen during calibration
    sensorValue = constrain(sensorValue, 0, 255);
    Blynk.virtualWrite(V1, sensorValue);
    Serial.println(sensorValue);

    xSemaphoreGive(xSerialSemaphore);
    vTaskDelay(10);
  }
 }
}
 