#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

//#include<HardwareSerial.h>
//#define SEMAPHORE_H
//#ifndef INC_FREERTOS_H
//#endif

#define BLYNK_PRINT Serial
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
BlynkTimer timer;


//char auth[] = "HmjJ7MVLfcpCdEl9-TwEiFVn0qsWpKE7";
char auth[] = "i0aj3JBtL9rRhnOKCH15jjJmlIWOul2h";
char ssid[] = "DIGI_066b80";
char pass[] = "aa21f994";

/*---------------------- Variables ---------------------*/
const int PulseWire = 34;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED16 = 16;          // The on-board Arduino LED, close to PIN 13.


/*---------------------- Tasks definition---------------------*/
//define two tasks: blink & analog read
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);

SemaphoreHandle_t xSerialSemaphore;       //used to ensure that only one task is accessing the resource at any time

/*---------------------- Setup ---------------------*/
//setup for reset/power on the board
void setup() {
  
  Serial.begin(9600); //serial communication initialized at 9600 bits per seconds
  Blynk.begin(auth, ssid, pass);
  pinMode(LED16, OUTPUT);
  pinMode(PulseWire, INPUT);

  while (!Serial) {
    ; // wait for serial port to connect
  } 
   if (xSerialSemaphore == NULL){                //check that the serial semaphore has not already been created
    xSerialSemaphore = xSemaphoreCreateMutex();  //create a mutex semaphore, used to manage serial port
    if ((xSerialSemaphore) != NULL)
    xSemaphoreGive((xSerialSemaphore));          //serial port available to use
    }
    
 xTaskCreatePinnedToCore(
                    TaskBlink,  //task function
                    "Blink",    //name of the task
                    1024,       //stack size (words)
                    NULL,       //parameter -> input of the task
                    1,          //priority of task
                    NULL,     
                    ARDUINO_RUNNING_CORE);                           

    
  xTaskCreatePinnedToCore(
                    TaskAnalogRead, //task function
                    "Read", //name of the task
                    10000,  //stack size (words)
                    NULL, //parameter -> input of the task
                    2,  //priority of task
                    NULL,
                    ARDUINO_RUNNING_CORE);

}

//The task scheduler is automatically started
void loop()
{  
}

/*---------------------- Tasks ---------------------*/
void TaskBlink(void *pvParameters)  
{
  (void) pvParameters;

  pinMode(LED16, OUTPUT);
 for(;;) //never return or exit
  {
  // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.  
  if (xSemaphoreTake (xSerialSemaphore, (TickType_t) 5) == pdTRUE){
  Serial.println("Led blinking");
  digitalWrite(LED16, HIGH);                //led on
  vTaskDelay( 1000 / portTICK_PERIOD_MS );  //wait 1sec  
  digitalWrite(LED16, LOW);                 //led off
  vTaskDelay( 1000 / portTICK_PERIOD_MS );  //wait 1sec
  }
    xSemaphoreGive(xSerialSemaphore);       //give serial port to others
    vTaskDelay(10);  // one tick delay (10ms) in between reads for stability
  }
}

void TaskAnalogRead(void *pvParameters)  
{
 (void) pvParameters;
 pinMode(PulseWire, INPUT);
 
  for (;;)
  {
  int sensorValue = analogRead(PulseWire);
  Blynk.virtualWrite(V1, sensorValue);
  Serial.println(sensorValue);
  if (xSemaphoreTake (xSerialSemaphore, (TickType_t) 100) == pdTRUE){
  if (sensorValue > 4000)
  {
  Blynk.notify("Pulse alert!"); 
  }
 }
   xSemaphoreGive(xSerialSemaphore);
   vTaskDelay(10);
}
}