#include <Arduino_FreeRTOS.h>          //Freertos library
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   
#include <semphr.h>                    //add the FreeRTOS functions for semaphores (flags)

//tried to show data in blynk app

//#define BLYNK_PRINT DebugSerial
//#include <SoftwareSerial.h>
//SoftwareSerial DebugSerial(2, 3); // RX, TX
//#include <BlynkSimpleStream.h>
//char auth[] = "i0aj3JBtL9rRhnOKCH15jjJmlIWOul2h";

//BlynkTimer timer; //creat a timer

/*---------------------- Variables ---------------------*/
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 515; //550     // Determine which Signal to "count as a beat" and which to ignore.

PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"


//declaration of a mutex semaphore handle
SemaphoreHandle_t xSerialSemaphore;   //used to ensure that only one task is accessing the resource at any time

/*---------------------- Tasks handlers---------------------*/
//TaskHandle_t taskDeletedHandle;
//TaskHandle_t TaskBlink_Handler;
//TaskHandle_t TaskSerial_Handler;

/*---------------------- Tasks definition---------------------*/
//define two tasks: blink & analog read
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);


/*---------------------- Setup ---------------------*/
//setup for reset/power on the board
 void setup() {
  // put your setup code here, to run once:
//   DebugSerial.begin(9600);

//  SwSerial.begin(9600);

  Serial.begin(9600); //serial communication initialized at 9600 bits per seconds
//  Blynk.begin(Serial, auth);

//    Blynk.begin(auth);

//    timer.setInterval(1000);
  
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
  pulseSensor.analogInput(PulseWire);   
//  pulseSensor.blinkOnPulse(LED13);       //blink Arduino's LED with heartbeat.
  pulseSensor.setThreshold(Threshold);     

//Check if the pulse sensor object was created. If yes => we see a signal. 
   if (pulseSensor.begin()) {
    Serial.println("PulseSensor Object successfuly created!");  //This prints one time at Arduino power-up,  or on Arduino reset.  
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
                326,               //stack size (words)
                NULL,                //parameter -> input of the task
                1,                   //priority of task
                NULL);               //task handle (reference the task from ouside the execution
              


              //The task scheduler is automatically started
}

//void SendUptime(){
//  int sensorValue = pulseSensor.getBeatsPerMinute(); //sensorValue holds the BPM value
//  Blynk.virtualWrite(V1,sensorValue);
//  vTaskDelay(1000/portTICK_PERIOD_MS);     //wait 1sec
//}

void loop() {
  //is empty => things are done in tasks
//  Blynk.run();
//  timer.run();
}



/*---------------------- Tasks ---------------------*/
void TaskBlink( void *pvParameters) {
  (void) pvParameters;

  //initialize led on pin 13
  #define LED 13 //the led in connected to digital pin 13
  pinMode(LED, OUTPUT);

  for(;;) //never return or exit
  {
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ){
    Serial.println("Led blinking"); }
    
    digitalWrite(LED,HIGH);                  //led on
    vTaskDelay(1000/portTICK_PERIOD_MS);     //wait 1sec
    digitalWrite(LED,LOW);                   //led off
    vTaskDelay(1000/portTICK_PERIOD_MS);     //wait 1sec

    xSemaphoreGive(xSerialSemaphore);       //give serial port to others
    vTaskDelay(1);    // one tick delay (15ms) in between reads for stability
  }
}


void TaskAnalogRead(void *pvParameters) {
  (void) pvParameters;
 
//Blynk.virtualWrite(V1,sensorValue);
    
  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = pulseSensor.getBeatsPerMinute(); //sensorValue holds the BPM value

    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE ) {
   
    if (pulseSensor.sawStartOfBeat()){
      Serial.println("♥ HeartBeat found ! ");  //if test = true => print message
      Serial.println("BPM:");
      Serial.println(sensorValue);              //print out the value read

      //to show data in Blynk app    
//      vTaskDelay(1);    // one tick delay (15ms) in between reads for stability

//      BLYNK_READ(V5){
//        Blynk.virtualWrite(5,millis()/1000);
//      }
//      Blynk.virtualWrite(V1,sensorValue); //try 1
//
//      if (sensorValue > 40){
//        Blynk.email("silvia_d97@yahoo.com", "Alerta", "Pulse value over 40");
//        Blynk.notify("Alert");
//      }

// Blynk.virtualWrite(V1, millis()/1000);
      }
      xSemaphoreGive(xSerialSemaphore);       //give serial port to others
      vTaskDelay(1);    // one tick delay (15ms) in between reads for stability
    }
    vTaskDelay(1);    // one tick delay (15ms) in between reads for stability
  }
}