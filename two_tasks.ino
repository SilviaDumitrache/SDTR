#include <Arduino_FreeRTOS.h>
#define USE_ARDUINO_INTERRUPTS true    // Set-up low-level interrupts for most acurate BPM math.
#include <PulseSensorPlayground.h>     // Includes the PulseSensorPlayground Library.   

/*---------------------- Variables ---------------------*/
const int PulseWire = 0;       // PulseSensor PURPLE WIRE connected to ANALOG PIN 0
const int LED13 = 13;          // The on-board Arduino LED, close to PIN 13.
int Threshold = 550;           // Determine which Signal to "count as a beat" and which to ignore.

PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"


/*---------------------- Tasks handlers---------------------*/
TaskHandle_t taskDeletedHandle;
TaskHandle_t TaskBlink_Handler;
TaskHandle_t TaskSerial_Handler;

/*---------------------- Tasks definition---------------------*/
//define two tasks: blink & analog read
void TaskBlink(void *pvParameters);
void TaskAnalogRead(void *pvParameters);
//void TaskDelete(void *pvParameters);


/*---------------------- Setup ---------------------*/
//setup for reset/power on the board
 void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600); //serial communication initialized at 9600 bits per seconds
  while(!Serial) {
    ;//wait for serial port to connect
  }

//Configuration of the PulseSensor (variables are assigned)
  pulseSensor.analogInput(PulseWire);   
//  pulseSensor.blinkOnPulse(LED13);       //auto-magically blink Arduino's LED with heartbeat.
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
                &TaskBlink_Handler);          //task handle (reference the task from ouside the execution
              


  xTaskCreate( 
                TaskAnalogRead,      //task function
                "Read",              //name of the task
                128,                 //stack size (words)
                NULL,                //parameter -> input of the task
                1,                   //priority of task
                &TaskSerial_Handler);               //task handle (reference the task from ouside the execution
              
              
}


void loop() {
  //is empty => things are done in tasks
}

//The task scheduler is automatically started



/*---------------------- Tasks ---------------------*/
void TaskBlink( void *pvParameters) {
  (void) pvParameters;

  //initialize led on pin 13
  #define LED 13 //the led in connected to digital pin 13
  pinMode(LED, OUTPUT);
  for(;;) //never return or exit
  {
    digitalWrite(LED,HIGH);                  //led on
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1sec
    digitalWrite(LED,LOW);                   //led off
    vTaskDelay(1000/portTICK_PERIOD_MS);    //wait 1sec
  }
}


void TaskAnalogRead(void *pvParameters) {
  (void) pvParameters;

  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = pulseSensor.getBeatsPerMinute(); //sensorValue holds the BPM value

    if (pulseSensor.sawStartOfBeat()){
      Serial.println("♥ HeartBeat found ! ");  //if test = true => print message
      Serial.println("BPM:");
      Serial.println(sensorValue);              //print out the value read
    }


    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}







