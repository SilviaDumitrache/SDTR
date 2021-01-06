#define LED 13 //the led in connected to digital pin 13

void setup() {
  
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT); //digital pin set as output
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH); //led on
  delay(1000);             //wait 1s
  digitalWrite(LED, LOW);  //led off
  delay(1000);
}
