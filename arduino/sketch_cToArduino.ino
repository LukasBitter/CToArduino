/* Simple Serial ECHO script : Written by ScottC 03/07/2012 */

/* Use a variable called byteRead to temporarily store
 the data coming from the computer */
byte byteRead;
bool led_on = false;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)

void setup() {                
  // Turn the Serial Protocol ON
  Serial.begin(9600);
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

void loop() {
  // read the analog in value:
  sensorValue = analogRead(analogInPin);            
  // map it to the range of the analog out:
  outputValue = map(sensorValue, 0, 1023, 0, 255);  
  // change the analog out value:
  analogWrite(analogOutPin, outputValue);           

  // wait 2 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(2);  

  /*  check if data has been sent from the computer: */
  if (Serial.available()) {
    /* read the most recent byte */
    byteRead = Serial.read();
    if (byteRead == 49){
      led_on = true;
    }
    if(byteRead == 48){
      led_on = false;
    }
    if(byteRead == 50){
      if(outputValue < 10)
      {
        Serial.print(1);    
      }
      else
      {
        Serial.print(2);    
      }
    }
  }  

  if (led_on)
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  else
    digitalWrite(13, LOW);   // turn the LED off by making the voltage LOW
}

