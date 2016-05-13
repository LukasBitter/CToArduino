/* Simple Serial ECHO script : Written by ScottC 03/07/2012 */

/* Use a variable called byteRead to temporarily store
   the data coming from the computer */
byte byteRead;
bool led_on = false;

void setup() {                
  // Turn the Serial Protocol ON
  Serial.begin(9600);
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
}

void loop() {
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
    /*ECHO the value that was read, back to the serial port. */
    Serial.write(byteRead);
  }  
  if (led_on)
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  else
    digitalWrite(13, LOW);   // turn the LED off by making the voltage LOW
}
