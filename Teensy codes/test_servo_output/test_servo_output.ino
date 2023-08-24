#include <Servo.h>

Servo myServo;
int servo_pin = 21;

void setup() {
  Serial.begin(38400);
  myServo.attach(servo_pin);
  Serial.print("Centering the servos...");
  myServo.write(90);
  
  Serial.println("Enter an angle.");
}

char rx_byte = 0;
String rx_str = "";
boolean not_number = false;
int result;

void loop() {
  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    
    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      rx_str += rx_byte;
    }
    else if (rx_byte == '\n') {
      // end of string
      if (not_number) {
        Serial.println("Not a number");
      }
      else {
        // multiply the number by 2
        result = rx_str.toInt();
        // print the result
        Serial.print(result);
        Serial.println("");
        Serial.println("Enter an angle.");
      }
      not_number = false;         // reset flag
      rx_str = "";                // clear the string for reuse
    }
    else {
      // non-number character received
      not_number = true;    // flag a non-number
    }
    myServo.write(result);
  } // end: if (Serial.available() > 0)
}
