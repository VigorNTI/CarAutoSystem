#include <Wire.h>
#include <Servo.h>

Servo servo;

/*
 * Function: setup
 * --------------------
 * Setting up the wire protocol with an address to communicate with the servo
 * also sets up a wire interupt for incomming data from master arduino
 *
 * returns: void
 */
void setup() {
  Wire.begin(13);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  servo.attach(5);
}

/*
 * Function: loop
 * --------------------
 * Does nothing and is not used, repeats a delay
 *
 * returns: void
 */
void loop() {
  delay(100);
}

/*
 * Function: loop
 * --------------------
 * Function that executes whenever data is received from master.
 * This function is registered as an event in setup()
 *
 * returns: void
 */
void receiveEvent(int howMany) {
  while (0 < Wire.available()) {
    servo.write((uint8_t) Wire.read());
  }
}
