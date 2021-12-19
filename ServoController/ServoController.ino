#include <Wire.h>
#include <Servo.h>

Servo servo;

void setup() {
  Wire.begin(13);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  servo.attach(5);
}

void loop() {
  delay(100);
}

// function that executes whenever data is received from master
// this function is registered as an event in setup()
void receiveEvent(int howMany) {
  while (0 < Wire.available())
    servo.write((char) Wire.read());
}
