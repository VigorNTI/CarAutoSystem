/**
 * Implementation of the ultrasonic sensor.
 * 
 * Class is constructed into an object with parameters for trigger pin and echo pin.
 * Calling ping returns a distance.
 *
 */

class Ultrasonic {
public:
  // defines variables
  long duration; // variable for the duration of sound wave travel
  int distance; // variable for the distance measurement

/*
 * constructor: Ultrasonic
 * --------------------
 * Initiates gpios for the ultrasonic sensor
 *
 * trigPin_in: trigger gpio pin
 * echoPpin_in: echo gpio pin
 *
 * returns: ultrasonic class object
 */
  Ultrasonic(int trigPin_in, int echoPin_in){
    this->trigPin = trigPin_in;
    this->echoPin = echoPin_in;

    pinMode(trigPin, OUTPUT); // Sets the trigPin as an OUTPUT
    pinMode(echoPin, INPUT); // Sets the echoPin as an INPUT
    Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
    Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
    Serial.println("with Arduino UNO R3");
  }

/*
 * Function: ping
 * --------------------
 * Does the ping logic and drives the ultrasonic sensor
 *
 *
 * returns: an unsigned int of the distance between sensor and object in front of it
 */
  unsigned int ping() {
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Waits for the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 since we now measure back and forth
    // Displays the distance on the Serial Monitor
    if (distance > 100)
      distance = 100;
    return distance;
  }
private:
  int trigPin;
  int echoPin;
  
};
