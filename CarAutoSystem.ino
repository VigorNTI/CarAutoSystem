#include <Wire.h>
#include <EEPROM.h>
#include "Matrix.h"
#include "Ultrasonic.h"
#define MOTOR_LD 4
#define MOTOR_RD 2
#define MOTOR_LS 5
#define MOTOR_RS 9

#define LS_L 6
#define LS_C 7
#define LS_R 8

#define COLLISION_DISTANCE  15
#define SECTOR_THRES_START  10
#define SECTOR_THRES_END    25
#define SECTORS_NUM         5

Matrix matrix;
Ultrasonic ultrasonic(12, 13);

enum DIRECTION {
  LEFT,
  RIGHT,
  CENTER
};

void setup(){
  Serial.begin( 9600 );
  Wire.begin();
  pinMode( MOTOR_LD, OUTPUT );
  pinMode( MOTOR_RD, OUTPUT );
  pinMode( MOTOR_LS, OUTPUT );
  pinMode( MOTOR_RS, OUTPUT );

  pinMode( LS_L, INPUT );
  pinMode( LS_C, INPUT );
  pinMode( LS_R, INPUT );
  matrix.setup();
  matrix.set_mode(0);

  delay(1000);
  writeServo(90);
  delay(200);
}

int16_t d_speed = 100;
float l_fac = 1;
float r_fac = 1;

int offCounter = 0;

bool brakes_en = false;
bool obstacleAvoidance = false;
bool aligningLine = false;
enum DIRECTION lastTurn = CENTER;

void loop(){
  if (Serial.available()) { // failsafe, send something via serial com and this locks the car
    String buf = "";
    while (Serial.available()) {
      buf += (char)Serial.read();
    }
    if (strcmp(buf.c_str(), "stop") == 0)
      for (;;);
  }
  //matrix.update();
  if (!obstacleAvoidance) {
    writeServo(90);
    check_for_obstacle();
    if (!obstacleAvoidance)
      follow_path();
  } else if (!aligningLine)
    avoidObstacle();
  else {
    // back up if any of the lights are on the line
    if (ls_l() || ls_c() || ls_r()) {
      blindBrake();
      goto line_found_f;
    } else if (all_off()) { // if no lights are on the line, find the line
      drive(100, 100); // find forward
      for (int i = 0; i < 200; i++) {
        if (ls_l() || ls_c() || ls_r()) {
          blindBrake();
          delay(100);
          blindBrake(); // back up a little
          goto line_found_f;
        }
        delay(1);
      }
      blindBrake();
      // if nothing is found, we search backwards
      drive(-100, -100);
      for (int i = 0; i < 400; i++) {
        if (ls_l() || ls_c() || ls_r()) {
          blindBrakeReverse();
          delay(100);
          blindBrake(); // back up a little
          goto line_found_f;
        }
        delay(1);
      }
      blindBrakeReverse();
      return;
    }
 line_found_f:


    drive(200, 200);
    delay(200);
    blindBrake();
    delay(200);
    drive(255, -150);
    while (!((ls_c() && ls_r()) || (ls_l() && ls_c())));
    drive(0,0);
    aligningLine = false;
    obstacleAvoidance = false;
  }
}

void avoidObstacle(){
  detectWall();

  // Divide into sectors
  uint8_t* sectors = sectorsStatus();
  // Take action from sector status
  if (*(sectors + 2) < SECTOR_THRES_START)
    blindBrake();
  else if ((*(sectors + 2) > SECTOR_THRES_START && *(sectors + 2) < SECTOR_THRES_END) ||
           (*(sectors + 1) < SECTOR_THRES_START) ||
           (*(sectors + 0) < 7)) {
    // turn right
    drive(200, -200);
    delay(200);
    // brake
    drive(0,0);
    delay(200);
  } else if (*(sectors + 0) <= SECTOR_THRES_END && 
             *(sectors + 1) >  SECTOR_THRES_START &&
             *(sectors + 2) >  SECTOR_THRES_START) 
  {
    // drive forward
    drive(200, 200);
    for (int i = 0; i < 200; i++) {
      if (ls_l() || ls_r()) {
        aligningLine = true;
        blindBrake();
        blindBrake();
        return;
      }
      delay(1);
    }
    // brake
    blindBrake();
  } else if (*(sectors + 0) >  SECTOR_THRES_END && 
             *(sectors + 2) >  SECTOR_THRES_END &&
             *(sectors + 1) >  SECTOR_THRES_END) 
  {
    // turn half left
    drive(0, 200);
    delay(200);
    // brake
    drive(0, -100);
    delay(100);
    drive(0,0);
    delay(200);
  }
  
  Serial.println("Sectors: ");
  for (int i = 0; i < 5; i++) {
    Serial.println(*(sectors + i));
  }
  Serial.println("");
  // cleanup
  delete[] sectors;
  //delay(-1);
}
uint8_t* sectorsStatus() {
  uint8_t* output = new uint8_t[SECTORS_NUM];
  uint8_t range = 180/SECTORS_NUM;

  for (int i = 0; i < SECTORS_NUM; i++) {
    uint8_t minValue = 100;
    for (int j = range*i; j < range*(i+1); j++) {
      int a = EEPROM.read(j);
      for (int x = 0; x < a; x++)
      if (a < minValue)
        minValue = a;
    }
    *(output + i) = minValue;
  }
  // well, return the status
  return output;
}
void detectWall() {
  writeServo(180);
  delay(500);
  int addr = 0;
  for (int i = 180; i >= 0; i--) {
    writeServo(i);
    delay(10);
    EEPROM.update(addr, (byte)ultrasonic.ping());
    addr += 1;
  }
}
void check_for_obstacle(){
  if (ultrasonic.ping() <= COLLISION_DISTANCE)
    obstacleAvoidance = true;
  if (!obstacleAvoidance)
    return;
  brake();
}
void follow_path() {
  // firstly check for failsafe such as outside the map and at stop point
  if( !all_on() && !all_off() ) {
    offCounter = 0;
    // for tuning left and right
    if (ls_l())
      l_fac = (!ls_r() ?  0   : 0);
    else
      l_fac = ( ls_r() ?  3   : 0.8);

    if (ls_r())
      r_fac = (!ls_l() ?  0   : 0);
    else
      r_fac = ( ls_l() ?  3   : 0.8);
    
    // for desiding if we turn or if we are still going forward
    if(ls_l() && !ls_r())
      lastTurn = LEFT;
    else if (!ls_l() && ls_r())
      lastTurn = RIGHT;
    else
      lastTurn = CENTER;

    // make use of calculated left and right factors and drive
    drive(d_speed*l_fac, d_speed*r_fac);
    brakes_en = false;
  }else {
    if (all_off()){
      offCounter++;
      if (offCounter >= 100) {
        drive(0,0);
      } else {
        if (!brakes_en){
          brakes_en = true;
          drive(-100,-100);
          delay(100);
          drive(0,0);
          delay(200);
        }
        if (lastTurn == LEFT)
          drive(0, d_speed*3);
        if (lastTurn == RIGHT)
          drive(d_speed*3, 0);
      }
    } else if (all_on()) {
      if (!brakes_en){
        brakes_en = true;
        drive(-100,-100);
        delay(100);
        drive(0,0);
        delay(200);
      }
    }
  }
}
void blindBrake() {
  drive(-100,-100);
  delay(100);
  drive(0,0);
  delay(200);
}
void blindBrakeReverse() {
  drive(100,100);
  delay(100);
  drive(0,0);
  delay(200);
}
void brake() {
  if (!brakes_en){
    brakes_en = true;
    drive(-100,-100);
    delay(100);
    drive(0,0);
    delay(200);
  }
}
void writeServo(int degree) {
  Wire.beginTransmission(13);
  Wire.write(degree);
  Wire.endTransmission();
}

bool ls_l(){
  return digitalRead( LS_L );
}
bool ls_c(){
  return digitalRead( LS_C );
}
bool ls_r(){
  return digitalRead( LS_R );
}

bool all_on() {
  return ls_l() && ls_c() && ls_r();
}
bool all_off() {
  return !ls_l() && !ls_c() && !ls_r();
}

void drive(int L, int R) {
  /*analogWrite(MOTOR_LS, 0);
  analogWrite(MOTOR_RS, 0);
  Serial.print(L);
  Serial.print(" ");
  Serial.println(R);
  return;*/
  if ( L < 0 )
    gear_reverse(LEFT);
  else
    gear_drive(LEFT);
    
  if ( R < 0 )
    gear_reverse(RIGHT);
  else
    gear_drive(RIGHT);
  
  if (L < -255 || L > 255)
    L = 255;
  if (R < -255 || R > 255)
    R = 255;

  analogWrite( MOTOR_LS, abs(L) );
  analogWrite( MOTOR_RS, abs(R) );
}

void gear_mix(enum DIRECTION d){
  digitalWrite( MOTOR_LD, d );Serial.print(d);
  digitalWrite( MOTOR_RD, !d );Serial.println(!d);
}

void gear_drive(enum DIRECTION d){
  if (d == LEFT)
    digitalWrite( MOTOR_LD, LOW );
  else
    digitalWrite( MOTOR_RD, LOW );
}
void gear_reverse(enum DIRECTION d){
  if (d == LEFT)
    digitalWrite( MOTOR_LD, HIGH );
  else
    digitalWrite( MOTOR_RD, HIGH );
}
