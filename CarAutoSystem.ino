#include <Wire.h>
#include "Matrix.h"
#include "Ultrasonic.h"
#define MOTOR_LD 4
#define MOTOR_RD 2
#define MOTOR_LS 5
#define MOTOR_RS 9

#define LS_L 6
#define LS_C 7
#define LS_R 8

#define COLLISION_DISTANCE 10

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
}

int16_t d_speed = 100;
float l_fac = 1;
float r_fac = 1;

int offCounter = 0;

bool brakes_en = false;
bool obstacleAvoidance = false;
enum DIRECTION lastTurn = CENTER;

void loop(){
  //matrix.update();
  for (int i = 0; i < 180; i += 1) {
    Wire.beginTransmission(13);
    Wire.write(i);
    Wire.endTransmission();
    delay(10);
  }
  
  return;
  if (!obstacleAvoidance) {
    drive(100, 100);
    check_for_obstacle();
    //follow_path();
  } else
    avoidObstacle();
}

void avoidObstacle(){
 Serial.println("avvoiding");
}
void check_for_obstacle(){
  if (ultrasonic.ping() <= COLLISION_DISTANCE)
    obstacleAvoidance = true;
  if (!obstacleAvoidance)
    return;
  if (!brakes_en){
    brakes_en = true;
    drive(-100,-100);
    delay(100);
    drive(0,0);
    delay(200);
  }
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
      Serial.println("off");
      offCounter++;
      if (offCounter >= 100) {
        drive(0,0);
        Serial.println("offcnt");
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

  analogWrite( MOTOR_LS, L & 255 );
  analogWrite( MOTOR_RS, R & 255 );
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
