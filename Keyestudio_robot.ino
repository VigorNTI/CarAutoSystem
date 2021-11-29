#include "Matrix.h"
#define MOTOR_LD 4
#define MOTOR_RD 2
#define MOTOR_LS 5
#define MOTOR_RS 9

#define LS_L 6
#define LS_C 7
#define LS_R 8

Matrix matrix;

enum DIRECTION {
    LEFT,
    RIGHT,
    CENTER
  };

void setup(){
  Serial.begin( 9600 );
  pinMode( MOTOR_LD, OUTPUT );
  pinMode( MOTOR_RD, OUTPUT );

  pinMode( LS_L, INPUT );
  pinMode( LS_C, INPUT );
  pinMode( LS_R, INPUT );
  matrix.setup();
  matrix.set_mode(2);
}

int16_t d_speed = 100;
float l_fac = 1;
float r_fac = 1;

bool brakes_en = false;

enum DIRECTION lastTurn = CENTER;

void loop(){
  matrix.update();
  return;

  if( !all_on() && !all_off() ) {
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
      

  Serial.print(l_fac);
  Serial.print(" ");
  Serial.println(r_fac);
      
    drive(d_speed*l_fac, d_speed*r_fac);
    
  }else 
    drive(0,0);

  while (all_off()){
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
  while(all_on()) {
    if (!brakes_en){
      brakes_en = true;
      drive(-100,-100);
      delay(100);
      drive(0,0);
      delay(200);
    }
  }
  brakes_en = false;
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
