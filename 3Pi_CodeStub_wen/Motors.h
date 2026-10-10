/************************************
,        .       .           .      , 
|        |       |           |     '| 
|    ,-: |-. ,-. |-. ,-. ,-. |-     | 
|    | | | | `-. | | |-' |-' |      | 
`--' `-` `-' `-' ' ' `-' `-' `-'    ' 
*************************************/

// this #ifndef stops this file
// from being included mored than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions.  By using #define we can
// switch the number here, and everywhere the
// text appears (i.e. L_PWM) it will be
// replaced.
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

#define FWD LOW
#define REV HIGH

#define MAX_PWM 180.0f

class Motors_c {
  public:
    Motors_c() {}

    void initialise() {
      pinMode(L_PWM, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_DIR, OUTPUT);
      
      digitalWrite(L_DIR, FWD);
      digitalWrite(R_DIR, FWD);
      
      analogWrite(L_PWM, 0);
      analogWrite(R_PWM, 0);
    }

    void setPWM(float left_pwr, float right_pwr) {
      left_pwr = constrain(left_pwr, -MAX_PWM, MAX_PWM);
      right_pwr = constrain(right_pwr, -MAX_PWM, MAX_PWM);
      
      if (left_pwr < 0) { 
        digitalWrite(L_DIR, REV);  
        analogWrite(L_PWM, (int)(-left_pwr));
      } else {
        digitalWrite(L_DIR, FWD); 
        analogWrite(L_PWM, (int)left_pwr);
      }
      
      if (right_pwr < 0) { 
        digitalWrite(R_DIR, REV); 
        analogWrite(R_PWM, (int)(-right_pwr));
      } else {
        digitalWrite(R_DIR, FWD);
        analogWrite(R_PWM, (int)right_pwr);
      }
    }
};

#endif