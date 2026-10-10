#include "Motors.h"
#include "Encoders.h"
#include "Kinematics.h"
#include "Magnetometer.h"
#include "oled.h"
#include <math.h>

Motors_c motors;
Kinematics_c pose;
Magnetometer_c magnetometer;

//OLED countdown
OLED_c display(1, 30, 0, 17, 13);

//buzzer pin
#define BUZZER_PIN 6

//run clock
#define RUN_TIME 240000UL //total run time，4 minutes stop
#define POSE_UPDATE 20 //update odometry every 20ms
unsigned long runStartTime; //time when run starts
unsigned long lastUpdateTime; //last time robot updated odometry
bool runFinished = false; //true after 4 minutes, robot stops

//FSM states
#define SET_WAYPOINT 0
#define TRAVEL 1
#define ALIGN 2
#define PUSH 3
#define RETURN 4
#define PAUSE 5
#define STOP 6
#define PAUSE_TIME 4000 //pause duration between rounds
int state;
unsigned long pauseStartTime;

//waypoints(1-6 puck)
#define TOTAL_WAYPOINTS 7
float pointX[TOTAL_WAYPOINTS] = {0, 142, 270, 396, 419, 275, 145}; //x
float pointY[TOTAL_WAYPOINTS] = {0, 258, 159, 275, 70, -20, 100}; //y
int currentWaypoint; //which waypoint robot go to now

//magnetometer
#define DETECT_THRESHOLD 6.69 //if m is bigger than this, puck is found
#define DETECT_DIST 55.0 //distance from robot centre to puck when detected
#define CALIBRATE_PWM 29 //slow spin for calibration
#define CALIBRATE_TIME 4000 //spin-calibrate for 4 seconds
float magOffset[MAX_AXIS]; 
float magScaling[MAX_AXIS];

void returnToStartHeading();

void runMagCalibration(){
  float magMin[MAX_AXIS]; //min value per axis
  float magMax[MAX_AXIS]; //max value per axis
  //set min very big and max very small first
  //so the first real reading can replace them
  for (int a = 0; a < MAX_AXIS; a++){
    magMin[a] = 9999.9; 
    magMax[a] = -9999.9;
    }
  motors.setPWM(-CALIBRATE_PWM, CALIBRATE_PWM); //spin on the spot
  unsigned long spinStartTime = millis(); //record spin start time
  unsigned long lastPoseTime = millis(); //record last pose-update time
  while (millis() - spinStartTime < CALIBRATE_TIME){ //keep spinning for 4 seconds
    if (millis() - lastPoseTime >= POSE_UPDATE){  //every 20ms
      lastPoseTime = millis();
      pose.update();
    }

    display.timeRemaining(); //refresh the countdown 

    magnetometer.getReadings(); //read the three raw axis values
    for(int a = 0; a < MAX_AXIS; a++){
      if (magnetometer.readings[a] > magMax[a]) magMax[a] = magnetometer.readings[a]; //update max
      if (magnetometer.readings[a] < magMin[a]) magMin[a] = magnetometer.readings[a]; //update min
    }
  }
  motors.setPWM(0, 0);

  //after stop, wait a short time so theta becomes stable
  unsigned long settleStartTime = millis(); //record settle start time
  while (millis() - settleStartTime < 400){ //settle about 0.4 seconds
    pose.update();
    display.timeRemaining();
    delay(POSE_UPDATE); //wait 20ms each pass
  }

  for (int a = 0; a < MAX_AXIS; a++){
    //offset as middle of min and max, scaling makes the value about -1 to 1
    float range = magMax[a] - magMin[a];
    magOffset[a] = magMin[a] + (range / 2.0);
    if (range > 0) magScaling[a] = 1.0 / (range / 2.0);
    else magScaling[a] = 0;
  }
}
//read one calibrated field strength m
float readMagStrength(){
  magnetometer.calcCalibratedMag(magOffset, magScaling);
  return magnetometer.getCalibratedMagnitude();
}

//turn
#define TURN_THRESHOLD 0.05
#define TURN_GAIN 118.0
#define TURN_MAX_PWM 69.5
#define TURN_MIN_PWM 18.0

//slower speeds for the one-off return-to-heading after calibration
#define RETURN_MAX_PWM 35.0
#define RETURN_MIN_PWM 16.0
#define STABLE_COUNT 5
float targetAngle;

//smallest signed angle from current to target, in [-pi, +pi]
float shortestAngleDifference(float target, float current){
  float diff = target - current; //raw difference first
  return atan2(sin(diff), cos(diff)); //wrap into [-pi,+pi] and return
}

//set the target heading angle
void setTurn(float angle){
  targetAngle = angle; //store the target angle
  }

//turn toward the target
bool checkTurn(){
  float diff = shortestAngleDifference(targetAngle, pose.theta); //how far to turn
  if (fabs(diff) < TURN_THRESHOLD){ //close enough, stop
    motors.setPWM(0, 0); 
    return true;
    }
  float turnPwm = diff * TURN_GAIN;
  if (turnPwm > TURN_MAX_PWM) turnPwm = TURN_MAX_PWM;
  if (turnPwm < -TURN_MAX_PWM) turnPwm = -TURN_MAX_PWM;
  if (turnPwm > 0 && turnPwm < TURN_MIN_PWM) turnPwm = TURN_MIN_PWM;
  if (turnPwm < 0 && turnPwm > -TURN_MIN_PWM) turnPwm = -TURN_MIN_PWM;
  motors.setPWM(-turnPwm, turnPwm);
  return false;
}

//after calibration spin, turn back to theta = 0
void returnToStartHeading(){
  unsigned long lastPoseTime = millis();
  int settle = 0;

  while (true){
    if (millis() - lastPoseTime >= POSE_UPDATE){
      lastPoseTime = millis();
      pose.update();
      display.timeRemaining();

      float diff = shortestAngleDifference(0.0, pose.theta);

      if (fabs(diff) < TURN_THRESHOLD){
        motors.setPWM(0, 0);
        settle++;
        if (settle >= STABLE_COUNT) break;
      } else{
        settle = 0;
        float turnPwm = diff * TURN_GAIN;
        if (turnPwm > RETURN_MAX_PWM) turnPwm = RETURN_MAX_PWM;
        if (turnPwm < -RETURN_MAX_PWM) turnPwm = -RETURN_MAX_PWM;
        if (turnPwm > 0 && turnPwm < RETURN_MIN_PWM) turnPwm = RETURN_MIN_PWM;
        if (turnPwm < 0 && turnPwm > -RETURN_MIN_PWM) turnPwm = -RETURN_MIN_PWM;
        motors.setPWM(-turnPwm, turnPwm);
      }
    }
  }
  motors.setPWM(0, 0);
}

//travel
#define TRAVEL_TOLERANCE 21.0
#define MOVE_PWM 18.0
#define MOVE_GAIN 40.0
#define MOVE_MAX_STEER 35.0
#define RETURN_TO_TURN_ANGLE 0.5
#define SLOW_RADIUS 77.0
#define SLOW_MIN_PWM 22.0
#define TRAVEL_TURN 0
#define TRAVEL_DRIVE 1
#define TRAVEL_DONE 2
int travelState;
float arriveTolerance;
float targetX, targetY;

//set a travel target
void setTravel(float tx, float ty, float tol){
  targetX = tx; 
  targetY = ty;
  arriveTolerance = tol;
  travelState = TRAVEL_TURN;
}

float targetDistance(){
  float dx = targetX - pose.x, dy = targetY - pose.y;
  return sqrt(dx * dx + dy * dy);
}

float headingToTarget(){
  float dx = targetX - pose.x, dy = targetY - pose.y;
  return atan2(dy, dx);
}

//go to target point
bool travelToXY(){
  if (travelState == TRAVEL_TURN){
    setTurn(headingToTarget());
    if (checkTurn() == true) travelState = TRAVEL_DRIVE;
    return false;
  }
  else if (travelState == TRAVEL_DRIVE){
    float dist = targetDistance(); //distance left
    if (dist < arriveTolerance){ //close enough, stop
      motors.setPWM(0, 0);
      travelState = TRAVEL_DONE;
      return true;
    }

    float headingErr = shortestAngleDifference(headingToTarget(), pose.theta);
    if (fabs(headingErr) > RETURN_TO_TURN_ANGLE){ //off course too much, turn again
      travelState = TRAVEL_TURN; 
      return false;
      }

    float forwardPwm;
    //if is far: full speed; near: slow down step by step
    if (dist >= SLOW_RADIUS) forwardPwm = MOVE_PWM;
    else{
      float frac = dist / SLOW_RADIUS; 
      forwardPwm = SLOW_MIN_PWM + (MOVE_PWM - SLOW_MIN_PWM ) * frac;
      }

    float turn = headingErr * MOVE_GAIN;
    if (turn > MOVE_MAX_STEER) turn = MOVE_MAX_STEER;
    if (turn < -MOVE_MAX_STEER) turn = -MOVE_MAX_STEER;
    //add correction to two wheels, so it drives and steers together
    motors.setPWM(forwardPwm - turn, forwardPwm + turn);
    return false;
  }
  motors.setPWM(0, 0);
  return true;
}

#define SAFE_RADIUS 170.0 //go-around radius around the puck

float puckX, puckY; //puck centre
float midX, midY; //M: go-around point
float behindX, behindY; //E: behind the puck on the push line

//robot's current distance from the Start(0,0)
float distanceFromStart(){
  return sqrt(pose.x * pose.x + pose.y * pose.y);
}

//M: a side point, so robot can go around the puck without touching it
//E: behind the puck, on the line from start to puck
void computeTwoPoints(float robot_x, float robot_y){
  float puckDist = sqrt(puckX * puckX + puckY * puckY); //distance from start to puck
  if (puckDist < 1.0) puckDist = 1.0; //avoid divide by zero
  float pushDirX = puckX / puckDist, pushDirY = puckY / puckDist; //push line direction
  //E point
  behindX = puckX + SAFE_RADIUS * pushDirX; 
  behindY = puckY + SAFE_RADIUS * pushDirY;
  //cross > 0 means robot is on the left of the push line
  float cross = pushDirX * (robot_y - puckY) - pushDirY * (robot_x - puckX);
  float px, py;
  if (cross >= 0) {px = -pushDirY;  py = pushDirX;} //left side
  else {px = pushDirY; py = -pushDirX;} //right side
  //M point
  midX = puckX + SAFE_RADIUS * px;
  midY = puckY + SAFE_RADIUS * py;
}

//backoff
float backoffStartX, backoffStartY;
#define BACKOFF_DIST 70.0
#define BACKOFF_PWM 22.0
#define E_TOLERANCE 15.0

//push toward start
#define PUSH_PWM 20.0
#define PUSH_HEAD_GAIN 15.0
#define PUSH_MAX_STEER 11.0
#define START_ZONE_RADIUS 80.0

//FSM(sub)
#define BACKOFF 0
#define GO_M 1
#define GO_E 2
#define FACE_START 3
int alignStep;

//setup
void setup() {
  motors.initialise();
  pinMode(BUZZER_PIN, OUTPUT);
  display.setMaxMinutes(4);
  display.startStopwatch();
  runStartTime = millis();
  lastUpdateTime = millis();

  if (!magnetometer.initialise()){
    while (1); //magnetometer not working, stop here
  }

  setupEncoder0();
  setupEncoder1();
  pose.initialise(0, 0, 0);
  runMagCalibration();
  returnToStartHeading();
  pose.initialise(0, 0, 0);
  currentWaypoint = 1; //start from waypoint 1
  state = SET_WAYPOINT;
}

//loop
void loop(){
  unsigned long now = millis();
  //update odometry every 20 ms, only in one place
  if (now - lastUpdateTime >= POSE_UPDATE){
    lastUpdateTime = now;
    pose.update();
  }

  display.timeRemaining(); 

  //stop after 4 minutes
  if (!runFinished && (now - runStartTime >= RUN_TIME)){
    runFinished = true;
    state = STOP;
  }
  if (state == STOP){
    motors.setPWM(0, 0);
    return;
  }

  //set waypoint
  if (state == SET_WAYPOINT){
    if (currentWaypoint >= TOTAL_WAYPOINTS){
      setTravel(0, 0, TRAVEL_TOLERANCE);
      state = RETURN;
    } else{
      setTravel(pointX[currentWaypoint], pointY[currentWaypoint], TRAVEL_TOLERANCE);
      state = TRAVEL;
    }
    return;
  }

  //travel
  if (state == TRAVEL){
    float m = readMagStrength();
    if (m > DETECT_THRESHOLD){
      motors.setPWM(0, 0);
      tone(BUZZER_PIN, 1000, 200); //beep, puck found
      puckX = pose.x + DETECT_DIST * cos(pose.theta); //guess puck position in front
      puckY = pose.y + DETECT_DIST * sin(pose.theta);
      computeTwoPoints(pose.x, pose.y); //work out M and E
      backoffStartX = pose.x; //remember where we start to reverse
      backoffStartY = pose.y;
      alignStep = BACKOFF;
      state = ALIGN;
      return;
    }
    if (travelToXY() == true){
      currentWaypoint = currentWaypoint + 1;
      state = SET_WAYPOINT;
    }
    return;
  }

  //align
  if (state == ALIGN){
    if (alignStep == BACKOFF){
      float dx = pose.x - backoffStartX, dy = pose.y - backoffStartY;
      if (sqrt( dx * dx + dy * dy) >= BACKOFF_DIST){
        motors.setPWM(0, 0);
        setTravel(midX, midY, TRAVEL_TOLERANCE);
        alignStep = GO_M;
      } else{
        motors.setPWM(-BACKOFF_PWM, -BACKOFF_PWM);
      }
      return;
    }

    if (alignStep == GO_M){
      if (travelToXY() == true){
        setTravel(behindX, behindY, E_TOLERANCE);
        alignStep = GO_E;
      }
      return;
    }

    if (alignStep == GO_E){
      if (travelToXY() == true){
        setTurn(atan2(0.0 - pose.y, 0.0 - pose.x));
        alignStep = FACE_START;
      }
      return;
    }

    if (alignStep == FACE_START){
      if (checkTurn() == true){
        tone(BUZZER_PIN, 1200, 150);
        state = PUSH;
      }
      return;
    }
    return;
  }

  //push
  if (state == PUSH){
    if (distanceFromStart() < START_ZONE_RADIUS){
      motors.setPWM(0, 0);
      tone(BUZZER_PIN, 1500, 500);
      pauseStartTime = now;
      state = PAUSE;
      return;
    }
    float headingToStart = atan2(0.0 - pose.y, 0.0 - pose.x);
    float headingErr = shortestAngleDifference(headingToStart, pose.theta);
    float turn = headingErr * PUSH_HEAD_GAIN;
    if (turn > PUSH_MAX_STEER) turn = PUSH_MAX_STEER;
    if (turn < -PUSH_MAX_STEER) turn = -PUSH_MAX_STEER;
    motors.setPWM(PUSH_PWM - turn, PUSH_PWM + turn);
    return;
  }

  //return
  if (state == RETURN){
    if (travelToXY() == true){
      pauseStartTime = now;
      state = PAUSE;
    }
    return;
  }

  //pause
  if (state == PAUSE){
    motors.setPWM(0, 0);
    if (now - pauseStartTime >= PAUSE_TIME){
      currentWaypoint = 1;
      state = SET_WAYPOINT;
    }
    return;
  }

  motors.setPWM(0, 0);
}
