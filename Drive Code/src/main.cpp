#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Encoder.h>

double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
const double Kp =2.2, Ki = 0.01, Kd = 0.1;
const double Kpt =4.9, Kit = 0.01, Kdt = 0.08;
const double Kp1 = 1.1, Ki1 = 0.01, Kd1 = 0.1;
const double KpTurn = 3;
const double KpStright = 1.6;

PID PIDL(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID PIDR(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

static enum states {turnState, straightState, wait, blowCandle, goBack}
state;

int inL;
int inR;
int turnAngle;
int goDistance;
Servo L;
Servo R;
bool add;
bool goAfterTurn;
bool firstCheck;
int turnError;
double startError;
double avgTurn;
int checkPrev = 0;
String str;
Encoder encL(2,3);
Encoder encR(18,19);
int mapToServo(double num){
  return (int)num+90;
}

bool isStable(double input,double desired, int range){
  if(checkPrev > 15){
    checkPrev = 0;
    return true;
  }
  else if(abs(input-desired)<range){
    checkPrev ++;
    return false;
  }
  else{
    return false;
  }
}

void runPIDS(int dist){
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  PIDL.Compute();
  PIDR.Compute();
  if(!(dist - abs(SetpointR - InputR) > 50)){
    L.write(180-mapToServo(OutputL/2-turnError));
    R.write(180-mapToServo(OutputR/2+turnError));
  }
  else{
    L.write(180-mapToServo(OutputL/1.5-turnError));
    R.write(180-mapToServo(OutputR/1.5+turnError));
    Serial.print("   R:");
    Serial.println(InputR);
  }
}

double degToMM(double deg){
  return ((deg / 180) * 276.46);
}

void runPID(){
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  PIDL.Compute();
  PIDR.Compute();
  L.write(180-mapToServo((OutputL-turnError)/1.5));
  R.write(180-mapToServo((OutputR-turnError)/1.5));
  // Serial.print("R:");
  // Serial.print(InputR);
  // Serial.print(SetpointR);
  // Serial.print("L:");
  // Serial.print(InputL);
  // Serial.println(SetpointL);
}

// void left90(){
//   if(add){
//     SetpointL -= 138;
//     SetpointR += 138;
//     add = false;
//   }
//   runPID();
//   if(firstCheck){
//     if(isStable(InputR, SetpointR, 3)){
//       firstCheck = false;
//       state = turnRight;
//       add = true;
//     }
//   }
//   else if(isStable(InputL, SetpointL, 3)){
//     firstCheck = true;
//   }
// }
//
// void right90(){
//   if(add){
//     SetpointL += 500;
//     SetpointR += 500;
//     add = false;
//   }
//   InputL = encL.read()*0.1340413;
//   InputR = encR.read()*0.1340413;
//   InputD = InputL-InputR;
//   runPID();
//   if(firstCheck){
//     if(isStable(InputR, SetpointR, 3)){
//       firstCheck = false;
//       //state = turnLeft;
//       add = true;
//     }
//   }
//   else if(isStable(InputL, SetpointL, 3)){
//     firstCheck = true;
//   }
// }

void turn(double inp){//takes -180 to 360
//0-180 turns right
//181-360 turns left
//-180 to 0 turns left
  int deg;
  if(inp> 180){
    deg = 360-inp;
  }
  else{
    deg = -1*inp;
  }
  if(add){
    SetpointL -= degToMM(deg);
    SetpointR += degToMM(deg);
    PIDL.SetTunings(Kpt, Kit, Kdt);
    PIDR.SetTunings(Kpt, Kit, Kdt);
    avgTurn = (InputL + InputR) /2;
    add = false;
  }
  // turnError = 20;
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  turnError = (((InputL+InputR)/2)-avgTurn)*KpTurn; //pos = right faster
  runPID();
  if(firstCheck){
    if(isStable(InputR, SetpointR, 3)){
      firstCheck = false;
      PIDL.SetTunings(Kp, Ki, Kd);
      PIDR.SetTunings(Kp, Ki, Kd);
      if(goAfterTurn){
        state = straightState;
        goAfterTurn = false;
        encL.write(0);
        encR.write(0);
      }
      else{
        state = wait;
      }
      add = true;
    }
  }
  else if(isStable(InputL, SetpointL, 3)){
    firstCheck = true;
  }
}

void straight(int dist){
  Serial.println(dist);
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  if(add){
    inL = InputL;
    inR = InputR;
    SetpointL += dist;
    SetpointR += dist;
    startError = InputL - InputR;
    add = false;
  }
  turnError = (((InputL-inL) - (InputR-inR))-startError)*KpStright;
  runPIDS(dist);
  if(firstCheck){
    if(isStable(InputR, SetpointR, 4)){
      firstCheck = false;
      state = wait;
      add = true;
    }
  }
  else if(isStable(InputL, SetpointL, 4)){
    firstCheck = true;
  }
}

void runState(){
  switch(state){
    case turnState:
      turn(turnAngle);
      break;

    case straightState:
      straight(goDistance);
      break;

    case wait:
      R.write(90);
      L.write(90);
      break;

    case blowCandle:
      break;

    case goBack:
      break;
  }
}

void setup() {
  add = true;
  R.attach(10);  // attaches the servo on pin 9 to the servo object
  L.attach(11);
  Serial.begin(9600);
  Serial.println("Initalized");
  Serial3.begin(115200);
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetMode(AUTOMATIC);
  SetpointL = 0;
  SetpointR = 0;
  PIDL.SetTunings(Kp, Ki, Kd);
  PIDR.SetTunings(Kp, Ki, Kd);
  PIDL.SetOutputLimits(-34, 34);
  PIDR.SetOutputLimits(-34, 34);
  state = wait;
}

void loop() {

  runState();

  if(Serial3.available() > 0){
      char com = Serial3.read();
      if(com == '*'){
        str = Serial3.readStringUntil('\n');
        state = turnState;
        goAfterTurn = false;
        turnAngle = str.toInt();
      }
      else if(com == '#'){
        str = Serial3.readStringUntil('\n');
        turnAngle = str.substring(0,3).toInt()-200;
        goDistance = str.substring(3).toInt();
        state = turnState;
        goAfterTurn = true;
      }
  }
}
