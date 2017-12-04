#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Encoder.h>

double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
const double Kp =2.8, Ki = 0, Kd = 0;
const double Kpt =4.9, Kit = 0, Kdt = 0.07;
const double Kp1 = 1.1, Ki1 = 0.01, Kd1 = 0.1;
const double KpTurn = 6;
const double KpStright = 2.5;

PID PIDL(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID PIDR(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

static enum states {turnState, straightState}
state;

Servo L;
Servo R;
bool add;
bool firstCheck;
int turnError;
double startError;
double avgTurn;
int checkPrev = 0;
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
  Serial.print("R:");
  Serial.print(InputR);
  Serial.print(SetpointR);
  Serial.print("L:");
  Serial.print(InputL);
  Serial.println(SetpointL);
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

void turn(double deg){
  if(add){
    SetpointL -= degToMM(deg);
    SetpointR += degToMM(deg);
    PIDL.SetTunings(Kpt, Kit, Kdt);
    PIDR.SetTunings(Kpt, Kit, Kdt);
    avgTurn = (InputL + InputR) /2;
    add = false;
  }
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  // turnError = 20;
  turnError = (((InputL+InputR)/2)-avgTurn)*KpTurn; //pos = right faster
  runPID();
  if(firstCheck){
    if(isStable(InputR, SetpointR, 3)){
      firstCheck = false;
      PIDL.SetTunings(Kp, Ki, Kd);
      PIDR.SetTunings(Kp, Ki, Kd);
      //state = straightState;
      //add = true;
    }
  }
  else if(isStable(InputL, SetpointL, 3)){
    firstCheck = true;
  }
}

void straight(int dist){
  if(add){
    SetpointL += dist;
    SetpointR += dist;
    startError = InputL - InputR;
    add = false;
  }
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  turnError = ((InputL - InputR)-startError)*KpStright;
  runPIDS(dist);
  if(firstCheck){
    if(isStable(InputR, SetpointR, 3)){
      firstCheck = false;
      //State = turnLeft;
      //add = true;
    }
  }
  else if(isStable(InputL, SetpointL, 3)){
    firstCheck = true;
  }
}

void runState(){
  switch(state){
    case turnState:
      turn(3);
      break;

    case straightState:
      straight(700);
      break;
  }
}

void setup() {
  add = true;
  R.attach(10);  // attaches the servo on pin 9 to the servo object
  L.attach(11);
  Serial.begin(9600);
  Serial.println("Initalized");
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetMode(AUTOMATIC);
  SetpointL = 0;
  SetpointR = 0;
  PIDL.SetTunings(Kp, Ki, Kd);
  PIDR.SetTunings(Kp, Ki, Kd);
  PIDL.SetOutputLimits(-30, 30);
  PIDR.SetOutputLimits(-30, 30);
  state = turnState;
}

void loop() {
  runState();
}
