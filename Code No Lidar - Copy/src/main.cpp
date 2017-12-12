#include <Arduino.h>
#include <PID_v1.h>
#include <Servo.h>
#include <Encoder.h>
#include <Wire.h>
#include <EasyNeoPixels.h>

int IRsensorAddress = 0xB0;
//int IRsensorAddress = 0x58;
int slaveAddress;
int ledPin = 13;
int z;
boolean ledState = false;
byte data_buf[16];

int Ix[4];
int Iy[4];
int s;


double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
const double Kp =2.2, Ki = 0.01, Kd = 0.1;
//const double Kpt =6, Kit = 0.02, Kdt = 0.3;
const double Kpt = 4 , Kit = 0.1, Kdt = 0.3;
const double Kp1 = 1.1, Ki1 = 0.01, Kd1 = 0.1;
const double KpTurn = 3;
const double KpStright = 0.1;
const double Kpheading = 0.5;
const int wallDist = 200;
const int timeout = 5000;
const int blowTime = 4000;
const int blowForce = 1550; //1700
const int linePin = 6;
bool add = true;
bool flag = true;
bool followingLeft = false; //derp

PID PIDL(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID PIDR(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

Encoder encL(3,2);
Encoder encR(18,19);

Servo L;
Servo R;
Servo Blower;
String str;         // a String to hold incoming data
int inL;
int inR;
int curX = 0;
int curY = 0;
double startError;
int checkPrev = 0;
int avgTurn;
int headingAdj;
bool addToPos = true;
bool inita;
bool leftTurn;
bool start = false;
bool firstCheck;
bool straightAfterTurn = false;
bool waitToWallFollow = false;
bool lookAgain = false;
bool cliffAfterTurn = false;
int heading = 0; //0 , 90, 180, 360
char input;
int distance;
int front = 1000; //stores lidar data
int left = 1000;
int right = 1000;
int candleDist = 0;
int yheight = 0;
unsigned long prevMillis = 0;
double turnError;
const int waitMillis = 700;
static enum states {lookCandle, idle, wallFollowState, turnState, waitState, continuePast, blowCandle, turnToCandle, turnBack, cliffState, straightUntilWall}
state;

void Write_2bytes(byte d1, byte d2)
{
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1); Wire.write(d2);
  Wire.endTransmission();
}


int mapToServo(double num){
  int ret = (int)num+90;
  if(ret < 20){
    return 20;
  }
  else if(ret > 160){
    return 160;
  }
  return ret;
}

void setup(){
  setupEasyNeoPixels(12, 1);
  pinMode(13, OUTPUT);
  slaveAddress = IRsensorAddress >> 1;   // This results in 0x21 as the address to pass to TWI
  pinMode(ledPin, OUTPUT);      // Set the LED pin as output
  Wire.begin();
  // IR sensor initialize
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10);

  Serial.begin(115200);
  Serial2.begin(115200);
  Serial3.begin(115200);
  Serial.println("Initalized");
  state = idle;
  leftTurn = true;
  R.attach(10);  // attaches the servo on pin 9 to the servo object
  L.attach(11);
  Blower.attach(8);
  PIDL.SetMode(AUTOMATIC);
  PIDR.SetMode(AUTOMATIC);
  SetpointL = 0;
  SetpointR = 0;
  PIDL.SetTunings(Kp, Ki, Kd);
  PIDR.SetTunings(Kp, Ki, Kd);
  PIDL.SetOutputLimits(-60, 60);
  PIDR.SetOutputLimits(-60, 60);
  Blower.writeMicroseconds(700);
  digitalWrite(13, HIGH);
  L.write(90);
  R.write(90);
  // while(!(Serial2.available() >0)){
  //   Serial2.read();
  //   start = true;
  //   state = wallFollowState; //first state after go
  // }
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
  // Serial.print("R: in:");
  // Serial.print(InputR);
  // Serial.print(" set:");
  // Serial.print(SetpointR);

}

bool isStable(double input,double desired, int range){
  if(checkPrev > 100){
    checkPrev = 0;
    return true;
  }
  else if(abs(input-desired)<range){
    checkPrev ++;
    return false;
  }
  else if(millis()>prevMillis+timeout){
    checkPrev = 0;
    return true;
  }
  else{
    return false;
  }
}

void wallFollow(){
  headingAdj = (right - wallDist)*Kpheading; // returns positive when turning right
  if (headingAdj>10){
    headingAdj=10;
  }
  else if(headingAdj<-10){
    headingAdj=-10;
  }
  //Serial.println(headingAdj);
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  turnError = -4;
  turnError += (int)((InputL - InputR)*KpStright);
  if (turnError>30){
    turnError=30;
  }
  else if(turnError<-30){
    turnError=-30;
  }
  //L.write(60+turnError-headingAdj);
  L.write(60+turnError-headingAdj);
  //L.write(20);
  //  R.write(20);
  //R.write(60-turnError+headingAdj);
  R.write(60-turnError);
}

void runPIDS(int dist){
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  PIDL.Compute();
  PIDR.Compute();
  L.write(180-mapToServo(OutputL/2-turnError));
  R.write(180-mapToServo(OutputR/2+turnError));
}

void runPIDB(int dist){
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  PIDL.Compute();
  PIDR.Compute();
  L.write(180-mapToServo(OutputL/2-turnError));
  R.write(180-mapToServo(OutputR/2+turnError));
}

void straight(int dist){
  //Serial.println(dist);
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  if(add){
    encL.write(0);
    encR.write(0);
    InputL = encL.read()*0.1340413;
    InputR = encR.read()*0.1340413;
    inL = InputL;
    inR = InputR;
    SetpointL = InputL;
    SetpointR = InputR;
    SetpointL += dist;
    SetpointR += dist;
    startError = InputL - InputR;
    add = false;
  }
  turnError = -4;
  turnError += (((InputL-inL) - (InputR-inR))-startError)*KpStright;
  runPIDS(dist);
  prevMillis = millis();

}

void backwards(int dist){
  //Serial.println(dist);
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  L.write(130);
  R.write(120);
  //turnError -= ((InputL) - (InputR))*KpStright;

}


void straightAtCliff(){
  InputL = encL.read()*0.1340413;
  InputR = encR.read()*0.1340413;
  if(add){
    encL.write(0);
    encR.write(0);
    InputL = encL.read()*0.1340413;
    InputR = encR.read()*0.1340413;
    inL = InputL;
    inR = InputR;
    startError = InputL - InputR;
    add = false;
    prevMillis = millis();
  }
  turnError = -4;
  turnError += (((InputL-inL) - (InputR-inR))-startError)*KpStright;
  L.write(180-mapToServo(30-turnError));
  R.write(180-mapToServo(30+turnError));
}

void addPos(){
  if(addToPos){
    if(heading == 0){
      curX += encR.read()/17.8643;
    }
    else if(heading == 90){
      curY += encR.read()/17.8643;
    }
    else if(heading == 180){
      curX -= encR.read()/17.8643;
    }
    else{
      curY -= encR.read()/17.8643;
    }
  }
}

void addCandleDist(){
    if(heading == 0){
      curX += candleDist;
    }
    else if(heading == 90){
      curY += candleDist;
    }
    else if(heading == 180){
      curX -= candleDist;
    }
    else{
      curY -= candleDist;
    }
}

void turn(){

  if(add){

    encL.write(0);
    encR.write(0);
    InputL = encL.read()/17.8643;
    InputR = encR.read()/17.8643;
    if(leftTurn){
      SetpointL = -degToMM(90) + encL.read()/17.8643;
      SetpointR = degToMM(90) + encR.read()/17.8643;
    }
    else{
      SetpointL = -degToMM(-90) + encL.read()/17.8643;
      SetpointR = degToMM(-90) + encR.read()/17.8643;
    }
    PIDL.SetTunings(Kpt, Kit, Kdt);
    PIDR.SetTunings(Kpt, Kit, Kdt);
    avgTurn = (InputL + InputR) /2;
    add = false;
    prevMillis = millis();
  }
  // turnError = 20;
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  encR.write(0);
  turnError = (((InputL+InputR)/2)-avgTurn)*KpTurn; //pos = right faster

  // Serial.print(((InputL+InputR)/2));
  // Serial.print(" avg:");
  // Serial.println(avgTurn);
  runPID();
}

void turn270(){
  InputL = encL.read()/17.8643;
  if(add){
    //Serial.println(encR.read());
    encL.write(0);
    encR.write(0);
    //Serial.println(encR.read());
    InputR = encR.read()/17.8643;
    SetpointL = -degToMM(-266) + encL.read()/17.8643;
    SetpointR = degToMM(-266) + encR.read()/17.8643;
    PIDL.SetTunings(Kpt, Kit, Kdt);
    PIDR.SetTunings(Kpt, Kit, Kdt);
    avgTurn = (InputL + InputR) /2;
    add = false;
    prevMillis = millis();
  }
  // turnError = 20;
  InputL = encL.read()/17.8643;
  InputR = encR.read()/17.8643;
  turnError = (((InputL+InputR)/2)-avgTurn)*KpTurn; //pos = right faster

  // Serial.print(((InputL+InputR)/2));
  // Serial.print(" avg:");
  // Serial.println(avgTurn);
  runPID();
}

void runState(){
  switch(state){
    case lookCandle:
    writeEasyNeoPixel(0, 255, 69,0);
    if(Ix[0] > 480 && Ix[0] != 1023){
      leftTurn = false;
      state = turnToCandle;
      candleDist = left;
      heading = (heading+90)%360;
      addCandleDist();
      yheight = Iy[0];
      addPos();
      addToPos = false;
    }

    Serial.println(Ix[0]);
    // if(Ix[0] == 1023){
    //   state = wallFollowState;
    // }
    wallFollow();
    break;
    case idle:
    writeEasyNeoPixel(0, 0, 0,255);
    L.write(90);
    R.write(90);
    Blower.writeMicroseconds(700);
    break;
    case wallFollowState:
    wallFollow();
    writeEasyNeoPixel(0, 0, 255,0);//green
    if(front < 220){
      addPos();
      state = waitState;
      prevMillis = millis();
    }
    if(right > 500){
      addPos();
      state = continuePast;
    }
    break;
    case turnState:
    writeEasyNeoPixel(0, 0, 255,255); //turqoise
    turn();
    if(firstCheck){
      if(isStable(InputR, SetpointR, 2)){
        firstCheck = false;
        PIDL.SetTunings(Kp, Ki, Kd);
        PIDR.SetTunings(Kp, Ki, Kd);
        if(leftTurn){
          heading = (heading+90)%360;
        }
        else{
          if(heading == 0){
            heading = 270;
          }
          else{
            heading -= 90;
          }
        }
        if(cliffAfterTurn){
          state = straightUntilWall;
          cliffAfterTurn=false;
        }
        else if(straightAfterTurn)
          state = continuePast;
        else
          state = wallFollowState;
        encL.write(0);
        encR.write(0);
        add = true;
      }
    }
    else if(isStable(InputL, SetpointL, 2)){
      firstCheck = true;
    }
    break;
    case waitState:
    writeEasyNeoPixel(0, 255, 255,0);
    L.write(90);
    R.write(90);
    if(millis()>prevMillis+waitMillis){
      if(waitToWallFollow){
        state = wallFollowState;
        waitToWallFollow = false;
      }
      else{
        state = turnState;
      }
      leftTurn = true;
    }
    break;
    case continuePast:
    writeEasyNeoPixel(0, 0, 255,100);
    if(straightAfterTurn){
      straight(300);
    }
    else{
      straight(160);
    }

    if(firstCheck){
      if(isStable(InputR, SetpointR, 14)){
        firstCheck = false;
        addPos();
        encL.write(0);
        encR.write(0);
        if(straightAfterTurn){
          state = waitState;
          waitToWallFollow = true;
          straightAfterTurn = false;
        }
        else{
          state = turnState;
          straightAfterTurn = true;
        }
        leftTurn = false;
        add = true;
      }
    }
    else if(isStable(InputL, SetpointL, 14)){
      firstCheck = true;
    }
    break;
    case blowCandle:
    writeEasyNeoPixel(0, 255, 0, 255);
    L.write(90);
    R.write(90);
    if(millis()>prevMillis+blowTime){
      Blower.write(700);
      lookAgain = false;

      state = turnBack;
    }
    else if(millis()>prevMillis+((blowTime*7)/8)) {
      Blower.write(blowForce);
      R.write(74);
      L.write(100);
    }
    else if(millis()>prevMillis+((blowTime*5)/8)){
      L.write(70);
      R.write(100);
      Blower.write(blowForce);
    }
    else if(millis()>prevMillis+((blowTime*4)/8)){
      R.write(74);
      L.write(100);
      Blower.write(blowForce);
    }
    else{
      Blower.write(((blowForce-1000)/2)+1000);
    }
    break;
    case turnToCandle:
    writeEasyNeoPixel(0, 255, 0,0);
    turn();
    if(firstCheck){
      if(isStable(InputR, SetpointR, 2)){
        firstCheck = false;
        PIDL.SetTunings(Kp, Ki, Kd);
        PIDR.SetTunings(Kp, Ki, Kd);
        state = blowCandle;
        prevMillis = millis();
        encL.write(0);
        encR.write(0);
        add = true;
      }
    }
    else if(isStable(InputL, SetpointL, 2)){
      firstCheck = true;
    }
    break;
    case turnBack:
    writeEasyNeoPixel(0, 255, 140, 0);
    turn270();
    if(firstCheck){
      if(isStable(InputR, SetpointR, 2)){
        firstCheck = false;
        PIDL.SetTunings(Kp, Ki, Kd);
        PIDR.SetTunings(Kp, Ki, Kd);
        if(lookAgain){
          state = turnToCandle;
          leftTurn = false;
        }
        else{
          state = idle;
        }
        prevMillis = millis();
        encL.write(0);
        encR.write(0);
        add = true;
      }
    }
    else if(isStable(InputL, SetpointL, 2)){
      firstCheck = true;
    }
    break;
    case cliffState:
      writeEasyNeoPixel(0, 139, 69,19);
      backwards(300);
      if(add){
        SetpointL = InputL - 300;
        SetpointR = InputR - 300;
        add = false;
      }
      else if(InputL < SetpointL){
        state = waitState;
        cliffAfterTurn= true;
        add = true;
      }
    break;
    case straightUntilWall:
      straightAtCliff();
      writeEasyNeoPixel(0, 0, 255,127);

      if(right < 400 ){
        add = true;
        state = wallFollowState;
      }
      if(front < 220 ){
        add = true;
        addPos();
        state = wallFollowState;
      }
    break;
  }
}

void loop() {
  runState();
  //Serial.println(state);
  if(Serial3.available() > 0){
    char in = Serial3.read();
    int dist = Serial3.readStringUntil('\n').toInt();
    if(in =='F'){
      front = dist;
    }
    else if(in == 'R'){
      right = dist;
    }
    else if(in == 'L'){
      left = dist;
    }
  }

  if(Serial2.available() > 0){
    Serial2.read();
    start = true;
    state = wallFollowState; //first state after go
  }

  if(analogRead(6)>800 && (state == wallFollowState || state == continuePast || state == straightUntilWall)){
    add = true;
    state = cliffState;
  }
  digitalWrite(13, HIGH);
  if((state == wallFollowState || state == continuePast || state == lookCandle || state == turnBack || state == straightUntilWall) && millis()%15 == 0){
    Wire.beginTransmission(slaveAddress);
    Wire.write(0x36);
    Wire.endTransmission();

    Wire.requestFrom(slaveAddress, 16);        // Request the 2 byte heading (MSB comes first)
    for (z=0;z<16;z++) { data_buf[z]=0; }
    z=0;
    while(Wire.available() && z< 16) {
      data_buf[z] = Wire.read();
      z++;
    }
    Ix[0] = data_buf[1];
    Iy[0] = data_buf[2];
    s   = data_buf[3];
    Ix[0] += (s & 0x30) <<4;
    Iy[0] += (s & 0xC0) <<2;
    // Serial.print("IR position   ");
    // Serial.print(int(Ix[0]));
    // Serial.print("   ");
    // Serial.println(int(Iy[0]));
    if(state == turnBack && (Ix[0] < 1023 || Iy[0] < 1023)){
      lookAgain = true;
    }
    else if(Ix[0] < 1023 || Iy[0] < 1023){
      state = lookCandle;
    }
  }
  //
  //S
  //Serial.println(state);
  if(millis()%250==0){
    Serial2.print('X');
    Serial2.println(curX);
  }
  if(millis()%250==125){
    Serial2.print('Y');
    Serial2.println(curY);
  }
}
