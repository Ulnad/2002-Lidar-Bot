#include <Arduino.h>
#include <Monitor.h>
String str;         // a String to hold incoming data
const int minCandleSpace = 300;
const int minWallSpace = 400;
const int maxContinous = 100;
const int maxWallContinous = 100;
int contCounter = 1;
bool findPosEdge = true;
bool start = false;
bool inita;

int driveDist = 100;
char input;
int blowAngle;
int angle;
int distance;
int maxDist = 0;
int candle = -99;
int position[180]; //stores lidar data
int x[180];
int y[180];
const int angleCalibration = 0;
Monitor screen(1);
static enum states {lookCandle, idle, lookNextMovement, oneShot}
state;

int findGoAngle(){
  Serial.print("distPOS:");
  int acc = 1;
  bool negEdge = false;
  findPosEdge = true;
  for(int a = 134; a < 265; a++){//
    int i = a%180;
      if(acc > 2){
        int edgeAngle = (i-acc)*2;
        int addAngle;
        int distPOS = position[(a-1)%180];
        int distNEG = position[(a-acc-1)%180];

        if(negEdge){
          addAngle = degrees(asin((double)350/(double)distNEG));
          driveDist = distNEG;

        }//235
        else{
          addAngle = degrees(asin((double)190/(double)distPOS));
          driveDist = distPOS;
        }

        Serial.println(distPOS);
        Serial.print("distNEG:");
        Serial.println(distNEG);
        Serial.print("angle:");
        Serial.println(edgeAngle);
        Serial.print("neg");
        Serial.println(String(negEdge));
        if(negEdge){
          negEdge = false;
          acc = 1;
          if(edgeAngle<0 || (abs(distPOS - distNEG) < minWallSpace || distPOS>2450 || distNEG>2450 || distPOS<200 || distNEG<200 ) ){

            return -999;

          }

          return edgeAngle+addAngle;
        }
        else{
          acc=1;
          if(edgeAngle<0 || (abs(distPOS - distNEG) < minWallSpace || distPOS>2450 || distNEG>2450 || distPOS<200 || distNEG<200)){
            return -999;
          }
          return edgeAngle-addAngle;
        }
      }
      if(abs(position[i] - position[i+1]) > minWallSpace && findPosEdge){ //found positive edge
        if(position[i] - position[i+1] < 0){
          negEdge = true;
        }
        findPosEdge = false;
      }
      else if(abs(position[i] - position[i+1])<maxContinous && !findPosEdge){
        acc++;
      }
      else if(abs(position[i] - position[i+1])>maxContinous && !findPosEdge){
        acc = 1;
        negEdge = false;
        findPosEdge = true;
      }
    }

  //
  // int minCandleAmt = 2;
  // for(int a = 134; a < 225; a++){
  //   int i = a%180;
  //   if(position[i] - position[i+1] > minWallSpace && findPosEdge){ //found positive edge
  //     findPosEdge = false;
  //   }
  //   else if(abs(position[i-contCounter+1] - position[i+1]) < maxContinous && !findPosEdge){ //add if within continous range
  //     contCounter++;
  //   }
  //   else if( contCounter > 2 && !findPosEdge){ //return if bigger than min and negative edge
  //       findPosEdge = true;
  //       //int deg1 = i-contCounter; //deg2 = i
  //       // len1 = position[deg1]    len2 = position[i]
  //       //int candleWidth = sqrt((double)position[deg1]*(double)position[deg1]+(double)position[i]*(double)position[i]-(2*(double)position[deg1]*(double)position[i]*cos((double)(deg1-i)*2.0*57.29578)));
  //       if(i-contCounter < 0){
  //         int ret = (180-(i-(contCounter/2)))*2;
  //         contCounter = 1;
  //         return ret;
  //       }
  //       else{
  //         int ret =  (i-contCounter)*2;
  //         contCounter = 1;
  //         return ret;
  //       }
  //
  //     contCounter = 1;
  //   }
  //   else if(!(abs(position[i-contCounter+1] - position[i+1]) < maxWallContinous) && !findPosEdge){ //resets if not continous
  //     a--;
  //     findPosEdge = true;
  //     contCounter = 1;
  //   }
  // }
  findPosEdge = true;
  // contCounter = 1;
  return -999;
}
int findCandle(){
  int minCandleAmt = 2;
  int maxCandleAmt = 10;
  for(int a = 134; a < 225; a++){
    int i = a%180;
    if(contCounter > maxCandleAmt){ //reset if gap too large
        findPosEdge = true;
        contCounter = 1;
    }
    if(position[i] - position[i+1] > minCandleSpace && findPosEdge){ //found positive edge
      findPosEdge = false;
    }
    else if(abs(position[(a-contCounter+1)%180] - position[i+1]) < maxContinous && !findPosEdge){ //add if within continous range
      contCounter++;
    }
    else if(position[i+1] - position[i] > minCandleSpace && !findPosEdge){ //return if bigger than min and negative edge
      findPosEdge = true;
      if(contCounter >=minCandleAmt){
        //int deg1 = i-contCounter; //deg2 = i
        // len1 = position[deg1]    len2 = position[i]
        //int candleWidth = sqrt((double)position[deg1]*(double)position[deg1]+(double)position[i]*(double)position[i]-(2*(double)position[deg1]*(double)position[i]*cos((double)(deg1-i)*2.0*57.29578)));
        if(i-(contCounter/2) < 0){
          int ret = (180-(i-(contCounter/2)))*2;
          contCounter = 1;
          return ret;
        }
        else{
          int ret =  (i-(contCounter/2))*2;
          contCounter = 1;
          return ret;
        }
      }
      contCounter = 1;
    }
    else if(!(abs(position[i-contCounter+1] - position[i+1]) < maxContinous) && !findPosEdge){ //resets if not continous
      a--;
      findPosEdge = true;
      contCounter = 1;
    }
  }
  findPosEdge = true;
  contCounter = 1;
  return -99;
}
//FUNCTIONS:
void updateFrame(){
  maxDist = 10;
  for(int i = 0; i < 180; i++){
    if(position[i]>maxDist)
      maxDist = position[i];
    y[i] = (int)((double)position[i]*cos(((double)i * 71 *2) / 4068));
    x[i] = (int)((double)position[i]*sin(((double)i * 71 *2) / 4068));

  }

}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial3.begin(115200);
  Serial.println("Initalized");
  for(int i = 0; i < 180; i++){
    position[i] = 1000; //set all angle positions into 1m
  }
  // position[0] =200;
  // position[180] =200;
  // position[90] =200;
  // position[270] =200;
  screen.setup();
  pinMode(50, OUTPUT);
  state = idle;
}

void runState(){
  switch(state){
    case lookCandle:
      if(blowAngle==-99){
        state = lookNextMovement;
      }
      else{
        Serial3.print('*');
        Serial3.println(blowAngle);
        // Serial.print(blowAngle);
      }
      break;
    case idle:
      break;
    case lookNextMovement:
      break;
    case oneShot:
      int x = findGoAngle();
      if(x!= -999){
        Serial3.print('#');
        Serial3.print(String(x+200));
        Serial3.println(String(driveDist+350));
               // Serial.print('#');
               //  Serial.println(String(x));
        state = idle;
      }
      break;
    }
  }

void loop() {
  runState();
  // print the string when a newline arrives:
  if(Serial1.available() > 0){
        str = Serial1.readStringUntil('\n');
        angle = str.substring(0,3).toInt()-100;
        distance = str.substring(3).toInt();
        if(angle >= 0 && angle < 360 && distance < 10000 && distance > 1){
          position[(359-angle)/2] = distance;
        }
  }
  else{
  //if(millis() % 1200 == 600)
    //updateFrame();
  if(millis() % 1200 == 0)
    inita = screen.run(blowAngle,maxDist,x,y);
    if(inita && !start){
      start = true;
      state = oneShot; //first state after go
    }
  }

  // if(millis()%400 == 0){
  //   Serial.println(state);
  //   blowAngle = findCandle();
  //   if(findCandle()!=-99){
  //     digitalWrite(50, HIGH);
  //   }
  //   else{
  //     digitalWrite(50, LOW);
  //   }
  // }
  if(Serial3.available() > 0){
    input = Serial3.read();
    if(input == ' '){

    }
  }
}
