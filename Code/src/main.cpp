#include <Arduino.h>
#include <Monitor.h>
String str;         // a String to hold incoming data
const int minCandleSpace = 110;
const int maxContinous = 70;
int contCounter = 1;
bool findPosEdge = true;
int angle;
int distance;
int maxDist = 0;
int candle = -99;
int position[180]; //stores lidar data
int x[180];
int y[180];
const int angleCalibration = 0;
Monitor screen(1);

int findCandle(){
  // for(int i = 0; i < 179; i++){
  //   if(contCounter > 12){
  //     findPosEdge = true;
  //     contCounter = 1;
  //   }
  //   if(position[i] - position[i+1] > minCandleSpace && findPosEdge){
  //     findPosEdge = false;
  //   }
  //   else if(abs(position[i] - position[i+1]) < maxContinous && !findPosEdge){
  //     contCounter++;
  //   }
  //   else if(position[i+1] - position[i] > minCandleSpace && contCounter >=2 && !findPosEdge){
  //     findPosEdge = true;
  //     contCounter = 1;
  //     return i-(contCounter/2);
  //   }
  // }
  // findPosEdge = true;
  // contCounter = 1;
  // return -1;
  for(int a = 134; a < 225; a++){
    int i = a%180;
    if(position[i] < 500){
      if(contCounter > 20){
        findPosEdge = true;
        contCounter = 1;
      }
    }
    else{
      if(contCounter > 12){
        findPosEdge = true;
        contCounter = 1;
      }
    }
    if(position[i] - position[i+1] > minCandleSpace && findPosEdge){
      findPosEdge = false;
    }
    else if(abs(position[i] - position[i+1]) < maxContinous && !findPosEdge){
      contCounter++;
    }
    else if(position[i+1] - position[i] > minCandleSpace && contCounter >=2 && !findPosEdge){
      findPosEdge = true;
      contCounter = 1;
      if(i-(contCounter/2) < 0){
        return (180-(i-(contCounter/2)))*2;
      }
      else{
        return (i-(contCounter/2))*2;
      }
    }
  }
  findPosEdge = true;
  contCounter = 1;
  return -99;
}
//FUNCTIONS:
void updateFrame(){
  maxDist = 0;
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
}

void loop() {
  // print the string when a newline arrives:
  if(Serial1.available() > 0){
        str = Serial1.readStringUntil('\n');
        angle = str.substring(0,3).toInt()-100;
        distance = str.substring(3).toInt();
        if(angle >= 0 && angle < 360 && distance < 10000){
          position[(359-angle)/2] = distance;
        }
  }
  else{
  if(millis() % 1200 == 600)
    updateFrame();
  if(millis() % 1200 == 0)
    screen.run(candle,x,y);
  }

  if(millis()%400 == 0){
    candle = findCandle();
    if(findCandle()!=-99){
      digitalWrite(50, HIGH);
    }
    else{
      digitalWrite(50, LOW);
    }
  }

}
