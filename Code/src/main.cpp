#include <Arduino.h>
#include <Monitor.h>
String str;         // a String to hold incoming data
int angle;
int distance;
int maxDist = 0;
int position[180]; //stores lidar data
int x[180];
int y[180];
const int angleCalibration = 0;
Monitor screen(1);

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
  if(millis() % 1000 == 500)
    updateFrame();
  if(millis() % 1000 == 0)
    screen.run(maxDist,x,y);
}
}
