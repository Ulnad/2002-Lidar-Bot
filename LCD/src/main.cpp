#include <Arduino.h>
#include <Monitor.h>
Monitor screen(1);
bool start = false;
bool ret = false;
int X = 0;
int Y = 0;
int Z = 0;
void setup() {
    Serial3.begin(115200);
      screen.setup();
    // put your setup code here, to run once:
}

void loop() {
    ret = screen.run(0,X,Y,Z);
    if(!start && ret){
      Serial3.write('S');
      start = true;
    }
    if(Serial3.available() > 0){
      int in = Serial3.read();
      if(in == 'X'){
        int xt = Serial3.readStringUntil('\n').toInt();
        if(xt != 0){
          X = xt;
        }
      }
      else if(in == 'Y'){
        int yt = Serial3.readStringUntil('\n').toInt();
        if(yt != 0){
          Y = yt;
        }
      }
      else if(in == 'Z'){
        int zt = Serial3.readStringUntil('\n').toInt();
        if(zt != 0){
          Z = zt;
        }
      }
    }
    // put your main code here, to run repeatedly:
}
