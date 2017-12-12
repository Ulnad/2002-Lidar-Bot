/*
  monitor.h - Library for flashing tft screen.
  Created by Danny Lu
  Released into the public domain.
*/
#ifndef Monitor_h
#define Monitor_h
#include "Arduino.h"

class Monitor{
  public:
    Monitor(int state);
    void setup();
    bool run(int print,int X, int Y, int Z);
  private:
    int _state;
    int X;
    int Y;
    int Z;
};

#endif
