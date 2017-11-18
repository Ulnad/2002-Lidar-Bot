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
    void run(int max, int x[], int y[]);
  private:
    int _state;
    int Px[180];
    int Py[180];
};

#endif
