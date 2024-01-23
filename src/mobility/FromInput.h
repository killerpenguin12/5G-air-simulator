
#ifndef FROMINPUT_H_
#define FROMINPUT_H_

#include "Mobility.h"

class InputPosition: public Mobility
{
public:
  InputPosition(double x, double y, int speed, double direction);
  virtual ~InputPosition();

  virtual void UpdatePosition (double x, double y);
  virtual void UpdateSpeed (int speed, double direction);
};

#endif /* FROMINPUT_H_ */
