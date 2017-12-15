#ifndef ENTITY_H
#define ENTITY_H

#include <Arduino.h>

class Entity
{
public:

  virtual void update() = 0;
  virtual void initlaize() = 0;

  float x() const { return x_; }
  float y() const { return y_; }
  float z() const { return z_; }

  void setX(float x) { x_ = x; }
  void setY(float y) { y_ = y; }
  void setZ(float z) { z_ = z; }

  float x_=0;
  float y_=0;
  float z_=0;
};


#endif //ENTITY_H
