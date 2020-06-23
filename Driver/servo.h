#ifndef _SERVO_H_
#define _SERVO_H_

#include "config.h"

class Servo {
public:
	Servo(Servo_TypeDef _servo, bool Reverse);
	void init();
	void pos(float angle);
	float angle;
  bool reverse;
private:
	Servo_TypeDef servo;
	uint16_t angle_to_pwm(float _angle);
};

#endif //_SERVO_H_
