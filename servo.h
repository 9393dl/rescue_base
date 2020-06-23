#ifndef _SERVO_H_
#define _SERVO_H_

#include "config.h"

class Servo {
	public:
		float angle;
		Servo(Servo_TypeDef _servo, uint32_t _arr, uint32_t _psc);
//		void updateAngle(float _angle);
		void init();
		void spin(float _angle);
	
	private:
		Servo_TypeDef servo;
		uint32_t arr;
		uint32_t psc;
	
		uint16_t angle_to_pwm(float _angle);
//		void servo_pwm_init();
		
};
#endif //_SERVO_H_