#include "servo.h"

GPIO_TypeDef* SERVO_PORT[SERVOn] = {RIKI_SERVO1_GPIO_PORT, RIKI_SERVO2_GPIO_PORT};
TIM_TypeDef*  SERVO_TIM[SERVOn] = {RIKI_SERVO1_TIM, RIKI_SERVO1_TIM};
const uint32_t  SERVO_PORT_CLK[SERVOn] = {RIKI_SERVO1_GPIO_CLK, RIKI_SERVO2_GPIO_CLK};
const uint16_t  SERVO_PIN[SERVOn] = {RIKI_SERVO1_PIN, RIKI_SERVO2_PIN};
const uint32_t  SERVO_TIM_CLK[SERVOn] = {RIKI_SERVO1_TIM_CLK, RIKI_SERVO2_TIM_CLK};

Servo::Servo(Servo_TypeDef _servo, bool Reverse)
{
	servo = _servo;
	angle = 0;
	reverse = Reverse;
}

void Servo::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(SERVO_PORT_CLK[this->servo], ENABLE);
	RCC_APB1PeriphClockCmd(SERVO_TIM_CLK[this->servo], ENABLE);

	GPIO_InitStructure.GPIO_Pin     = SERVO_PIN[this->servo];
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(SERVO_PORT[this->servo], &GPIO_InitStructure);

	TIM_BaseInitStructure.TIM_Period                = 9999;
	TIM_BaseInitStructure.TIM_Prescaler             = 143;
	TIM_BaseInitStructure.TIM_ClockDivision         = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_CounterMode           = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter     = 0;

	TIM_TimeBaseInit(SERVO_TIM[this->servo], &TIM_BaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 

	if(this->servo == SERVO1){
		TIM_OC3Init(SERVO_TIM[this->servo], &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(SERVO_TIM[this->servo], TIM_OCPreload_Enable);
	}

	if(this->servo == SERVO2){
		TIM_OC4Init(SERVO_TIM[this->servo], &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(SERVO_TIM[this->servo], TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(SERVO_TIM[this->servo], ENABLE);

	TIM_CtrlPWMOutputs(SERVO_TIM[this->servo], ENABLE);
	TIM_Cmd(SERVO_TIM[this->servo], ENABLE);
}

uint16_t Servo::angle_to_pwm(float _angle)
{
	uint16_t pwm_val;
	_angle=_angle+90.0;
	pwm_val = (u16)(50.0*_angle/9.0+249.0); 
	return 	pwm_val;
}

void Servo::pos(float _angle)
{
	uint16_t pwm;
	if (reverse)
		_angle = (-1) * _angle;
	if(_angle > 90) angle = 90;
    else if(_angle < -90) angle = -90;
	else angle = _angle;
	pwm = angle_to_pwm(angle);
	if(this->servo == SERVO1){
		TIM_SetCompare3(SERVO_TIM[this->servo], pwm);
	}

	if(this->servo == SERVO2){
		TIM_SetCompare4(SERVO_TIM[this->servo], pwm);
	}
}


