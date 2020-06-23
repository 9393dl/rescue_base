#include "servo.h"

GPIO_TypeDef* SERVO_PORT[SERVOn] = {RIKI_SERVO1_GPIO_PORT, RIKI_SERVO2_GPIO_PORT};
TIM_TypeDef*  SERVO_TIM[SERVOn] = {RIKI_SERVO1_TIM, RIKI_SERVO2_TIM};

const uint32_t SERVO_PORT_CLK[SERVOn] = {RIKI_SERVO1_GPIO_CLK, RIKI_SERVO2_GPIO_CLK};
const uint32_t SERVO_TIM_CLK[SERVOn] = {RIKI_SERVO1_TIM_CLK, RIKI_SERVO2_TIM_CLK};
const uint16_t SERVO_PIN[SERVOn] = {RIKI_SERVO1_PIN, RIKI_SERVO2_PIN};

Servo::Servo(Servo_TypeDef _servo, uint32_t _arr, uint32_t _psc)
{
	servo = _servo;
	arr = _arr;
	psc = _psc;
}

void Servo::init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(SERVO_PORT[this->servo], ENABLE);
	
	GPIO_InitStructure.GPIO_Pin     = SERVO_PIN[this->servo];
	GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_PORT[this->servo], &GPIO_InitStructure);
	
	servo_pwm_init();
}

void Servo::servo_pwm_init()
{
	//pwm value ((1 + psc)/72M)*(1+arr)
	//eg: ((1+143)/72M)*(1+9999) = 0.02s --10000 count use 0.02s
	//set arduino pwm value 490hz 255 count 
	//((1 + 575)/72M)(1 + 254) = (1 / 490)
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(SERVO_TIM_CLK[this->servo], ENABLE);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	
	TIM_TimeBaseInit(SERVO_TIM[this->servo], &TIM_BaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	
	if(this->servo == SERVO1){
	//	TIM_OC1Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
	//	TIM_OC1PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);

    TIM_OC3Init(SERVO_TIM[this->servo], &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(SERVO_TIM[this->servo], TIM_OCPreload_Enable);

	
	}

	if(this->servo == SERVO2) {
	//	TIM_OC2Init(MOTOR_PWM_TIM[this->motor], &TIM_OCInitStructure);
	//	TIM_OC2PreloadConfig(MOTOR_PWM_TIM[this->motor], TIM_OCPreload_Enable);

      TIM_OC4Init(SERVO_TIM[this->servo], &TIM_OCInitStructure);
	   	TIM_OC4PreloadConfig(SERVO_TIM[this->servo], TIM_OCPreload_Enable);
	
	}
	TIM_ARRPreloadConfig(SERVO_TIM[this->servo], ENABLE);
	
	TIM_CtrlPWMOutputs(SERVO_TIM[this->servo], ENABLE);
	TIM_Cmd(SERVO_TIM[this->servo], ENABLE);
}


void Servo::spin(float _angle)
{
	uint16_t pwm;
	if(_angle > 180) _angle = 180;  //待修改
    else if(_angle < 0) _angle = 0;
	
	this->angle = _angle;
	
	pwm = angle_to_pwm(_angle);
	
	if(this->servo == SERVO1){
		TIM_SetCompare1(SERVO_TIM[this->servo], pwm);
	}
	if(this->servo == SERVO2){
		TIM_SetCompare2(SERVO_TIM[this->servo], pwm);
	}
}

uint16_t angle_to_pwm(float _angle)
{
	uint16_t pwm_val;

	pwm_val = (uint16_t)(50.0*_angle/9.0+249.0); 
	return 	pwm_val;
}