#include "timer.h"
#include "led.h"

//通用定时器中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
//这里使用的是定时器3!

//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void PWM_Init(u16 arr,u16 psc,u16 ccr1,u16 ccr2)
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB1ENR|=1<<1;       //TIM3时钟使能
	RCC->APB2ENR|=1<<2;      //使能GPIOA


	GPIOA->CRL&=0X00FFFFFF;//PA7 PA6输出
	GPIOA->CRL|=0XBB000000;//复用功能输出 	  
	GPIOA->ODR|=0<<7;//PA7上拉
	GPIOA->ODR|=0<<6;


	TIM3->ARR=arr-1;//设定计数器自动重装值 
	TIM3->PSC=psc;//预分频器不分频
	
	TIM3->CCMR1|=7<<12;  //CH2 PWM2模式		 
	TIM3->CCMR1|=1<<11; //CH2预装载使能
	TIM3->CCMR1|=7<<4;   //CH1 使能
 	TIM3->CCMR1|=1<<3;   //CH1预装载使能
		   
	TIM3->CR1&=0<<4;     //向上计数模式
	TIM3->CCER|=1<<4;   //OC2 输出使能	
	TIM3->CCER|=1<<0;   // OC1 输出使能
	
	TIM3->BDTR|=1<<15;  //开启OC和OCN输

	TIM3->CCR1=ccr1; //设定占空比
    TIM3->CCR2=ccr2; 
    TIM3->EGR|=1<<0;    //重新初始化计数器	

	TIM3->CR1=0x8000;   //ARPE使能 
	TIM3->CR1|=0x01;    //使能定时器3 		



  
}
 	 
















