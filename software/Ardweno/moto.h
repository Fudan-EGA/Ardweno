#ifndef MOTO_H_
#define MOTO_H_

#include<Arduino.h>
#include<dummy.h>

/************电机参数宏**************/
#define PWML1 13
#define PWML2 14
#define PWMR1 33
#define PWMR2 32
#define RCODE 26
#define LCODE 25
/**********************************/

//extern int direcL=1;   //电机方向标志
//extern int direcR=1;   //电机方向标志

/*使用ESP32LEDC库函数配置为pwm输出模式，16个通道：0-7为高速模式，8-15为低速模式 */
void moto_pwm_init();

/* 根据pwm输出电机转速，适用于驱动芯片DRV8833，正反方向根据pwm正负选择*/
void moto_pwm_set(uint8_t moto, int pwm);


#endif/* MOTO_H_ */
