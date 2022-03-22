#include "moto.h"

int direcL=1;   //电机方向标志
int direcR=1;   //电机方向标志

/*使用ESP32LEDC库函数配置为pwm输出模式，16个通道：0-7为高速模式，8-15为低速模式 */
void moto_pwm_init()
{
  ledcSetup(8, 1000, 10);  //设置LEDC通道8频率为1000，分辨率为10位，即占空比可选0~1023
  ledcAttachPin(PWMR1, 8); //设置LEDC通道8在IO上输出
  ledcSetup(9, 1000, 10);  
  ledcAttachPin(PWMR2, 9); //右边电机正反两个通道

  ledcSetup(10, 1000, 10);  
  ledcAttachPin(PWML1, 10); 
  ledcSetup(11, 1000, 10);  
  ledcAttachPin(PWML2, 11); //左边电机正反两个通道
}

#define LEFT        0x02
#define RIGHT       0x03

/* 根据pwm输出电机转速，适用于驱动芯片DRV8833，正反方向根据pwm正负选择*/
void moto_pwm_set(uint8_t moto, int pwm)
{ 
  if(moto==LEFT){
    if(pwm<0){
      direcL=-1;
      ledcWrite(10, -pwm); //设置输出PWM占空比
      ledcWrite(11, 0);
    }
    else{
      direcL=1;
      ledcWrite(11, pwm); //设置输出PWM占空比
      ledcWrite(10, 0);
    }
  }
  else{
    if(pwm<0){
      direcR=-1;
      ledcWrite(9, -pwm); //设置输出PWM占空比
      ledcWrite(8, 0);
    }
    else{
      direcR=1;
      ledcWrite(9, 0);
      ledcWrite(8, pwm); //设置输出PWM占空比
    }
  }
}
