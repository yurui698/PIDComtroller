#ifndef __MYPID_H
#define __MYPID_H
#include "stm32f10x.h"
/*PID结构体定义*/
typedef struct {
  float maximum;               /*输出值上限*/
  float minimum;                /*输出值下限*/

  float setpoint;                /*设定值*/
  float kp;                      /*比例系数*/
  float ki;                     /*积分系数*/
                       

  float lasterror;              /*前一拍偏差*/
  float preerror;               /*前两拍偏差*/
  float result;                /*PID控制器结果*/
  float output;                 /*输出值*/

  float errorabsmax;
  float errorabsmin;
  float deadband;               /*死区*/
  float integralValue;
}vPID;

#define MOTOR1_UP   	do{GPIO_ResetBits(GPIOC, GPIO_Pin_3); GPIO_SetBits(GPIOB, GPIO_Pin_12);   }while(0)
#define MOTOR1_DOWN 	do{GPIO_SetBits(GPIOC, GPIO_Pin_3);   GPIO_ResetBits(GPIOB, GPIO_Pin_12); }while(0)
#define MOTOR1_STOP 	do{GPIO_SetBits(GPIOC, GPIO_Pin_3);   GPIO_SetBits(GPIOB, GPIO_Pin_12);   }while(0)
#define MOTOR2_UP   	do{GPIO_ResetBits(GPIOB, GPIO_Pin_14);GPIO_SetBits(GPIOB, GPIO_Pin_13);   }while(0)
#define MOTOR2_DOWN 	do{GPIO_SetBits(GPIOB, GPIO_Pin_14);  GPIO_ResetBits(GPIOB, GPIO_Pin_13); }while(0)
#define MOTOR2_STOP 	do{GPIO_SetBits(GPIOB, GPIO_Pin_14);GPIO_SetBits(GPIOB, GPIO_Pin_13);}while(0)

void PID_init(vPID *mypid,float vMin,float vMax,float sp);//sp为setpoint
vs8 PIDRegulator(vPID *mypid,float pv);//pv为processvalue 实际测量值
void motor_ctrl(u8 t_motor);

#endif
