#ifndef __MYPID_H
#define __MYPID_H
#include "stm32f10x.h"
#include "systemclock.h"
/*PID结构体定义*/
typedef struct {
  float maximum;               /*输出值上限*/
  float minimum;                /*输出值下限*/

  float setpoint;                /*设定值*/
  float kp;                      /*比例系数*/
  float ki;                     /*积分系数*/
            

  float lasterror;              /*前一拍偏差*/
               
  float result;                /*PID控制器结果*/
 
 
 
  float errorabsmax;
  float errorabsmin;
  float deadband;               /*死区*/
 
}vPID;


extern u16 t_speed[3];
extern u16 t_on,t_off;
extern u8 motorflg;
void PID_init(vPID *mypid,float vMin,float vMax,float sp);//sp为setpoint
void PIDRegulator(vPID *mypid,float processvalue); 
void motor_ctrl(u8 t_motor);
void PIDTest(u16 setpoint,u16 processvalue); 


float VariableIntegralCoefficient(float error,float absmax,float absmin);
//u8 motorflg;
#endif
