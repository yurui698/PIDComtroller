#include <math.h>
#include <stdio.h>
#include "myPID.h"


/*PID初始化函数*/

void PID_init(vPID *mypid, float vMin, float vMax, float sp)
{
	mypid->maximum = vMax;                /*输出值上限*/
	mypid->minimum = vMin;                /*输出值下限*/

	mypid->setpoint = sp;                 /*设定值*/
	mypid->kp = 0.3;                      /*比例系数*/
	mypid->ki = 0.35;                     /*积分系数*/

	mypid->lasterror = 0.0;              /*前一拍偏差*/
              
	mypid->result = vMin;                /*PID控制器结果*/


	mypid->errorabsmax = (vMax - vMin)*0.8f;
	mypid->errorabsmin = (vMax - vMin)*0.2f;

	mypid->deadband = (vMax - vMin)*0.005f;               /*死区*/
	               
 		
}

void PIDRegulator(vPID *mypid,float processvalue) //pv为processvalue 实际测量值
{
  float thisError;
  float result;
  float increment;
  float pError,iError,dError;
  float factor;
	processvalue *= 7.5f; //频率转换流量
  u8  index = 1;
  thisError=mypid->setpoint-processvalue; //得到偏差值
 
  result=mypid->result;
  factor=VariableIntegralCoefficient(thisError,mypid->errorabsmax,mypid->errorabsmin);
  
  if (fabs(thisError)>mypid->deadband) //误差大于死区
  {
    pError=thisError-mypid->lasterror; //比例误差
    iError=thisError;		                //积分误差
    
    
  
    increment=mypid->kp*pError+mypid->ki*factor*iError;//增量计算,去除kd
  }
  else
  {
    if((fabs(mypid->setpoint-mypid->minimum)<mypid->deadband)&&(fabs(processvalue-mypid->minimum)<mypid->deadband)) //当实际值接近最小值且设定值在死区内,最小输出
    {
      result=mypid->minimum;
    }
    increment=0.0;
  }

  result=result+increment;  //计算输出

  /*对输出限值，避免超调和积分饱和问题*/
  if(result>=mypid->maximum)
  {
    result=mypid->maximum;
  }
  if(result<=mypid->minimum)
  {
    result=mypid->minimum;
  } 

    //存放偏差用于下次运算
	
  mypid->lasterror=thisError;
  mypid->result=result;
 //流量增量和阀门开度换算
 
	
//	motor_ctrl(t_motor);
	
}
float VariableIntegralCoefficient(float error,float absmax,float absmin)
{
  float factor=0.0;

  if(fabs(error)<=absmin)
  {
    factor=1.0;
  }
  else if(fabs(error)>absmax)
  {
    factor=0.0;
  }
  else
  {
    factor=(absmax-fabs(error))/(absmax-absmin);
  }

  return factor;
}

void motor_ctrl(u8 t_motor)
{
//	if(t_motor > 0)
//	{
//		motorflg = 1;
//	}else if(t_motor < 0)
//	{
//		motorflg = 2;
//	}
//	
//	Start_timerEx( MOTOR_CTR_EVT,t_motor );
//	
}
void PIDTest(u16 setpoint,u16 processvalue)
{
	vs16 error = 0;
	error = setpoint - processvalue;
	/*控制正反转*/
	if(error > 0){motorflg = 1;}        
  else if(error < 0){motorflg = 2;}
	else {motorflg = 0;}
	
	/*控制转速*/
	if(abs(error) > (u16)(processvalue)/2)  //快速
	{
		t_on =  t_speed[0];
	}
	if( (abs(error) < (u16)(processvalue)/2) && (abs(error) > (u16)(processvalue)/8) )  //中速
	{
		t_on =  t_speed[1];
	}
	else{t_on =  t_speed[2];}   //慢速
	
		t_off = 1000 - t_on;
	
}
