#include <math.h>
#include <stdio.h>
typedef struct {
  float maximum;               /*输出值上限*/
  float minimum;                /*输出值下限*/

  float setpoint;                /*设定值*/
  float kp;                      /*比例系数*/
  float ki;                     /*积分系数*/
  float kd;              

  float lasterror;              /*前一拍偏差*/
  float preerror;               /*前两拍偏差*/
  float result;                /*PID控制器结果*/
  float output;                 /*输出值*/
 float deltadiff;
 float alpha;
  float errorabsmax;
  float errorabsmin;
  float deadband;               /*死区*/
  float integralValue;
}vPID;
float pv;
float VariableIntegralCoefficient(float error,float absmax,float absmin);


/*PID初始化函数*/

void PID_init(vPID *mypid, float vMin, float vMax, float sp)
{
	mypid->maximum = vMax;                /*输出值上限*/
	mypid->minimum = vMin;                /*输出值下限*/

	mypid->setpoint = sp;                 /*设定值*/
	mypid->kp = 0.3;                      /*比例系数*/
	mypid->ki = 0.35;                     /*积分系数*/
	mypid->kd = 0.01; 
	mypid->lasterror = 0.0;              /*前一拍偏差*/
	mypid->preerror = 0.0;               /*前两拍偏差*/
	mypid->result = vMin;                /*PID控制器结果*/
	mypid->output = 0.0;                 /*输出值*/

	mypid->errorabsmax = (vMax - vMin)*0.8;
	mypid->errorabsmin = (vMax - vMin)*0.2;

	mypid->deadband = (vMax - vMin)*0.005;               /*死区*/
	mypid->alpha=0.2;                  /*不完全微分系数*/
 	 mypid->deltadiff=0.0;
	mypid->integralValue = 0.0;
}

int PIDRegulator(vPID *mypid,float processvalue) //pv为processvalue 实际测量值
{
  float thisError;
  float result;
  float increment;
  float pError,iError,dError;
  float factor;
  int  index = 1;
  thisError=mypid->setpoint-processvalue; //得到偏差值
   printf("mypid->setpoint = %f\n",mypid->setpoint);
    printf("processvalue = %f\n",processvalue);
  printf("thisError = %f\n",thisError);
  result=mypid->result;
  factor=VariableIntegralCoefficient(thisError,mypid->errorabsmax,mypid->errorabsmin);
  
  if (fabs(thisError)>mypid->deadband)
  {
    pError=thisError-mypid->lasterror;
    iError=thisError;
    printf("pError = %f\n",pError);
    printf("iError = %f\n",iError);
   dError=thisError-2*(mypid->lasterror)+mypid->preerror;
   mypid->deltadiff= mypid->kd*(1-mypid->alpha)*dError+mypid->alpha*mypid->deltadiff;
   printf("mypid->deltadiff = %f\n",mypid->deltadiff);
    increment=mypid->kp*pError+mypid->ki*factor*iError+mypid->deltadiff;//增量计算,去除kd
  }
  else
  {
    if((fabs(mypid->setpoint-mypid->minimum)<mypid->deadband)&&(fabs(processvalue-mypid->minimum)<mypid->deadband))
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
  pv = result;
  printf("result = %f\n",mypid->result);
  printf("increment = %f\n",increment);

	return 0;    //电机开转时间,单位s,thisError为正则正转,阀门开大,否则反转
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

int main(int argc, char const *argv[])
{
	int i = 0;
	int t = 0;
	vPID mypid;
	PID_init(&mypid, 0, 100, 70);
	for (i = 0; i < 100; i++)
	{
		printf("第%d次PID\n", i);
		t = PIDRegulator(&mypid, pv);
		printf("t = %d\n", t);
		printf("\n");
	}
	return 0;
}