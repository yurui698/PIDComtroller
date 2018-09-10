/* 子站ID：PA.00,PC.02,PB.10,PB.11PB.08,PB.09(1--6) ；无线信道：PD.02 PB.03~PB.06(1---5)；控制通道：J0---PC.03,J1---PB.12,J3---PB.13,J2---PB.14
电流检测：I0---PC0(IN10)，I1---PB0(IN8),I2---PC1(IN11)，I3---PB1(IN9);电力载波串口2:TXD2---PA3 ,RXD2---PA2,RTS2---PA1;串口3:TXD3---PC10,RXD3---PC11,RTS3---PA15,Remap
SWD接口:SWCLK---PA14,SWDIO---PA13,NRST---NRST;TTL串口1：TXD1---PA9，RXD1---PA10；有线指示--PC5;无线指示--PC12*/
#include "HandleTask.h"
#include "systemclock.h"
#include "string.h"
#include "DataFlash.h"
#include "Stm32_Configuration.h"
#include "si4463.h"

/*Modbus子站ID*/
#define SLAVE_ID                 0x01
#define ID                       0x88
#define DEBUG_ID                 0x99

/*Modbus功能码*/
#define READ_COIL_STATUS         0x01   /*读线圈寄存器  :不支持广播，规定要读的起始线圈和线圈量                  */
#define READ_INPUT_STATUS        0x02	/*读状态寄存器	:不支持广播，规定要读的输入起始地址，以及输入信号的数量  */
#define READ_HOLDING_REGISTER    0x03	/*读保持寄存器	:不支持广播，规定要读的寄存器起始地址及寄存器的数量      */
#define READ_INPUT_REGISTER      0x04 	/*读输入寄存器	:不支持广播，规定要读的寄存器起始地址及寄存器的数量      */
#define WRITE_SINGLE_COIL        0x05 	/*写单线圈寄存器:支持广播，规定被请求线圈的ON/OFF状态，0xFF00值请求线圈处
														 于ON状态，0x0000值请求线圈处于OFF状态，其他值对线圈无效 */
#define WRITE_SINGLE_REGISTER    0x06 	/*写单保持寄存器:支持广播，请求的预置值在查询数据区                      */
#define WRITE_MULTIPLE_COIL      0x0F	/*写多线圈寄存器:支持广播                                                */
#define WRITE_MULTIPLE_REGISTER  0x10	/*写多保持寄存器:支持广播                                                */

#define WirelessLedToggle  		 GPIO_WriteBit(GPIOC,GPIO_Pin_12, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_12)))//PC6-PC12修改
#define YX_LED_TOGGLE  		 GPIO_WriteBit(GPIOC,GPIO_Pin_5, (BitAction)(1-GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_5)))
#define YX_LED_ON               GPIO_ResetBits(GPIOC, GPIO_Pin_5)
/*RS485收发切换,PB12控制*/
#define TXENABLE2		         GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define RXENABLE2		         GPIO_ResetBits(GPIOA, GPIO_Pin_1)

#define CONTROLLER        		0xEE

#define MEASURE_PERIOD          300

#define ROLLER1_UP   	do{GPIO_ResetBits(GPIOC, GPIO_Pin_3); GPIO_SetBits(GPIOB, GPIO_Pin_12);   }while(0)
#define ROLLER1_DOWN 	do{GPIO_SetBits(GPIOC, GPIO_Pin_3);   GPIO_ResetBits(GPIOB, GPIO_Pin_12); }while(0)
#define ROLLER1_STOP 	do{GPIO_SetBits(GPIOC, GPIO_Pin_3);   GPIO_SetBits(GPIOB, GPIO_Pin_12);   }while(0)
#define ROLLER2_UP   	do{GPIO_ResetBits(GPIOB, GPIO_Pin_14);GPIO_SetBits(GPIOB, GPIO_Pin_13);   }while(0)
#define ROLLER2_DOWN 	do{GPIO_SetBits(GPIOB, GPIO_Pin_14);  GPIO_ResetBits(GPIOB, GPIO_Pin_13); }while(0)
#define ROLLER2_STOP 	do{GPIO_SetBits(GPIOB, GPIO_Pin_14);GPIO_SetBits(GPIOB, GPIO_Pin_13);}while(0)
#define CTRL_OPEN0    GPIO_ResetBits(GPIOC, GPIO_Pin_3)
#define CTRL_OPEN1    GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define CTRL_OPEN2    GPIO_ResetBits(GPIOB, GPIO_Pin_14)
#define CTRL_OPEN3    GPIO_ResetBits(GPIOB, GPIO_Pin_13)
#define CTRL_CLOSE0   GPIO_SetBits(GPIOC, GPIO_Pin_3)
#define CTRL_CLOSE1   GPIO_SetBits(GPIOB, GPIO_Pin_12)
#define CTRL_CLOSE2   GPIO_SetBits(GPIOB, GPIO_Pin_14)
#define CTRL_CLOSE3   GPIO_SetBits(GPIOB, GPIO_Pin_13)

#define true 1
#define false 0
#define bool u8

/*CRC低位字节表*/
const unsigned char crc_lo[256]={
	0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,
	0x04,0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
	0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,
	0x1D,0x1C,0xDC,0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
	0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,
	0x37,0xF5,0x35,0x34,0xF4,0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
	0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,
	0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
	0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,
	0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
	0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,0x78,0xB8,0xB9,0x79,0xBB,
	0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
	0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,0x50,0x90,0x91,
	0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
	0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,0x88,
	0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
	0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,
	0x40};


/*CRC高位字节表*/
const unsigned char crc_hi[256]={
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
	0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
	0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,
	0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
	0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
	0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,
	0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
	0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,
	0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,
	0x40};

extern u8   RxFlag2;	//串口2接收标志位，0正在接收，1接收完毕
extern u8   RxFlag1;//串口1接收标志位，0正在接收，1接收完毕

extern u8   TxFlag1;//串口1发送标志位，0正在发送，1发送完毕
extern u8   TxFlag2;	//串口2发送标志位，0正在发送，1发送完毕

extern u8   RecDataBuffer2[]; 
extern u8   RecLen2;
extern u8   RecDataBuffer1[]; 
extern u8   RecLen1;

extern u8   USART1SendTCB[];
extern u8   USART1BufferCNT;
extern u8   USART2SendTCB[];
extern u8   USART2BufferCNT;	

extern __IO uint16_t ADC_ConvertedValue[];

extern u8 	SI4463_RxBUFF[];
extern u8 	SI4463_RxLenth;
extern u8 	Int_BUFF[];

static u8  ReportData2[128];
static u8 ReceiveData2[128];
static u8 RxReport2_len;

__attribute__((section("NO_INIT"),zero_init)) static u16 ControlValue[4];

__attribute__((section("NO_INIT"),zero_init)) static u8  collector_data_buff[16];
static u8  ctrlslave_param_set[24]={0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF,
                                    0xFF,0xFF,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00,0xFF,0xFF};//控制器控制要求设定：最大限流、起始运行时间、持续运行时间
static u8  cjqslave_param_set[36];

static u8  ReadDataCNT2        =  0;
static u8  ReadDataCNT1       =  0;
static u8  slave_ctrl_ID      =  0;
static u8  SI4463_TxBUFF[128];
static u16 regLen;
static u8  SI4463_Channel     =  0;

static u8  roller1_state=0;//0表示停；1表示上行；2表示下行
static u8  roller2_state=0;//0表示停；1表示上行；2表示下行
static u16 MotorCurrent0              =0;
static u16 MotorCurrent1              =0;
static u16 MotorMAX[4]={0,0,0,0};
static u16 MotorCurrent2              =0;
static u16 MotorCurrent3              =0;

static u16 regLen2;

static void RxReport2(u8 len,u8 *pBuf);

static void SI4463RxReport(u8 len,u8 *pData);

static void sensor_data(void);
static u16  GetCRC16(u8 *Msg, u16 Len);
static void WriteDataToBuffer(u8 port,u8 *ptr,u8 start,u8 len);
static void Clear_Buf(u8 buf[], u8 length, u8 data);
//static void	Get_ID(void);
//static void	Get_Channel(void);
static void startadc(void);
static void IO_ctrl_cmd(void);
//参数设定新定义
static u8 single_delay_time[4]={0,0,0,0};//单向控制延时启动时间
static u8 single_delay_flg[4]={0,0,0,0};//单向控制停止运行标志，single_delay_flg[i]!=0则不允许启动
static u8 bidirection_delay_time[4]={0,0,0,0};//正向-停-正向、反向-停-反向 延时启动时间
static u8 bidirection_delay_flg[4]={0,0,0,0};//双向控制停止运行标志，bidirection_delay_flg[i]!=0则不允许启动
static u8 bidirection_run_time[4]={0,0,0,0};//正转-反转切换延时启动时间
static u8 bidirection_run_flg[4]={0,0,0,0};//正转-反转切换运行标志，bidirection_run_flg[i]=0 停状态；=1正向运行；=2 反向运行
static u16 single_old_ControlValue[4] = {0x0000,0x0000,0x0000,0x0000};
static u16 bidirection_old_ControlValue[4] = {0x0008,0x0008,0x0008,0x0008};
static u16 max_limit_current[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};//max_limit_current[i]>0xFFF0 不限流；max_limit_current[i]/65535*量程
static u16 start_run_time[4] = {0x0000,0x0000,0x0000,0x0000};//start_run_time[i]=0x0000 立即启动；单位：秒；65534/3600=18.2小时
static u16 continue_run_time[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};//continue_run_time[i]=0xFFFF,连续运行，不限制；单位：秒；65534/3600=18.2小时
static u32 start_run_nowtime[4] = {0x0000,0x0000,0x0000,0x0000};//start_run_now[i]=0x0000 立即启动；单位：秒；655280/3600=18.133小时;计时数组变量
static u32 continue_run_nowtime[4] = {0xFFFF,0xFFFF,0xFFFF,0xFFFF};//continue_run_time[i]>0xFF00,连续运行，不限制；单位：秒；65534/3600=18.2小时;计时数组变量
static void slave_init_readflash(void);//子站初始化读flash
static u8  bidirection_location_flg[4]={0,0,0,0};//电机运行状态及位置，0或8表示停，1表示正转停的位置，2表示反转停的位置
static u8  control_type[4]={0,0,0,0};//0表示单向控制，1表示正反控制，由收到的控制命令设定的。
static u8  batch_ctrl_finish[4]={0,0,0,0};//0表示批量控制命令完成，1表示正在执行批量控制批量
       u8 close_433MHZ=1;//不使用433MHZ通信或433MHZ通信出问题，则close_433MHZ=0
//以下为自动识别站地址和433MHZ信道定义
static u8  slaveID_radioID[2]={0xFF,0xFF};//站地址、433MHZ信道设定

/*ADC采集函数定义*/
u16 Get_Adclvbo(u8 TD_Xnumber,u16 TD_xiaxian,u16 TD_shangxian);	//采集100个数据，排序取40~60平均值
u16 First_Getaverage(u8 td_xnumber,u8 maxlvbo_xnumber,u16 temp_adc);//初始化平均值
u16 TD_Getaverage(u8 td_xnumber,u8 tdlvbo_xnumber,u16 temp_xadc,u8 tdcycle_xi);//ADC通道求平均值
u16 SelectTD(u8 TD_number);//选择通道，采集各个通道的值
 /*求通道平均值定义*/
const u8 MaxTD_number= 4;		  //4个ADC采集通道
const u8 Maxlvbo_number= 100;	  //平均数组的最大次数
static u8 TDlvbo_number[MaxTD_number]={50,50,50,50};	//通道0的50次数据求平均值，8秒/次
		 								//通道1的50次数据求平均值，8秒/次
static  u16 Adc_average[MaxTD_number][Maxlvbo_number]; //ADC求平均值数组，存放历史采集数据
static u8 tdcycle_i[MaxTD_number]={0,0,0,0}; //循环次数数组，0~TDlvbo_number[i]-1，i为通道号，i=0、1、....
static u8 First_adc_average[MaxTD_number]={0,0,0,0}; 
/*首次滤波，当First_adc_average=0时，调用First_Getaverage(u8 TD_number,u8 Maxlvbo_number,u16 temp_adc)
否则调用  TD_Getaverage(u8 TD_number,u8 TDlvbo_number[TD_number],u16 temp_adc,u8 tdcycle_i[TD_number])*/
//static u8 test[50];
//事件运行监控定义
static u8  evt_sensor_data;
static u8  evt_check_eachevt;																		
static u8  evt_wx_cmd=0;	

void Period_Events_Handle(u32 Events)
{
	if(Events&SYS_INIT_EVT)
	{
		RXENABLE2;
//		ROLLER1_STOP;
//   	ROLLER2_STOP;

		//有线通信灯亮
		YX_LED_ON;//需要看原理图？

    slave_init_readflash(); 
		startadc();
    if(RCC_GetFlagStatus(RCC_FLAG_PORRST)!=RESET) 
		 {
			ROLLER1_STOP;
   	  ROLLER2_STOP; 
		  memset(ControlValue,0x00,sizeof(ControlValue));
			memset(collector_data_buff,0x00,sizeof(collector_data_buff));			
		 }
		RCC_ClearFlag();	 		
    if(close_433MHZ!=0)
		{
		 SI4463_Init();
		 Start_timerEx(WX_CMD_EVT,2000);
		}
		Start_timerEx(SENSOR_DATA_EVT,2000);		  		
		Start_timerEx(IO_CTRL_CMD_EVT,1000);
		Start_timerEx(CHECK_EACHEVT_EVT,10000);	
	}

  YX_LED_TOGGLE;	
	if(Events&IO_CTRL_CMD_EVT)
	{
		IO_ctrl_cmd();
		Start_timerEx( IO_CTRL_CMD_EVT,100 );
	}

	if(Events&SENSOR_DATA_EVT)
	{
		evt_sensor_data=0;
		sensor_data();
		Start_timerEx( SENSOR_DATA_EVT, MEASURE_PERIOD );
		if(evt_check_eachevt<=250){evt_check_eachevt++;} else{ __set_FAULTMASK(1);NVIC_SystemReset();while(1);}
	}
		if(Events&RX2_DELAY_EVT)
	{
		
		RxFlag2 = 1;		
		RxReport2(RxReport2_len,ReceiveData2);
	}

	if(Events&RX2_TIMEOUT_EVT)
	{
		RecDataBuffer2[0] = RecLen2-1;
		RecLen2 = 1;
		RxReport2_len=RecDataBuffer2[0];
		memcpy(ReceiveData2,RecDataBuffer2+1,RecDataBuffer2[0]);	
		Start_timerEx(RX2_DELAY_EVT,3);
	}
	
	if(Events&WX_CMD_EVT)//设置SI4463处于接收状态，等待接收中断
	{
		evt_wx_cmd=0;
		Clear_Buf(SI4463_RxBUFF,SI4463_RxLenth,0);    		
		SI4463_SET_PROPERTY_1( PKT_FIELD_1_LENGTH_12_8, 0x00 );
		SI4463_SET_PROPERTY_1( PKT_FIELD_1_LENGTH_7_0, 0x01 );
		SI4463_START_RX( SI4463_Channel, 0, PACKET_LENGTH, 8, 3, 3 );   
	}
	
	if(Events&WX_RECEIVE_EVT)
	{
		WirelessLedToggle;
		SI4463_RxLenth = SI4463_READ_PACKET(SI4463_RxBUFF);		
		SI4463RxReport(SI4463_RxLenth,SI4463_RxBUFF);
		Start_timerEx(WX_CMD_EVT,50); 
	 }
  if(Events&CHECK_EACHEVT_EVT)
	{
	 evt_check_eachevt=0;
	 if(evt_sensor_data<=50){evt_sensor_data++;} else{ __set_FAULTMASK(1);NVIC_SystemReset();while(1);}
	 if(evt_wx_cmd<=180){evt_wx_cmd++;} 
   else
   {
	  close_433MHZ=1;
    evt_wx_cmd=0;		 
	  slave_init_readflash();
	  SI4463_Init();
	  Start_timerEx(WX_CMD_EVT,2000);}
	 Start_timerEx(CHECK_EACHEVT_EVT,10000);		
	}	
}

void Scan_Events_Handle(void)
{
	IWDG_ReloadCounter();					//独立看门狗喂狗
}



static void WriteDataToBuffer(u8 port,u8 *ptr,u8 start,u8 len) 
{ 
	//USART_SendData函数中，u32 i=0x004FFFFF;//防止全双工硬件通信时，硬件CTS不起作用造成等待状态反复进行watchdog(3S)不停起作用;2S左右，以看门狗不起作用为准
	u8 i;
	if(port==2)
	{
	    for(i=0;i<len;i++) 
	        USART2SendTCB[i] = ptr[start+i]; 
		USART2BufferCNT=len; 
		TXENABLE2;
		TxFlag2=1;
		USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
	}

	else if(port==1)
	{
	    for(i=0;i<len;i++) 
	        USART1SendTCB[i] = ptr[start+i]; 
		USART1BufferCNT=len; 
		TxFlag1=0;
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
	}

	else
		return; 
} 


static u16 GetCRC16(u8 *Msg, u16 Len)
{
	u8 CRCHigh = 0xFF;//高CRC字节初始化
	u8 CRCLow  = 0xFF;//低CRC字节初始化
	u16 index;		  //CRC循环中的索引
	u8 i=0;

	for(i=0;i<Len;i++)
	{
		index   = CRCHigh^*Msg++;
		CRCHigh = CRCLow^crc_hi[index];
		CRCLow  = crc_lo[index];
	}

	return(CRCLow<<8|CRCHigh);
}

/*处理有线收到的数据*/
static void RxReport2(u8 len,u8 *pBuf) 
{
	if(pBuf[0]==slave_ctrl_ID)
	{
		
	  if(GetCRC16(pBuf,len)==0)
		{  
			u16 address,CRCReport2;
							
//			RS485LedToggle;										//有线通信灯翻转
			if(pBuf[1]==0x03)//采集的电流及最大电流返回
			{	        
						address = (pBuf[2]<<8) | pBuf[3];
						regLen2  = (pBuf[4]<<8) | pBuf[5];	        
						ReportData2[0] = slave_ctrl_ID;	                //站号
						ReportData2[1] = 0x03;	                        //功能码
						ReportData2[2] = regLen2*2;	                    //字节数
						memcpy(ReportData2+3,collector_data_buff+address*2,ReportData2[2]);		
						CRCReport2=GetCRC16(ReportData2,3+regLen2*2);							
						ReportData2[3+regLen2*2] = CRCReport2&0x00FF;      //CRC低位
						ReportData2[4+regLen2*2] = (CRCReport2&0xFF00)>>8; //CRC高位				    
						WriteDataToBuffer(2,ReportData2,0,5+regLen2*2);	
            return; 				
			}
			if(pBuf[1]==0x06)
			{				
				if(pBuf[2]==0x00&&pBuf[3]<=0x03) //pBuf[6]==0x08 ；接收控制命令      
				{	
					ControlValue[pBuf[3]]=(pBuf[4]|(((u16)pBuf[5])<<8));		//取出命令指令	          					
			   }
				memcpy(ReportData2,pBuf,8);//返回命令=下发命令，8个字节完全相同			  	    
		    WriteDataToBuffer(2,ReportData2,0,8);
				return;  
			}
			if(pBuf[1]==0x10)
			{
			 memcpy(ReportData2,pBuf,6);//返回命令前6个字节完全相同，只需加CRC校验码
			 CRCReport2=GetCRC16(ReportData2,6);
			 ReportData2[6] = CRCReport2&0x00FF;      //CRC低位
	     ReportData2[7] = (CRCReport2&0xFF00)>>8; //CRC高位			    
		   WriteDataToBuffer(2,ReportData2,0,8);
		   if(pBuf[3]>=0x04&&pBuf[3]<0x10)//控制器控制要求设定：最大限流、起始运行时间、持续运行时间,12个变量，24个字节
		   {
			   memcpy(ctrlslave_param_set,pBuf+7,pBuf[6]);//控制器控制要求设定：最大限流、起始运行时间、持续运行时间
			   Flash_Write(0x0801C400, ctrlslave_param_set,24) ; //RBT6的FLASH为128k，ram为20k，此处将控制设定参数保存在flash最后的16k
				 slave_init_readflash();
				 return; 
		    }
		 
		   if(pBuf[3]>=0x10&&pBuf[3]<0x22)//控制器电流采集设定参数保存在flash最后的16k
		   {
			  memcpy(cjqslave_param_set,pBuf+7,pBuf[6]); //采集器或控制器的采集参数设定，以采集器或控制器为1组，每组18个参数（每个采集通道3个参数:采集方式，参数个数，滤波次数，6通道采集器）
			  Flash_Write(0x0801C000, cjqslave_param_set,36) ; //RBT6的FLASH为128k，ram为20k，此处将参数保存在flash最后的16k;采集方式，参数个数，滤波次数
				slave_init_readflash();
        return; 				 
		   }			    
		 }
	  }		 
	}
	if(pBuf[0]==0xF0&&pBuf[1]==0x10&&pBuf[3]==34)
	{
	 if(GetCRC16(pBuf,len)==0)
		{  
			u16 CRCReport2;
		 if(pBuf[7]!=65)//老信道=0xFF表示433MHZ通信信道要改变，pBuf[7]==65所有的子站都需要改变，不需要回复。
		 {
			memcpy(ReportData2,pBuf,6);//返回命令前6个字节完全相同，只需加CRC校验码
			CRCReport2=GetCRC16(ReportData2,6);
			ReportData2[6] = CRCReport2&0x00FF;      //CRC低位
			ReportData2[7] = (CRCReport2&0xFF00)>>8; //CRC高位			
			WriteDataToBuffer(2,ReportData2,0,8);
		 }
		 if(slaveID_radioID[0]==0xFF||slaveID_radioID[0]==0x00||(slaveID_radioID[0]==pBuf[7]&&pBuf[7]<=64&&pBuf[9]!=255))
     {
		  slaveID_radioID[0]=pBuf[8];//设定新站地址或修改老站地址
		  }
		 
		 if(pBuf[9]==255&&(pBuf[7]==65||slaveID_radioID[0]==pBuf[7]))
      {
			 slaveID_radioID[1]=pBuf[10];//slaveID_radioID[0]不修改，只改信道，不改子站地址
			}//pBuf[7]子站老地址；pBuf[8]子站新地址；pBuf[9]子站老信道；pBuf[10]子站新信道
		 Flash_Write(0x0801C800, slaveID_radioID,2) ; //RBT6的FLASH为128k，ram为20k，此处将参数保存在flash最后16k的第3个bank（第3k）;
		 slave_init_readflash(); 
		 if(close_433MHZ!=0&&pBuf[9]==255&&(pBuf[7]==65||slaveID_radioID[0]==pBuf[7]))//pBuf[7]==65为网关设定信道
			{
			 SI4463_Init();
			 Start_timerEx(WX_CMD_EVT,2000);
			}
		}
	 }
}		
	

//收到RBT6无线数据
static void SI4463RxReport(u8 len,u8 *pData)
{
//	memcpy(test,pData,30);
  if(pData[0]==slave_ctrl_ID)
	{
		if(GetCRC16(pData,len)==0)		
		{
			u16 address,SI4463_CRC;
					
			if(pData[1]==0x03)
			{			
						address = (pData[2]<<8) | pData[3];
						regLen  = (pData[4]<<8) | pData[5];	        
						SI4463_TxBUFF[0] = pData[0];	                //站号	     
						SI4463_TxBUFF[1] = 0x03;	                        //功能码
						SI4463_TxBUFF[2] = regLen*2;	                    //字节数
						memcpy(SI4463_TxBUFF+3,collector_data_buff+address*2,SI4463_TxBUFF[2]); 
						SI4463_CRC=GetCRC16(SI4463_TxBUFF,3+regLen*2);	        
						SI4463_TxBUFF[3+regLen*2] = SI4463_CRC&0x00FF;      //CRC低位
						SI4463_TxBUFF[4+regLen*2] = (SI4463_CRC&0xFF00)>>8; //CRC高位
						SI4463_SEND_PACKET(SI4463_TxBUFF,5+regLen*2,SI4463_Channel,0);
						return; 			
			 }
			if(pData[1]==0x06)
			{
				if(pData[2]==0x00&&pData[3]<=0x03) 
				{
				 ControlValue[pData[3]]=(pData[4]|(((u16)pData[5])<<8));		//取出命令指令				            						
				}
				memcpy(SI4463_TxBUFF,pData,8);//返回命令=下发命令，8个字节完全相同					
				SI4463_SEND_PACKET(SI4463_TxBUFF,8,SI4463_Channel,0);
				return;	
			}			
			if(pData[1]==0x10)
			{	  	
						memcpy(SI4463_TxBUFF,pData,6);//返回命令前6个字节完全相同，只需加CRC校验码
						SI4463_CRC=GetCRC16(SI4463_TxBUFF,6);
						SI4463_TxBUFF[6] = SI4463_CRC&0x00FF;      //CRC低位
						SI4463_TxBUFF[7] = (SI4463_CRC&0xFF00)>>8; //CRC高位
						SI4463_SEND_PACKET(SI4463_TxBUFF,8,SI4463_Channel,0);							
						if((pData[3]>=0x04)&(pData[3]<0x10)) //控制器设定以1个控制器为1组,每组12个参数（每个控制通道3个参数）
						{
						 memcpy( ctrlslave_param_set,pData+7,pData[6]);
						 Flash_Write(0x0801C400, ctrlslave_param_set,24) ; //RBT6的FLASH为128k，ram为20k，此处将参数保存在flash最后的16k
						 slave_init_readflash();
						 return;	          						
						}
						if((pData[3]>=0x10)&(pData[3]<0x22))  //采集器或控制器的采集参数设定，以采集器或控制器为1组，每组18个参数（每个采集通道3个参数，6通道采集器）；有线、无线共用
						{
						 memcpy( cjqslave_param_set,pData+7,pData[6]);
						 Flash_Write(0x0801C000, cjqslave_param_set,36) ; //RBT6的FLASH为128k，ram为20k，此处将参数保存在flash最后的16k
						 slave_init_readflash();
							return;	          						
						} 
				}
		}	
	}
	if(pData[0]==0xF0&&pData[1]==0x10&&pData[3]==34)
	{
	 if(GetCRC16(pData,len)==0)
		{  
			u16 CRCReport2;
		if(pData[7]!=65)//老信道=0xFF表示433MHZ通信信道要改变，pData[7]==65所有的子站都需要改变，不需要回复。
		 {
			memcpy(ReportData2,pData,6);//返回命令前6个字节完全相同，只需加CRC校验码
			CRCReport2=GetCRC16(ReportData2,6);
			ReportData2[6] = CRCReport2&0x00FF;      //CRC低位
			ReportData2[7] = (CRCReport2&0xFF00)>>8; //CRC高位
			WriteDataToBuffer(2,ReportData2,0,8);
		 }
		 if(slaveID_radioID[0]==0xFF||slaveID_radioID[0]==0x00||(slaveID_radioID[0]==pData[7]&&pData[7]<=64&&pData[9]!=255))
     {
		  slaveID_radioID[0]=pData[8];//设定新站地址或修改老站地址
		  }
		 if(pData[9]==255&&(pData[7]==65||slaveID_radioID[0]==pData[7]))
      {
			  slaveID_radioID[1]=pData[10];//slaveID_radioID[0]不修改，只改信道，不改子站地址
			}//pData[7]子站老地址；pData[8]子站新地址；pData[9]子站老信道；pData[10]子站新信道
		 Flash_Write(0x0801C800, slaveID_radioID,2) ; //RBT6的FLASH为128k，ram为20k，此处将参数保存在flash最后16k的第3个bank（第3k）;
		 slave_init_readflash(); 
		 if(close_433MHZ!=0&&pData[9]==255&&(pData[7]==65||slaveID_radioID[0]==pData[7]))//pData[7]==65为网关设定信道
			{
			 SI4463_Init();
			 Start_timerEx(WX_CMD_EVT,2000);
			}
		}
	 }
}


 static void Clear_Buf(u8 buf[], u8 length, u8 data)
{
	u8 i = 0;
	for(i = 0; i < length; i++)
		buf[i] = data;
	
}

static void startadc(void)
{
	DMA_Cmd(DMA1_Channel1,ENABLE);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);  //连续转换开始，ADC通过DMA方式不断的更新RAM区。 
}
//ADC_ConvertedValue[0]--PC0--IN10--I0--控制通道0,[1]--PB0--IN8--I2--控制通道1,[2]--PC1--IN11--I1--控制通道2,[3]--PB1--IN9--I3--控制通道3			
static void sensor_data(void)
{
	if(ReadDataCNT1==0)		//测电流0
	{
		u8   TD_number;
	  u16  temp_adc;
		
	 if(control_type[0]==0&&control_type[1]==0)//0为单向控制
	 {
    TD_number=0;
		temp_adc=Get_Adclvbo(TD_number,262,1305); //(4~20ma)*50Ω;0.2V对应262，1.0V对应1305   				
		     if(First_adc_average[TD_number]==0)
		      {  
				   temp_adc=First_Getaverage( TD_number, Maxlvbo_number, temp_adc);
				   First_adc_average[TD_number]=1;
		      }			
				  temp_adc=TD_Getaverage( TD_number,TDlvbo_number[TD_number], temp_adc,tdcycle_i[TD_number]);
			      First_adc_average[TD_number]=1;
			      tdcycle_i[TD_number]++;
			      if(tdcycle_i[TD_number]>=TDlvbo_number[TD_number])
			       {
			   	     tdcycle_i[TD_number]=0;
			       }		      
		MotorCurrent0 = temp_adc;
						 
		TD_number=1;
		temp_adc=Get_Adclvbo(TD_number,262,1305);   				
		     if(First_adc_average[TD_number]==0)
		      {  
				   temp_adc=First_Getaverage( TD_number, Maxlvbo_number, temp_adc);
				   First_adc_average[TD_number]=1;
		      }			
				  temp_adc=TD_Getaverage( TD_number,TDlvbo_number[TD_number], temp_adc,tdcycle_i[TD_number]);
			      First_adc_average[TD_number]=1;
			      tdcycle_i[TD_number]++;
			      if(tdcycle_i[TD_number]>=TDlvbo_number[TD_number])
			       {
			   	     tdcycle_i[TD_number]=0;
			       }		      
		MotorCurrent1 = temp_adc;				 
		if(MotorCurrent0> MotorMAX[0])
		{
			MotorMAX[0]=MotorCurrent0;
		}		
		if(MotorCurrent1> MotorMAX[1])
		{
			MotorMAX[1]=MotorCurrent1;
		}
		if((max_limit_current[0]<0xFF00)&&(max_limit_current[0]<MotorMAX[0]))ControlValue[0]=0;
		if((max_limit_current[1]<0xFF00)&&(max_limit_current[0]<MotorMAX[1]))ControlValue[1]=0;
    collector_data_buff[0]   =  MotorCurrent0&0x00ff;         	//0#电机当前电流低位
	  collector_data_buff[1]   =  (MotorCurrent0&0xff00)>>8;		//0#电机当前电流高位
	  collector_data_buff[2]   =  MotorMAX[0]&0x00ff;				     //0#电机最大电流低位
	  collector_data_buff[3]   =  (MotorMAX[0]&0xff00)>>8;			//0#电机最大电流高位
    collector_data_buff[4]   =  MotorCurrent1&0x00ff;         	//1#电机当前电流低位
	  collector_data_buff[5]   =  (MotorCurrent1&0xff00)>>8;		//1#电机当前电流高位
		collector_data_buff[6]   =  MotorMAX[1]&0x00ff;				     //1#电机最大电流低位
		collector_data_buff[7]   =  (MotorMAX[1]&0xff00)>>8;			//1#电机最大电流高位
		}
		else //0为正、反控制
		{
     TD_number=0;
		temp_adc=Get_Adclvbo(TD_number,0,1436); //1436=1305*1.1;0.2V对应262(0x0106)，1.0V对应1305(0x0519)  				
		     if(First_adc_average[TD_number]==0)
		      {  
				   temp_adc=First_Getaverage( TD_number, Maxlvbo_number, temp_adc);
				   First_adc_average[TD_number]=1;
		      }			
				  temp_adc=TD_Getaverage( TD_number,TDlvbo_number[TD_number], temp_adc,tdcycle_i[TD_number]);
			      First_adc_average[TD_number]=1;
			      tdcycle_i[TD_number]++;
			      if(tdcycle_i[TD_number]>=TDlvbo_number[TD_number])
			       {
			   	     tdcycle_i[TD_number]=0;
			       }		      
		 MotorCurrent0 = temp_adc;					
		 if(MotorCurrent0<100){MotorCurrent0=0;} //test时取消    
		 if(MotorCurrent0> MotorMAX[0]){ MotorMAX[0]=MotorCurrent0;}
		 MotorCurrent1=MotorCurrent0;
		 if(MotorCurrent1> MotorMAX[1]){ MotorMAX[1]=MotorCurrent1;}
		 if(roller1_state==0)
      {
			 collector_data_buff[0]   =  0;                              //0#电机当前正向电流低位
	     collector_data_buff[1]   =  0;		                         //0#电机当前正向电流高位
	     collector_data_buff[2]   =  0;				                    //0#电机最大正向电流低位
	     collector_data_buff[3]   =  0;			                     //0#电机最大正向电流高位
			 collector_data_buff[4]   =  0;         	                   //0#电机当前反向电流低位
			 collector_data_buff[5]   =  0;		                        //0#电机当前反向电流高位
			 collector_data_buff[6]   =  0;				                   //0#电机最反向大电流低位
			 collector_data_buff[7]   =  0;			                    //0#电机最大反向电流高位
			}
		 if(roller1_state==1)
		  {
			 if((max_limit_current[0]<0xFF00)&&(max_limit_current[0]<MotorMAX[0]))
			 {
				 ControlValue[0]=8;
		     ControlValue[1]=8;
			 }
			 collector_data_buff[0]   =  MotorCurrent0&0x00ff;         //0#电机当前正向电流低位
			 collector_data_buff[1]   =  (MotorCurrent0&0xff00)>>8;		//0#电机当前正向电流高位
			 collector_data_buff[2]   =  MotorMAX[0]&0x00ff;				   //0#电机最大正向电流低位
			 collector_data_buff[3]   =  (MotorMAX[0]&0xff00)>>8;			//0#电机最大正向电流高位
			 collector_data_buff[4]   =  0;         	                   //0#电机当前反向电流低位
			 collector_data_buff[5]   =  0;		                        //0#电机当前反向电流高位
			 collector_data_buff[6]   =  0;				                   //0#电机最反向大电流低位
			 collector_data_buff[7]   =  0;			                    //0#电机最大反向电流高位
			}
		 if(roller1_state==2)
		 {
			 if((max_limit_current[1]<0xFF00)&&(max_limit_current[1]<MotorMAX[1]))
			 {
				 ControlValue[0]=8;
				 ControlValue[1]=8;
			 }
		  collector_data_buff[0]   =  0;                             //0#电机当前正向电流低位
	    collector_data_buff[1]   =  0;		                        //0#电机当前正向电流高位
	    collector_data_buff[2]   =  0;				                   //0#电机最大正向电流低位
	    collector_data_buff[3]   =  0;			                    //0#电机最大正向电流高位
      collector_data_buff[4]   =  MotorCurrent1&0x00ff;         	//0#电机当前反向电流低位
	    collector_data_buff[5]   =  (MotorCurrent1&0xff00)>>8;		//0#电机当前反向电流高位
		  collector_data_buff[6]   =  MotorMAX[1]&0x00ff;				     //0#电机最大反向电流低位
		  collector_data_buff[7]   =  (MotorMAX[1]&0xff00)>>8;			//0#电机最大反向电流高位
		 }
		}	   		
	}
//ADC_ConvertedValue[0]--PC0--IN10--I0--控制通道0,[1]--PB0--IN8--I2--控制通道1,[2]--PC1--IN11--I1--控制通道2,[3]--PB1--IN9--I3--控制通道3			
	if(ReadDataCNT1==1)		//测电流1
	{
		u8   TD_number;
	  u16  temp_adc;		
		
	 if(control_type[2]==0&&control_type[3]==0)//0为单向控制
	 {
		 TD_number=2;
		temp_adc=Get_Adclvbo(TD_number,262,1305); //(4~20ma)*50Ω;0.2V对应262，1.0V对应1305   				
		     if(First_adc_average[TD_number]==0)
		      {  
				   temp_adc=First_Getaverage( TD_number, Maxlvbo_number, temp_adc);
				   First_adc_average[TD_number]=1;
		      }			
				  temp_adc=TD_Getaverage( TD_number,TDlvbo_number[TD_number], temp_adc,tdcycle_i[TD_number]);
			      First_adc_average[TD_number]=1;
			      tdcycle_i[TD_number]++;
			      if(tdcycle_i[TD_number]>=TDlvbo_number[TD_number])
			       {
			   	     tdcycle_i[TD_number]=0;
			       }		      
		MotorCurrent2 = temp_adc;
						 
		TD_number=3;
		temp_adc=Get_Adclvbo(TD_number,262,1305);   				
		     if(First_adc_average[TD_number]==0)
		      {  
				   temp_adc=First_Getaverage( TD_number, Maxlvbo_number, temp_adc);
				   First_adc_average[TD_number]=1;
		      }			
				  temp_adc=TD_Getaverage( TD_number,TDlvbo_number[TD_number], temp_adc,tdcycle_i[TD_number]);
			      First_adc_average[TD_number]=1;
			      tdcycle_i[TD_number]++;
			      if(tdcycle_i[TD_number]>=TDlvbo_number[TD_number])
			       {
			   	     tdcycle_i[TD_number]=0;
			       }		      
		MotorCurrent3 = temp_adc;//(4~20ma)*50Ω;0.2V对应262，1.0V对应1305				 
		
		if(MotorCurrent2> MotorMAX[2])
		{
			MotorMAX[2]=MotorCurrent2;
		}		
		if(MotorCurrent3> MotorMAX[3])
		{
			MotorMAX[3]=MotorCurrent3;
		}
		if((max_limit_current[2]<0xFF00)&&(max_limit_current[2]<MotorMAX[2]))ControlValue[2]=0;
		if((max_limit_current[3]<0xFF00)&&(max_limit_current[3]<MotorMAX[3]))ControlValue[3]=0;
    collector_data_buff[8]   =  MotorCurrent2&0x00ff;         	//2#电机当前电流低位
	  collector_data_buff[9]   =  (MotorCurrent2&0xff00)>>8;		 //2#电机当前电流高位
	  collector_data_buff[10]   =  MotorMAX[2]&0x00ff;				    //2#电机最大电流低位
	  collector_data_buff[11]   =  (MotorMAX[2]&0xff00)>>8;			 //2#电机最大电流高位
    collector_data_buff[12]   =  MotorCurrent3&0x00ff;         	//3#电机当前电流低位
	  collector_data_buff[13]   =  (MotorCurrent3&0xff00)>>8;		 //3#电机当前电流高位
		collector_data_buff[14]   =  MotorMAX[3]&0x00ff;				    //3#电机最大电流低位
		collector_data_buff[15]   =  (MotorMAX[3]&0xff00)>>8;			 //3#电机最大电流高位
		}
		else //0为正、反控制
		{
		 TD_number=2;
		temp_adc=Get_Adclvbo(TD_number,0,1436); //1436=1305*1.1;0.2V对应262(0x0106)，1.0V对应1305(0x0519)  				
		     if(First_adc_average[TD_number]==0)
		      {  
				   temp_adc=First_Getaverage( TD_number, Maxlvbo_number, temp_adc);
				   First_adc_average[TD_number]=1;
		      }			
				  temp_adc=TD_Getaverage( TD_number,TDlvbo_number[TD_number], temp_adc,tdcycle_i[TD_number]);
			      First_adc_average[TD_number]=1;
			      tdcycle_i[TD_number]++;
			      if(tdcycle_i[TD_number]>=TDlvbo_number[TD_number])
			       {
			   	     tdcycle_i[TD_number]=0;
			       }		      
		 MotorCurrent2 = temp_adc;//1436=1305*1.1;0.2V对应262(0x0106)，1.0V对应1305(0x0519)					
		 if(MotorCurrent2<100){MotorCurrent2=0;} //test时取消    
		 if(MotorCurrent2> MotorMAX[2]){ MotorMAX[2]=MotorCurrent2;}
		 MotorCurrent3=MotorCurrent2;
		 if(MotorCurrent3> MotorMAX[3]){ MotorMAX[3]=MotorCurrent3;}
		 if(roller2_state==0)
      {
			 collector_data_buff[8]    =  0;                             //2#电机当前正向电流低位
	     collector_data_buff[9]    =  0;		                         //2#电机当前正向电流高位
	     collector_data_buff[10]   =  0;				                    //2#电机最大正向电流低位
	     collector_data_buff[11]   =  0;			                     //2#电机最大正向电流高位
			 collector_data_buff[12]   =  0;         	                 //2#电机当前反向电流低位
			 collector_data_buff[13]   =  0;		                        //2#电机当前反向电流高位
			 collector_data_buff[14]   =  0;				                   //2#电机最反向大电流低位
			 collector_data_buff[15]   =  0;			                    //2#电机最大反向电流高位
			}
		 if(roller2_state==1)
		  {
			 if((max_limit_current[2]<0xFF00)&&(max_limit_current[2]<MotorMAX[2]))
			 {
				 ControlValue[2]=8;
		     ControlValue[3]=8;
			 }
			 collector_data_buff[8]   =  MotorCurrent2&0x00ff;           //2#电机当前正向电流低位
			 collector_data_buff[9]   =  (MotorCurrent2&0xff00)>>8;		  //2#电机当前正向电流高位
			 collector_data_buff[10]   =  MotorMAX[2]&0x00ff;				     //2#电机最大正向电流低位
			 collector_data_buff[11]   =  (MotorMAX[2]&0xff00)>>8;			//2#电机最大正向电流高位
			 collector_data_buff[12]   =  0;         	                 //2#电机当前反向电流低位
			 collector_data_buff[13]   =  0;		                        //2#电机当前反向电流高位
			 collector_data_buff[14]   =  0;				                   //2#电机最反向大电流低位
			 collector_data_buff[15]   =  0;			                    //2#电机最大反向电流高位
			}
		 if(roller2_state==2)
		 {
			if((max_limit_current[3]<0xFF00)&&(max_limit_current[3]<MotorMAX[3]))
			 {
				 ControlValue[2]=8;
		     ControlValue[3]=8;
			 }
		  collector_data_buff[8]   =   0;                             //2#电机当前正向电流低位
	    collector_data_buff[9]   =   0;		                         //2#电机当前正向电流高位
	    collector_data_buff[10]   =  0;				                    //2#电机最大正向电流低位
	    collector_data_buff[11]   =  0;			                     //2#电机最大正向电流高位
      collector_data_buff[12]   =  MotorCurrent3&0x00ff;          	//2#电机当前反向电流低位
	    collector_data_buff[13]   =  (MotorCurrent3&0xff00)>>8;		   //2#电机当前反向电流高位
		  collector_data_buff[14]   =  MotorMAX[3]&0x00ff;				      //2#电机最大反向电流低位
		  collector_data_buff[15]   =  (MotorMAX[3]&0xff00)>>8;			   //2#电机最大反向电流高位
		 }
		}	   		
	}
	if(ReadDataCNT1>=1)
	{
		ReadDataCNT1=0;
	}
	else 
	{  
		ReadDataCNT1++;
	}	
}

static void slave_init_readflash(void)//子站初始化读flash
{
  u8 init_flash_flg[2]={0,0},i;
	
	 Flash_Read(0x0801C000,  init_flash_flg, 2);//主站设定子站采集参数需要写入flash0x0801 C000，从112k开始写入，最后16k为参数设定,每次写入必需1Kbyte
	 if(init_flash_flg[0]!=0xFF||init_flash_flg[1]!=0xFF) //如果已经经过出厂配置，读出厂子站采集参数设定。否则使用程序初值
	 {
		 Flash_Read(0x0801C000,  cjqslave_param_set, 36);		 
	 }
	 
	 Flash_Read(0x0801C400,  init_flash_flg, 2);//主站设定子站控制参数需要写入flash0x0801 C400，从112k开始写入，最后16k为参数设定,每次写入必需1Kbyte
	 if(init_flash_flg[0]!=0xFF||init_flash_flg[1]!=0xFF) //如果已经经过出厂配置，读出厂子站控制参数设定。否则使用程序初值
	 {
		 Flash_Read(0x0801C400,  ctrlslave_param_set, 24);		 
	 }
	 for(i=0;i<24;i=i+6)
	 {
	  max_limit_current[i/6]=ctrlslave_param_set[i]|(ctrlslave_param_set[i+1]<<8);//max_limit_current[i]>0xFFF0 不限流；max_limit_current[i]/65535*量程		
		start_run_time[i/6]=ctrlslave_param_set[i+2]|(ctrlslave_param_set[i+3]<<8);//start_run_time[i]=0x0000 立即启动；单位：秒；65534/3600=18.2小时
		start_run_nowtime[i/6]=start_run_time[i/6]*10;
		continue_run_time[i/6]=ctrlslave_param_set[i+4]|(ctrlslave_param_set[i+5]<<8);//continue_run_time[i]=0xFFFF,连续运行，不限制；单位：秒；65534/3600=18.2小时
		continue_run_nowtime[i/6]= continue_run_time[i/6]*10;
	 }
   Flash_Read(0x0801C800,  slaveID_radioID, 2);//读取站地址及433MHZ信道
   	slave_ctrl_ID=slaveID_radioID[0];
    if(slaveID_radioID[1]==0xFF){SI4463_Channel=0;}
    else{SI4463_Channel=slaveID_radioID[1];}		
}


static void IO_ctrl_cmd(void)
{
  u8 i;
	for(i=0;i<=3;i++)
	{
		switch (ControlValue[i])//ControlValue[0]为0号通道命令值，以此类推;控制器控制要求设定：最大限流、起始运行时间、持续运行时间
		{
		 case 0x0000://开关型直接控制命令：关
			 control_type[i]=0;//单向控制
		 if(single_delay_flg[i]==0)
		 {	
			if(i==0){ CTRL_CLOSE0;}			
			if(i==1){ CTRL_CLOSE1;}			
			if(i==2){ CTRL_CLOSE2;}			
			if(i==3){ CTRL_CLOSE3;}			
			if(single_old_ControlValue[i]!=0)
			{
				single_delay_time[i]=30;//100ms循环一次，30次循环就是3S
				single_old_ControlValue[i]=0;
			}			
			single_delay_flg[i]=1;			
		 }
		 else if(single_delay_time[i]>=1){single_delay_time[i]--;}//长时间（3秒及以上）电机停止运行，可以立即启动电机
		break;
			case 0x0001://开关型直接控制命令：开
				control_type[i]=0;//单向控制
		 if(single_delay_time[i]==0)
		 {
			if(start_run_nowtime[i]>=1){ start_run_nowtime[i]--;}		
      if(start_run_nowtime[i]==0)//开采用延时启动参数和连续运行时间参数；关不延时
      {
			  if(i==0){CTRL_OPEN0;}
				if(i==1){CTRL_OPEN1;}
			  if(i==2){CTRL_OPEN2;}
			  if(i==3){CTRL_OPEN3;}				
				if(continue_run_nowtime[i]>=1&&continue_run_nowtime[i]<=0xFF00*10){continue_run_nowtime[i]--;}
				if(continue_run_nowtime[i]==0)
				{
				 ControlValue[i]=0x0000;
				 continue_run_nowtime[i]=continue_run_time[i]*10;
				 start_run_nowtime[i]=start_run_time[i]*10;					 
				}				 				
			 }			
      if(single_old_ControlValue[i]!=1){single_old_ControlValue[i]=1;MotorMAX[i]=0;First_adc_average[i]=0;}	
			single_delay_flg[i]=0;			
		  }			
			if(single_delay_time[i]>=1){single_delay_time[i]--;}			
		 break;
			
			case 0x0002://正反转批量控制命令反转
				control_type[i]=1;//正、反向控制,0表示单向控制，1表示正反控制，由收到的命令设定，供电流检测事件使用，决定采用哪种方式上报电流大小
			if(i==0&&bidirection_location_flg[i]!=2)//bidirection_location_flg[i]=2表示反转已经完全到位，不允许反转了。
			{	
			 if(batch_ctrl_finish[0]==0)
				{
					if(start_run_nowtime[i+1]==0)//反转使用通道1的参数作为反转控制参数;batch_ctrl_finish[0]=0，1#正转完成；;batch_ctrl_finish[1]=0，1#反转完成
					{
						ROLLER1_DOWN;						
						batch_ctrl_finish[1]=1;
						bidirection_location_flg[i]=0;
						if(roller1_state!=2){MotorMAX[1]=0;First_adc_average[1]=0;}
						roller1_state=2;//0表示停；1表示上行；2表示下行	
						if(continue_run_nowtime[i+1]>=1&&continue_run_nowtime[i+1]<=0xFF00*10){continue_run_nowtime[i+1]--;}						
						if(continue_run_nowtime[i+1]==0)
						{
						 batch_ctrl_finish[1]=0;
						 bidirection_location_flg[i]=2;
						 continue_run_nowtime[i+1]=continue_run_time[i+1]*10;
						 start_run_nowtime[i+1]=start_run_time[i+1]*10;
						 ROLLER1_STOP;
						 roller1_state=0;//0表示停；1表示上行；2表示下行
						 }					
					 }
					else if(start_run_nowtime[i+1]>=1){ start_run_nowtime[i+1]--;} 
			  }
       else
			  {
				 batch_ctrl_finish[0]=0;
				 bidirection_location_flg[i]=1;
				 continue_run_nowtime[i]=continue_run_time[i]*10;
				 start_run_nowtime[i]=start_run_time[i]*10;
				 ROLLER1_STOP;
				 roller1_state=0;//0表示停；1表示上行；2表示下行
				}				 
			}
			
		if(i==2&&bidirection_location_flg[i]!=2)//bidirection_location_flg[i]=2表示反转已经完全到位，不允许反转了。batch_ctrl_finish[2]=0，2#正转完成；;batch_ctrl_finish[3]=0，2#反转完成
		{	
			  if(batch_ctrl_finish[2]==0)
				{
					if(start_run_nowtime[i+1]==0)//反转使用通道3的参数作为反转控制参数
					{
						ROLLER2_DOWN;						
						batch_ctrl_finish[3]=1;
						bidirection_location_flg[i]=0;
						if(roller2_state!=2){MotorMAX[3]=0;First_adc_average[3]=0;}
						roller2_state=2;//0表示停；1表示上行；2表示下行					
						if(continue_run_nowtime[i+1]>=1&&continue_run_nowtime[i+1]<=0xFF00*10){continue_run_nowtime[i+1]--;}	
						if(continue_run_nowtime[i+1]==0)
						{
						 batch_ctrl_finish[3]=0;
						 bidirection_location_flg[i]=2;
						 continue_run_nowtime[i+1]=continue_run_time[i+1]*10;
						 start_run_nowtime[i+1]=start_run_time[i+1]*10;
						 ROLLER2_STOP;
						 roller2_state=0;//0表示停；1表示上行；2表示下行,在电流检测中判断上行或下行的检测电流及保护
						}						
					}
				else if(start_run_nowtime[i+1]>=1){ start_run_nowtime[i+1]--;} 
				}
    		else
			  {	
				 batch_ctrl_finish[2]=0;
				 bidirection_location_flg[i]=1;
				 continue_run_nowtime[i]=continue_run_time[i]*10;
				 start_run_nowtime[i]=start_run_time[i]*10;
				 ROLLER2_STOP;
				 roller2_state=0;//0表示停；1表示上行；2表示下行
				}	
		 }
			
			i++;//每二个通道控制1个正反转电机
				break;
			
			case 0x0003://正反转批量控制命令正转
				control_type[i]=1;//正、反向控制
			if(i==0&&bidirection_location_flg[i]!=1)//bidirection_location_flg[i]=1表示正转已经完全到位，不允许正转了。
			{	
				if(batch_ctrl_finish[1]==0)
				{
					if(start_run_nowtime[i]==0)//正转使用通道0的参数作为正转控制参数
					{
						ROLLER1_UP;						
						batch_ctrl_finish[0]=1;
						bidirection_location_flg[i]=0;
						if(roller1_state!=1){MotorMAX[0]=0;First_adc_average[0]=0;}					
						roller1_state=1;//0表示停；1表示上行；2表示下行					
						if(continue_run_nowtime[i]>=1&&continue_run_nowtime[i]<=0xFF00*10){continue_run_nowtime[i]--;}		
						if(continue_run_nowtime[i]==0)
						{
						 batch_ctrl_finish[0]=0;
						 bidirection_location_flg[i]=1;
						 continue_run_nowtime[i]=continue_run_time[i]*10;
						 start_run_nowtime[i]=start_run_time[i]*10;
						 ROLLER1_STOP;
						 roller1_state=0;//0表示停；1表示上行；2表示下行
						}						
					 }
					else if(start_run_nowtime[i]>=1){ start_run_nowtime[i]--;} 
         }
        else
				 {
					 batch_ctrl_finish[1]=0;
					 bidirection_location_flg[i]=2;
					 continue_run_nowtime[i+1]=continue_run_time[i+1]*10;
					 start_run_nowtime[i+1]=start_run_time[i+1]*10;
					 ROLLER1_STOP;
					 roller1_state=0;//0表示停；1表示上行；2表示下行 
				 }					
			}
			
			if(i==2&&bidirection_location_flg[i]!=1)//bidirection_location_flg[i]=1表示正转已经完全到位，不允许正转了。
			{	
				if(batch_ctrl_finish[3]==0)
				{
					if(start_run_nowtime[i]==0)//正转使用通道2的参数作为正转控制参数
					{
						ROLLER2_UP;						
						batch_ctrl_finish[2]=1;
						bidirection_location_flg[i]=0;
						if(roller2_state!=1){MotorMAX[2]=0;First_adc_average[2]=0;}
						roller2_state=1;//0表示停；1表示上行；2表示下行					
						if(continue_run_nowtime[i]>=1&&continue_run_nowtime[i]<=0xFF00*10){continue_run_nowtime[i]--;}	
						if(continue_run_nowtime[i]==0)
						{
						 batch_ctrl_finish[2]=0;
						 bidirection_location_flg[i]=1;
						 continue_run_nowtime[i]=continue_run_time[i]*10;
						 start_run_nowtime[i]=start_run_time[i]*10;
						 ROLLER2_STOP;
						 roller2_state=0;//0表示停；1表示上行；2表示下行
						}						
					}
					else if(start_run_nowtime[i]>=1){ start_run_nowtime[i]--;}
         }
        else
         {
				   batch_ctrl_finish[3]=0;
					 bidirection_location_flg[i]=2;
					 continue_run_nowtime[i+1]=continue_run_time[i+1]*10;
					 start_run_nowtime[i+1]=start_run_time[i+1]*10;
					 ROLLER2_STOP;
					 roller2_state=0;//0表示停；1表示上行；2表示下行,在电流检测中判断上行或下行的检测电流及保护
				 }					
			}
			
			i++;	
				break;
			case 0x0004://开关型批量控制命令：关
      control_type[i]=0;//单向控制				
      if(i==0){CTRL_CLOSE0;}
			if(i==1){CTRL_CLOSE1;}
			if(i==2){CTRL_CLOSE2;}
			if(i==3){CTRL_CLOSE3;}				
				break;
			case 0x0005://开关型批量控制命令：开
				control_type[i]=0;//单向控制
			if(start_run_nowtime[i]>=1){ start_run_nowtime[i]--;MotorMAX[i]=0;First_adc_average[i]=0;}		
      if(start_run_nowtime[i]==0)//开采用延时启动参数和连续运行时间参数；关不延时
      {
			  if(i==0){CTRL_OPEN0;}
				if(i==1){CTRL_OPEN1;}
			  if(i==2){CTRL_OPEN2;}
			  if(i==3){CTRL_OPEN3;}				
				if(continue_run_nowtime[i]>=1&&continue_run_nowtime[i]<=0xFF00*10){continue_run_nowtime[i]--;}
				if(continue_run_nowtime[i]==0)
				{
				 ControlValue[i]=0x0004;
				 continue_run_nowtime[i]=continue_run_time[i]*10;
				 start_run_nowtime[i]=start_run_time[i]*10;					 
				}				 				
			 }							
				break;
			case 0x0006://暂时未定义				
				break;
			case 0x0007://暂时未定义				
				break;
			case 0x0008://正反转直接控制命令：停
				control_type[i]=1;//正、反向控制
       if(bidirection_delay_flg[i]==0)
			 {				 
				if(i==0){ ROLLER1_STOP;roller1_state=0;}//0表示停；1表示上行；2表示下行				
				if(i==2){ ROLLER2_STOP;roller2_state=0;}//0表示停；1表示上行；2表示下行		
        
				if(ControlValue[i]!=bidirection_old_ControlValue[i])
				{
					bidirection_delay_time[i]=30;
					bidirection_old_ControlValue[i]=ControlValue[i];
				 }				
					bidirection_delay_flg[i]=1;
					bidirection_run_flg[i]=0;
					bidirection_run_time[i]=0;				
			  }
			 else if(bidirection_delay_time[i]>=1){bidirection_delay_time[i]--;}//长时间（3秒及以上）电机停止运行，可以立即启动电机
			 i++;
				break;
			case 0x0009://正反转直接控制命令：正转
				control_type[i]=1;//正、反向控制
			 if(bidirection_old_ControlValue[i]==11)//从反转直接转向正转，需要延时5S
			 {
				if(i==0){ ROLLER1_STOP;roller1_state=0;}//0表示停；1表示上行；2表示下行				
				if(i==2){ ROLLER2_STOP;roller2_state=0;}//0表示停；1表示上行；2表示下行		
			  bidirection_delay_time[i]=50;
				bidirection_old_ControlValue[i]=9; 
			 }
			 if(bidirection_delay_time[i]==0&&(bidirection_run_time[i]==0||bidirection_run_flg[i]==0))
			 {
				if(i==0&&bidirection_location_flg[i]!=1)//bidirection_location_flg[i]表示位置信号，只能用i=0或2；不用i=1或3；电机运行状态及位置，0或8表示停，1表示正转停的位置，2表示反转停的位置
        { 
				  ROLLER1_UP;					
					if(bidirection_old_ControlValue[i]!=9){bidirection_old_ControlValue[i]=9;}		
					if(roller1_state!=1){MotorMAX[0]=0;First_adc_average[0]=0;}
					roller1_state=1;//0表示停；1表示上行；2表示下行
          					
					if(continue_run_nowtime[i]>=1&&continue_run_nowtime[i]<=0xFF00*10){continue_run_nowtime[i]--;}
					if(continue_run_nowtime[i]==0)
          {
					 ROLLER1_STOP;
					 roller1_state=0;//0表示停；1表示上行；2表示下行		
					 bidirection_location_flg[i]=1;
					 continue_run_nowtime[i]=continue_run_time[i]*10;					 
				  }
				 }
				
				if(i==2&&bidirection_location_flg[i]!=1)
        { 
				  ROLLER2_UP;					
					if(bidirection_old_ControlValue[i]!=9){bidirection_old_ControlValue[i]=9;}	
          if(roller2_state!=1){MotorMAX[2]=0;First_adc_average[2]=0;}					
					roller2_state=1;//0表示停；1表示上行；2表示下行	
          			
					if(continue_run_nowtime[i]>=1&&continue_run_nowtime[i]<=0xFF00*10){continue_run_nowtime[i]--;}
					if(continue_run_nowtime[i]==0)
          {
					 ROLLER2_STOP;
					 roller2_state=0;//0表示停；1表示上行；2表示下行		
					 bidirection_location_flg[i]=1;
					 continue_run_nowtime[i]=continue_run_time[i]*10;					 
				  }
				 }					
				
				bidirection_delay_flg[i]=0;
				bidirection_run_flg[i]=2;
				bidirection_run_time[i]=30;				
			 }
				if(bidirection_delay_time[i]>=1){bidirection_delay_time[i]--;}
				if(bidirection_run_flg[i]!=2&&bidirection_run_time[i]>=1){ bidirection_run_time[i]--; }				
			 
				i++;		
				break;
			case 0x000B://正反转直接控制命令：反转
				control_type[i]=1;//正、反向控制
				if(bidirection_old_ControlValue[i]==9)//从正转直接转向反转，需要延时5S
			 {
				if(i==0){ ROLLER1_STOP;roller1_state=0;}//0表示停；1表示上行；2表示下行				
				if(i==2){ ROLLER2_STOP;roller2_state=0;}//0表示停；1表示上行；2表示下行		
			  bidirection_delay_time[i]=50;
				bidirection_old_ControlValue[i]=11; 
			 }
			 if(bidirection_delay_time[i]==0&&(bidirection_run_time[i]==0||bidirection_run_flg[i]==0))
			 {
				 
				if(i==0&&bidirection_location_flg[i]!=2)
        { 
				  ROLLER1_DOWN;					
					if(bidirection_old_ControlValue[i]!=11){bidirection_old_ControlValue[i]=11;}	
					if(roller1_state!=2){MotorMAX[1]=0;First_adc_average[1]=0;}
					roller1_state=2;//0表示停；1表示上行；2表示下行
          					
					if(continue_run_nowtime[i+1]>=1&&continue_run_nowtime[i+1]<=0xFF00*10){continue_run_nowtime[i+1]--;}
					if(continue_run_nowtime[i+1]==0)
          {
					 ROLLER1_STOP;
					 roller1_state=0;//0表示停；1表示上行；2表示下行		
					 bidirection_location_flg[i]=2;
					 continue_run_nowtime[i+1]=continue_run_time[i+1]*10;					 
				  }
				 }
				
				if(i==2&&bidirection_location_flg[i]!=2)//bidirection_location_flg[i]表示位置信号，只能用i=0或2；不用i=0或2
        { 
				  ROLLER2_DOWN;					
					if(bidirection_old_ControlValue[i]!=11){bidirection_old_ControlValue[i]=11;}	
					if(roller2_state!=2){MotorMAX[3]=0;First_adc_average[3]=0;}
					roller2_state=2;//0表示停；1表示上行；2表示下行
          			
					if(continue_run_nowtime[i+1]>=1&&continue_run_nowtime[i+1]<=0xFF00*10){continue_run_nowtime[i+1]--;}//反转采用通道1或通道3控制参数
					if(continue_run_nowtime[i+1]==0)
          {
					 ROLLER2_STOP;
					 roller2_state=0;//0表示停；1表示上行；2表示下行		
					 bidirection_location_flg[i]=2;
					 continue_run_nowtime[i+1]=continue_run_time[i+1]*10;					 
				  }
				 }					
				
				bidirection_delay_flg[i]=0;
				bidirection_run_flg[i]=1;
			 }
				if(bidirection_delay_time[i]>=1){bidirection_delay_time[i]--;}
				if(bidirection_run_flg[i]!=1&&bidirection_run_time[i]>=1){ bidirection_run_time[i]--; }				
			 
				i++;				
				break;
			default:
      break;
			
		}
	}
}
/*求平均电流*/
u16 Get_Adclvbo(u8 TD_Xnumber,u16 TD_xiaxian,u16 TD_shangxian)
 {
   u8 i,j;
   u16 AdcLvbo[100],Temp_adc,t;
   u32 Temp_adcto=0;
   for(i=0;i<100;i++)
     {
			Temp_adc=ADC_ConvertedValue[TD_Xnumber];
      if(Temp_adc<TD_xiaxian){Temp_adc=TD_xiaxian;}  //过滤	   
	    if(Temp_adc>TD_shangxian){ Temp_adc=TD_shangxian;}	 //过滤	  
      AdcLvbo[i]= Temp_adc;
     }

   for(i=0;i<100;i++)//100个数排序
   {
      for(j=i+1;j<100;j++)
       {
         if(AdcLvbo[i]>=AdcLvbo[j])
	     {
	       t=AdcLvbo[i];
	       AdcLvbo[i]=AdcLvbo[j];
	       AdcLvbo[j]=t;
	     }
       }
   }
   for(i=40;i<60;i++)	
   {
     Temp_adcto=Temp_adcto+AdcLvbo[i];
   }
	Temp_adc=Temp_adcto/20;  
  return Temp_adc;
}

u16 First_Getaverage(u8 td_xnumber,u8 maxlvbo_xnumber,u16 temp_adc)
{  u8 i;
   for(i=0;i<maxlvbo_xnumber;i++)
   {
   	  Adc_average[td_xnumber][i] =temp_adc;
	  
   }  return temp_adc;
}

u16 TD_Getaverage(u8 td_xnumber,u8 tdlvbo_xnumber,u16 temp_xadc,u8 tdcycle_xi)
{ u8 i;
  u32 average_adcto=0;
  Adc_average[td_xnumber][tdcycle_xi] =temp_xadc;
  for(i=0;i<tdlvbo_xnumber;i++)
  {
  	 average_adcto=average_adcto+Adc_average[td_xnumber][i];  //求和
  }
  temp_xadc=average_adcto/tdlvbo_xnumber;  //求平均值
  return temp_xadc;
}











