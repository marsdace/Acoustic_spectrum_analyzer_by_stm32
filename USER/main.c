#include "sys.h"
#include "delay.h"  
#include "usart.h"  
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "timer.h" 
#include "math.h" 
#include "arm_math.h"  
#include "adc.h" 
#include "dma.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx.h"

//ALIENTEK 探索者STM32F407开发板 实验47_2
//DSP FFT测试实验   -库函数版本
//技术支持：www.openedv.com
//淘宝店铺：http://eboard.taobao.com  
//广州市星翼电子科技有限公司  
//作者：正点原子 @ALIENTEK


#define FFT_LENGTH		4096 		//FFT长度，默认是1024点FFT
#define Fs 						40960		//40.96kHz
#define ADC1_DR_ADDRESS     ((uint32_t)0x4001204C)
 uint16_t ADC1Value[FFT_LENGTH]={0};//dma接收到的数据存储位置
  uint16_t aaa[1024]={0};
uint8_t ADC_TimeOutFlag;			//ADC定时采样时间到标志	
float32_t fft_inputbuf[FFT_LENGTH*2];	//FFT输入数组
float32_t fft_outputbuf[FFT_LENGTH];	//FFT输出数组
uint32_t ADC_DataNum=0;			//ADC采样点数
	unsigned int graphic[32];
	uint8_t PINLV=7;//频率区间7k~15k


u8 timeout;//定时器溢出次数
void fft_dma_Init(void);
void FFT(void);
void KEY_Init(void);


int main(void)
{ 
	arm_cfft_radix4_instance_f32 S; 
 	u8 key,t=0;
	u16 adcx,a;
	float temp;
	float time; 
	u8 buf[50]; 
	u16 i,ii,iiflag; 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);  //初始化延时函数
	uart_init(115200);		//初始化串口波特率为115200
	
	LED_Init();					//初始化LED
	KEY_Init();					//初始化按键
 	LCD_Init();					//初始化LCD
	TIM2_Configuration();

	TIM2_NVIC_Configuration();

  Adc_Init();         //初始化ADC 

	fft_dma_Init();	

	//TIM3_Int_Init(65535,84-1);	//1Mhz计数频率,最大计时65ms左右超出
 	POINT_COLOR=RED;	//设置字体为蓝色  
				for(i=1;i<101;i++){
					if(i<41)LCD_ShowxNum(82, 20*(i-1),i*10,5,16,0);		//显示整数部分
					else if((i>40)&&(i<81)){LCD_ShowxNum(232, 20*(i-41),i*10,5,16,0);}
					else if(i>80){LCD_ShowxNum(382, 20*(i-81),i*10,5,16,0);}
				}
	POINT_COLOR=BLUE;
	TIM_Cmd(TIM2, ENABLE);
		
 while(1)
	{int i=0;
	key=KEY_Scan(0);
		if(key)
		{						   
			switch(key)
			{				 
				case WKUP_PRES:	//控制蜂鸣器
						if(PINLV>14)PINLV=7;
						else PINLV+=1;
						LCD_ShowxNum(382, 450,	PINLV,5,16,0);
						break;
				case KEY1_PRES:	//控制LED1翻转	 
						if(PINLV<8)PINLV=15;
						else PINLV-=1;
				LCD_ShowxNum(382, 450,	PINLV,5,16,0);
						break;
					break;
			}
		}
			if(ADC_TimeOutFlag==1){//定时采样标志位
				 ADC_TimeOutFlag=0;
				 if(ADC_DataNum<FFT_LENGTH){					 
					 ADC_SoftwareStartConv(ADC1);	 
					 //while(DMA_GetFlagStatus(DMA2_Stream0,DMA_FLAG_TCIF0) != RESET)				 
					 DMA_ClearITPendingBit(DMA2_Stream0,DMA_IT_TCIF0);
					 fft_inputbuf[ADC_DataNum*2]=(float32_t)ADC1Value[0];	
					 fft_inputbuf[ADC_DataNum*2+1]=0;	
					 //fft_inputbuf[ADC_DataNum]=ADC1Value[0]<<16;
						//aaa[ADC_DataNum]	=ADC1Value[0];				 
					 //fft_inputbuf[ADC_DataNum*2+1]=0;						 
					 ADC_DataNum++;
					 //printf("%d\n",ADC_DataNum);
				 }
				 else{
					 TIM_Cmd(TIM2, DISABLE);
					 ADC_DataNum=0;

//					 for(i=0;i<FFT_LENGTH;i++){//测试使用
//					 fft_inputbuf[2*i]=(1+1*arm_sin_f32(20*PI*i/Fs)+3*arm_sin_f32(20*PI*i*4/Fs)+5*arm_sin_f32(20*PI*i*8/Fs));		
//					}
					 arm_cfft_radix4_init_f32(&S, FFT_LENGTH,0, 1); 
					 arm_cfft_radix4_f32(&S,fft_inputbuf); 
					 arm_cmplx_mag_f32(fft_inputbuf, fft_outputbuf, FFT_LENGTH);  
/*
采样点数：4096
fft_inputbuf有8192个数据，奇数为实部，偶数为虚部，所以奇数为adc采样值，虚部为0
Fs（采样频率）：当前为20.48khz，可正确运行，但是根据计算应该为40.96khz，未找到原因
所以分辨率为10hz
fft_outputbuf[0]是该频率点幅值的4096倍，即0hz(直流分量)幅值为	fft_outputbuf[0]/4096
fft_outputbuf[1~4096]是该频率点幅值点的2048倍，例如20hz的幅值为	fft_outputbuf[2]/2048
fft_outputbuf的4096个点具有共轭反对称性，即fft_outputbuf[1]=fft_outputbuf[4095] fft_outputbuf[2047]=fft_outputbuf[2049]
所以只考虑fft_outputbuf[0]~fft_outputbuf[2048]，即0~20480hz的幅值						 
*/		

				fft_outputbuf[0]=fft_outputbuf[0]/2;
				switch (PINLV){
					case 7:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[700+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[700+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[700+i],9,16,0);
										}
								}
								break;
					case 8:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[800+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[800+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[800+i],9,16,0);
										}
								}
								break;			
					case 9:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[900+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[900+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[900+i],9,16,0);
										}
								}
								break;	
					case 10:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[1000+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[1000+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[1000+i],9,16,0);
										}
								}
								break;
					case 11:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[1100+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[1100+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[1100+i],9,16,0);
										}
								}
								break;	
					case 12:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[1200+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[1200+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[1200+i],9,16,0);
										}
								}
								break;		
					case 13:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[1300+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[1300+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[1300+i],9,16,0);
										}
								}
								break;	
					case 14:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[1400+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[1400+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[1400+i],9,16,0);
										}
								}
								break;
					case 15:
								for(i=1;i<101;i++){
								//LCD_ShowxNum(0, 20*i,i*50,10,16,0);		//显示整数部分
									if(i<41){
									LCD_ShowxNum(0, 20*(i-1),fft_outputbuf[1500+i],9,16,0);}		//显示整数部分
									else if((40<i)&&(i<81)){
									LCD_ShowxNum(150, 20*(i-41),fft_outputbuf[1500+i],9,16,0);}
									else if(i>80){
									LCD_ShowxNum(300, 20*(i-81),fft_outputbuf[1500+i],9,16,0);
										}
								}
								break;									
			}
///************************150hz   200 hz*************************************/
//LCD_ShowxNum(0, 0,fft_outputbuf[10],9,16,0);		//100	
//LCD_ShowxNum(0, 20,fft_outputbuf[11],9,16,0);		//110
//LCD_ShowxNum(0, 40,fft_outputbuf[12],9,16,0);		//120
//LCD_ShowxNum(0, 60,fft_outputbuf[13],9,16,0);		//130
//LCD_ShowxNum(0, 80,fft_outputbuf[14],9,16,0);		//140

//LCD_ShowxNum(0, 120,fft_outputbuf[15],9,16,0);		//150	

//LCD_ShowxNum(0, 160,fft_outputbuf[16],9,16,0);		//160
//LCD_ShowxNum(0, 180,fft_outputbuf[17],9,16,0);		//170
//LCD_ShowxNum(0, 200,fft_outputbuf[18],9,16,0);		//180
//LCD_ShowxNum(0, 220,fft_outputbuf[19],9,16,0);		//190

//LCD_ShowxNum(0, 260,fft_outputbuf[20],9,16,0);		//200	

//LCD_ShowxNum(0, 300,fft_outputbuf[21],9,16,0);		//210
//LCD_ShowxNum(0, 320,fft_outputbuf[22],9,16,0);		//220
//LCD_ShowxNum(0, 340,fft_outputbuf[23],9,16,0);		//230
//LCD_ShowxNum(0, 360,fft_outputbuf[24],9,16,0);		//240
///****************************************************************/

/*************************1k*************************************/
//LCD_ShowxNum(0, 0,fft_outputbuf[93],9,16,0);		//930	
//LCD_ShowxNum(0, 20,fft_outputbuf[94],9,16,0);		//940
//LCD_ShowxNum(0, 40,fft_outputbuf[95],9,16,0);		//950
//LCD_ShowxNum(0, 60,fft_outputbuf[96],9,16,0);		//960
//LCD_ShowxNum(0, 80,fft_outputbuf[97],9,16,0);		//970

//LCD_ShowxNum(0, 100,fft_outputbuf[98],9,16,0);		//980	
//LCD_ShowxNum(0, 120,fft_outputbuf[99],9,16,0);		//990
//LCD_ShowxNum(0, 160,fft_outputbuf[100],9,16,0);		//1000hz
//LCD_ShowxNum(0, 200,fft_outputbuf[101],9,16,0);		//1010
//LCD_ShowxNum(0, 220,fft_outputbuf[102],9,16,0);		//1020

//LCD_ShowxNum(0, 240,fft_outputbuf[103],9,16,0);		//1030
//LCD_ShowxNum(0, 260,fft_outputbuf[104],9,16,0);		//1040
//LCD_ShowxNum(0, 280,fft_outputbuf[105],9,16,0);		//1040
//LCD_ShowxNum(0, 340,fft_outputbuf[106],9,16,0);		//1060
//LCD_ShowxNum(0, 360,fft_outputbuf[107],9,16,0);		//1070
//LCD_ShowxNum(0, 380,fft_outputbuf[108],9,16,0);		//1080

///**********************10k**************************************/
//LCD_ShowxNum(300, 0,fft_outputbuf[993],9,16,0);		//9930	
//LCD_ShowxNum(300, 20,fft_outputbuf[994],9,16,0);		//9940
//LCD_ShowxNum(300, 40,fft_outputbuf[995],9,16,0);		//9950
//LCD_ShowxNum(300, 60,fft_outputbuf[996],9,16,0);		//9960
//LCD_ShowxNum(300, 80,fft_outputbuf[997],9,16,0);		//9970

//LCD_ShowxNum(300, 100,fft_outputbuf[998],9,16,0);		//9980	
//LCD_ShowxNum(300, 120,fft_outputbuf[999],9,16,0);		//9990
//LCD_ShowxNum(300, 160,fft_outputbuf[1000],9,16,0);		//10000hz
//LCD_ShowxNum(300, 200,fft_outputbuf[1001],9,16,0);		//10010
//LCD_ShowxNum(300, 220,fft_outputbuf[1002],9,16,0);		//10020

//LCD_ShowxNum(300, 240,fft_outputbuf[1003],9,16,0);		//10030
//LCD_ShowxNum(300, 260,fft_outputbuf[1004],9,16,0);		//10040
//LCD_ShowxNum(300, 280,fft_outputbuf[1005],9,16,0);		//10050
//LCD_ShowxNum(300, 340,fft_outputbuf[1006],9,16,0);		//10060
//LCD_ShowxNum(300, 360,fft_outputbuf[1007],9,16,0);		//10070
//LCD_ShowxNum(300, 380,fft_outputbuf[1008],9,16,0);		//10080
///************************12k**************************************/
//LCD_ShowxNum(0, 0,fft_outputbuf[1193],9,16,0);			
//LCD_ShowxNum(0, 20,fft_outputbuf[1194],9,16,0);		
//LCD_ShowxNum(0, 40,fft_outputbuf[1195],9,16,0);		
//LCD_ShowxNum(0, 60,fft_outputbuf[1196],9,16,0);	
//LCD_ShowxNum(0, 80,fft_outputbuf[1197],9,16,0);		
//LCD_ShowxNum(0, 100,fft_outputbuf[1198],9,16,0);			
//LCD_ShowxNum(0, 120,fft_outputbuf[1199],9,16,0);		

//LCD_ShowxNum(0, 160,fft_outputbuf[1200],9,16,0);		//1200hz

//LCD_ShowxNum(0, 200,fft_outputbuf[1201],9,16,0);		
//LCD_ShowxNum(0, 220,fft_outputbuf[1202],9,16,0);		
//LCD_ShowxNum(0, 240,fft_outputbuf[1203],9,16,0);		
//LCD_ShowxNum(0, 260,fft_outputbuf[1204],9,16,0);		
//LCD_ShowxNum(0, 280,fft_outputbuf[1205],9,16,0);		
//LCD_ShowxNum(0, 300,fft_outputbuf[1206],9,16,0);		
//LCD_ShowxNum(0, 320,fft_outputbuf[1207],9,16,0);		
//LCD_ShowxNum(0, 340,fft_outputbuf[1208],9,16,0);		
/////************************15k**************************************/
//LCD_ShowxNum(300, 0,fft_outputbuf[1493],9,16,0);			
//LCD_ShowxNum(300, 20,fft_outputbuf[1494],9,16,0);		
//LCD_ShowxNum(300, 40,fft_outputbuf[1495],9,16,0);		
//LCD_ShowxNum(300, 60,fft_outputbuf[1496],9,16,0);		
//LCD_ShowxNum(300, 80,fft_outputbuf[1497],9,16,0);		

//LCD_ShowxNum(300, 100,fft_outputbuf[1498],9,16,0);		
//LCD_ShowxNum(300, 120,fft_outputbuf[1499],9,16,0);	
//LCD_ShowxNum(300, 160,fft_outputbuf[1500],9,16,0);		//15khz
//LCD_ShowxNum(300, 200,fft_outputbuf[1501],9,16,0);		
//LCD_ShowxNum(300, 220,fft_outputbuf[1502],9,16,0);		

//LCD_ShowxNum(300, 240,fft_outputbuf[1503],9,16,0);		
//LCD_ShowxNum(300, 260,fft_outputbuf[1504],9,16,0);		
//LCD_ShowxNum(300, 280,fft_outputbuf[1505],9,16,0);		
//LCD_ShowxNum(300, 300,fft_outputbuf[1506],9,16,0);		
//LCD_ShowxNum(300, 320,fft_outputbuf[1507],9,16,0);		
//LCD_ShowxNum(300, 340,fft_outputbuf[1508],9,16,0);						
					 TIM_Cmd(TIM2, ENABLE);					 
				 }	  							
  }
}
 }

 void fft_dma_Init(void)
 {
 	DMA_InitTypeDef  DMA_InitStructure;
	  NVIC_InitTypeDef NVIC_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	 
	DMA_DeInit(DMA2_Stream0);
	//while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE){}//等待DMA可配置 
	 
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  //通道选择
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&ADC1->DR);//DMA外设地址  ..par
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1Value;//DMA 存储器0地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
  DMA_InitStructure.DMA_BufferSize = 1;//ndtr;//每次只传1个数据传输量 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//DMA_PeripheralDataSize_Byte;//外设数据长度:8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA_MemoryDataSize_Byte;//存储器数据长度:8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用普通模式 
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_Medium;//中等优先级
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);//初始化DMA Stream
	  DMA_Cmd(DMA2_Stream0, ENABLE);
		DMA_ITConfig(DMA2_Stream0,DMA_IT_TC,ENABLE);
  /*****************************************************************************************/
//  /* Enable DMA request after last transfer (Single-ADC mode) */
 ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
//  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
//  //??DMA??
  DMA_ITConfig(DMA2_Stream0,DMA_IT_TC , ENABLE);
//	/* Enable the DMA2_Stream0 gloabal Interrupt */
//  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
 }

 void TIM2_IRQHandler(void)
{
   	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET){
// 	   TIM2->SR = (uint16_t)~TIM_FLAG_Update;
	   ADC_TimeOutFlag=1;
		//printf("ttt");	
		GPIO_ToggleBits(GPIOF,GPIO_Pin_9); 
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);	//清中断
		}
} 


