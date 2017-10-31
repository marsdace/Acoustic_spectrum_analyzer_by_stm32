#include "adc.h"
#include "delay.h"		 
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ADC 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//初始化ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟

	
  //先初始化ADC1通道5 IO口
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 通道5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
 
	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 
	ADC_DeInit();
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频2分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//初始化
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
 // ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_84Cycles);
	//ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);
	//ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
	
	//ADC_SoftwareStartConv(ADC1);
	/************************ADC 看门狗设置******************************/
//	ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
//ADC_AnalogWatchdogThresholdsConfig(ADC1, 0x0E8B, 0x0555);//??????:3V ?:1V
//ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_5);
//ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
//ADC_DMACmd(ADC1, ENABLE); //是能ADC DMA
//ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); //单通道模式下上次转换完成后DMA请求允许，也就是持续DMA
	

}				  
//获得ADC值
//ch: @ref ADC_channels 
//通道值 0~16取值范围为：ADC_Channel_0~ADC_Channel_16
//返回值:转换结果
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADC通道,480个周期,提高采样时间可以提高精确度			    
  
	ADC_SoftwareStartConv(ADC1);		//使能指定的ADC1的软件转换启动功能	
	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束

	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}
//获取通道ch的转换值，取times次,然后平均 
//ch:通道编号
//times:获取次数
//返回值:通道ch的times次转换结果平均值
u16 Get_Adc_Average(u8 ch,u8 times)
{
	u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 
	 



/********************定时器 DMA等设置，参考战舰频谱的程序*******************************/
void TIM2_Configuration(void)
{
   	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //  TIM_OCInitTypeDef  TIM_OCInitStructure ;
    TIM_DeInit(TIM2);                              //复位TIM1定时器
        
    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 25;        //定时中断频率=72M/71+1/25     
    TIM_TimeBaseStructure.TIM_Prescaler = 78;    // 分频36000       78---20.48k
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // 时钟分频  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //计数方向向上计数
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 /******************  测试采样频率引脚*******************************************************/
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//GPIOF9
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??????
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//??
 GPIO_Init(GPIOF, &GPIO_InitStructure);//??? GPIO
	/*******************************************************/
    /* Clear TIM2 update pending flag[清除TIM2溢出中断标志] */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//初始化TIM3
    /* Enable TIM2 Update interrupt [TIM2溢出中断允许]*/
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
 
    /* TIM2 enable counter [允许tim2计数]*/
    TIM_Cmd(TIM2, DISABLE);   
}

void TIM2_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

//  /* Enable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //???????? 1
//NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;//?????DMA2_Stream0_IRQn
//NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //?????? 1
//NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;//?????? 0
//NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;//??????
//NVIC_Init(&NVIC_InitStructure);
}




