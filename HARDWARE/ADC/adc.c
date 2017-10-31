#include "adc.h"
#include "delay.h"		 
#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ADC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//��ʼ��ADC															   
void  Adc_Init(void)
{    
  GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOAʱ��
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��

	
  //�ȳ�ʼ��ADC1ͨ��5 IO��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;//PA5 ͨ��5
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  
 
	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1��λ
	//RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//��λ����	 
	ADC_DeInit();
  ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//���������׶�֮����ӳ�5��ʱ��
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ2��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
  ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ɨ��ģʽ	
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
 // ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
  ADC_InitStructure.ADC_NbrOfConversion = 1;//1��ת���ڹ��������� Ҳ����ֻת����������1 
  ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_84Cycles);
	//ADC_DMARequestAfterLastTransferCmd(ADC1,ENABLE);
	//ADC_DMACmd(ADC1,ENABLE);
	ADC_Cmd(ADC1, ENABLE);//����ADת����	
	
	//ADC_SoftwareStartConv(ADC1);
	/************************ADC ���Ź�����******************************/
//	ADC_AnalogWatchdogCmd(ADC1, ADC_AnalogWatchdog_SingleRegEnable);
//ADC_AnalogWatchdogThresholdsConfig(ADC1, 0x0E8B, 0x0555);//??????:3V ?:1V
//ADC_AnalogWatchdogSingleChannelConfig(ADC1, ADC_Channel_5);
//ADC_ITConfig(ADC1, ADC_IT_AWD, ENABLE);
//ADC_DMACmd(ADC1, ENABLE); //����ADC DMA
//ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); //��ͨ��ģʽ���ϴ�ת����ɺ�DMA��������Ҳ���ǳ���DMA
	

}				  
//���ADCֵ
//ch: @ref ADC_channels 
//ͨ��ֵ 0~16ȡֵ��ΧΪ��ADC_Channel_0~ADC_Channel_16
//����ֵ:ת�����
u16 Get_Adc(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );	//ADC1,ADCͨ��,480������,��߲���ʱ�������߾�ȷ��			    
  
	ADC_SoftwareStartConv(ADC1);		//ʹ��ָ����ADC1�����ת����������	
	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������

	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}
//��ȡͨ��ch��ת��ֵ��ȡtimes��,Ȼ��ƽ�� 
//ch:ͨ�����
//times:��ȡ����
//����ֵ:ͨ��ch��times��ת�����ƽ��ֵ
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
	 



/********************��ʱ�� DMA�����ã��ο�ս��Ƶ�׵ĳ���*******************************/
void TIM2_Configuration(void)
{
   	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //  TIM_OCInitTypeDef  TIM_OCInitStructure ;
    TIM_DeInit(TIM2);                              //��λTIM1��ʱ��
        
    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 25;        //��ʱ�ж�Ƶ��=72M/71+1/25     
    TIM_TimeBaseStructure.TIM_Prescaler = 78;    // ��Ƶ36000       78---20.48k
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  // ʱ�ӷ�Ƶ  
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //�����������ϼ���
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
 /******************  ���Բ���Ƶ������*******************************************************/
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;//GPIOF9
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//??????
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//????
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//??
 GPIO_Init(GPIOF, &GPIO_InitStructure);//??? GPIO
	/*******************************************************/
    /* Clear TIM2 update pending flag[���TIM2����жϱ�־] */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
//��ʼ��TIM3
    /* Enable TIM2 Update interrupt [TIM2����ж�����]*/
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
 
    /* TIM2 enable counter [����tim2����]*/
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




