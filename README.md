# Acoustic_spectrum_analyzer_by_stm32
Based on stm32 acoustic spectrum analyzer,使用stm32进行声波频谱分析
## 设备
原子探索者开发板STM32F4<br>
4.3寸屏幕<br>
## 功能说明
1.通过ADC1_5通道使用DMA进行数据采集，通过定时器TIM2定时采样。每次只采一个值。<br>
2.通过mic模块将声波升压到指定电压输出给stm32的ADC。如图<br>
3.GPIOF9引脚输出高低电平，其频率为TIM2的采样频率<br>
4.LCD现实100个频率的幅值，默认为7khz，分辨率为10hz，可通过按键KEY_UP和KEY1调节。7khz~15khz<br>
## 相关参数
1.FFT取样点：4096<br>
2.采样频率：20.48khz<br>
3.分辨率：10hz<br>
## 说明
实现方法即原理可参考References文件夹。<br>
