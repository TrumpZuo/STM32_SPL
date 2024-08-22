
/**
  * @brief  开启或关闭APB2 (High Speed APB)外设时钟开关.
  * @param  RCC_APB2Periph: 
  			 RCC_APB2Periph_AFIO, RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB,
  *          RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE,
  *          RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG, RCC_APB2Periph_ADC1,
  *          RCC_APB2Periph_ADC2, RCC_APB2Periph_TIM1, RCC_APB2Periph_SPI1,
  *          RCC_APB2Periph_TIM8, RCC_APB2Periph_USART1, RCC_APB2Periph_ADC3,
  *          RCC_APB2Periph_TIM15, RCC_APB2Periph_TIM16, RCC_APB2Periph_TIM17,
  *          RCC_APB2Periph_TIM9, RCC_APB2Periph_TIM10, RCC_APB2Periph_TIM11     
  * @param  NewState:  ENABLE or DISABLE.
  */
  
void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
//使能 时钟 
/**
  * @brief  根据指定初始化GPIOx外设GPIO_InitStruct中的参数。
  * @param  GPIOx
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure 
  * @retval None
  */




void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
//关于GPIO_InitStruct，定义：
typedef struct
{
  uint16_t GPIO_Pin;             /* GPIO_Pin_0 GPIO_Pin_15 GPIO_Pin_All */

  GPIOSpeed_TypeDef GPIO_Speed;  /*  GPIO_Speed_10MHz = 1,GPIO_Speed_2MHz, GPIO_Speed_50MHz */

  GPIOMode_TypeDef GPIO_Mode;    /* GPIO_Mode_AIN = 0x0, ADC专用模式
  									GPIO_Mode_IN_FLOATING = 0x04,
 									GPIO_Mode_IPD = 0x28,  
									GPIO_Mode_IPU = 0x48,   input-pull-up 上接电阻成立，浮空默认为高，输入低电平为有效；
									
  									GPIO_Mode_Out_OD = 0x14, 
									GPIO_Mode_Out_PP = 0x10,
  									GPIO_Mode_AF_OD = 0x1C, 
									GPIO_Mode_AF_PP = 0x18 */
}GPIO_InitTypeDef;

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); // 获得输入
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);// 获得当前 输出




//使用 外部中断：

//AFIO，设置数据选择器，开启外部中断线；选择用作EXTI线的GPIO引脚。
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)；
//GPIO_PortSourceGPIOA GPIO_PortSourceGPIOE    
// 0x00-0x05,不支持 或 操作
// GPIO_PinSource0 GPIO_PinSource15  
// 0x00 - 0x0f,不支持 或 操作

void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
//EXTI_InitStruct.EXTI_Line : EXTI_Line0 EXTI_Line19
//  支持 或 操作；
//EXTI_InitStruct.EXTI_LineCmd : ENABLE DISABLE
//EXTI_InitStruct.EXTI_Mode: EXTI_Mode_Interrupt  EXTI_Mode_Event; 触发中断、事件
//EXTI_InitStruct.EXTI_Trigger: 
// EXTI_Trigger_Rising   EXTI_Trigger_Falling   EXTI_Trigger_Rising_Falling 


//设置NVIC通道、优先级


void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
// NVIC_PriorityGroup NVIC_PriorityGroup_2(均分) NVIC_PriorityGroup_4

void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);

//NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE DISABLE
//NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = ;
//NVIC_InitStruct.NVIC_IRQChannelSubPriority = ;

//NVIC_InitStruct.NVIC_IRQChannel    stm32.h 文件下
//WWDG_IRQn     PVD_IRQn    TAMPER_IRQn     TC_IRQn 
//FLASH_IRQn      RCC_IRQn    

//EXTI0_IRQn EXTI1_IRQn EXTI2_IRQn EXTI3_IRQn EXTI4_IRQn 

//DMA1_Channel1_IRQn DMA1_Channel2_IRQn DMA1_Channel3_IRQn DMA1_Channel4_IRQn DMA1_Channel5_IRQn DMA1_Channel6_IRQn DMA1_Channel7_IRQn

/*#ifdef STM32F10X_MD   
ADC1_2_IRQn     USB_HP_CAN1_TX_IRQn 
USB_LP_CAN1_RX0_IRQn      CAN1_RX1_IRQn     CAN1_SCE_IRQn  
EXTI9_5_IRQn  EXTI15_10_IRQn

TIM1_BRK_IRQn  TIM1_UP_IRQn    TIM1_TRG_COM_IRQn 
TIM1_CC_IRQn TIM2_IRQn  TIM3_IRQn  TIM4_IRQn 

I2C1_EV_IRQn  I2C1_ER_IRQn
I2C2_EV_IRQn  I2C2_ER_IRQn  

SPI1_IRQn SPI2_IRQn 
USART1_IRQn  USART2_IRQn USART3_IRQn 
RTCAlarm_IRQn   USBWakeUp_IRQn    */ 

//中断函数 可以 查看md.s的启动文件，Handler

void EXTI15_10_IRQHandler(void);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
//EXTI_InitStruct.EXTI_Line : EXTI_Line0 EXTI_Line19
// ITStatus = RESET SET 
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);//中断执行完成 必须调用，否则 一直在中断函数；


//使用定时器；

void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
//* RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
//* RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
//* RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
//* RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4,
//* RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
//* RCC_APB1Periph_USB, RCC_APB1Periph_CAN1, RCC_APB1Periph_BKP,
//* RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_CEC,
//* RCC_APB1Periph_TIM12, RCC_APB1Periph_TIM13, RCC_APB1Periph_TIM14

//选择  内部时钟源
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);

//设置时基单元
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);

/*TIM_TimeBaseInitStruct.TIM_ClockDivision = ;  用来调整滤波器的 频率；
 TIM_CKD_DIV1 此时滤波器的频率 和定时器的频率一样        TIM_CKD_DIV2     TIM_CKD_DIV4    */           

//TIM_TimeBaseInitStruct.TIM_CounterMode 
//TIM_CounterMode_Up                TIM_CounterMode_Down               
//TIM_CounterMode_CenterAligned1    TIM_CounterMode_CenterAligned2    TIM_CounterMode_CenterAligned3 

TIM_TimeBaseInitStruct.TIM_Period = ;//自动 重装计数器的值
TIM_TimeBaseInitStruct.TIM_Prescaler = ;//预分频， 注意减1的操作；
//TIM_TimeBaseInitStruct.TIM_RepetitionCounter = ;重复计数器，高级定时器可用；一般设为 0；


// 中断使能
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
//*@param TIM_IT : 指定要启用或禁用的TIM中断源
//*@arg TIM_IT_Update : TIM update Interrupt source
//* @arg TIM_IT_CC1 : TIM Capture Compare 1 Interrupt source
//* @arg TIM_IT_CC2 : TIM Capture Compare 2 Interrupt source
//* @arg TIM_IT_CC3 : TIM Capture Compare 3 Interrupt source
//* @arg TIM_IT_CC4 : TIM Capture Compare 4 Interrupt source
//* @arg TIM_IT_COM : TIM Commutation Interrupt source
//* @arg TIM_IT_Trigger : TIM Trigger Interrupt source
//* @arg TIM_IT_Break : TIM Break Interrupt source
//* @note
//* -TIM6 and TIM7 can only generate an update interrupt.
//* -TIM9, TIM12 and TIM15 can have only TIM_IT_Update, TIM_IT_CC1,
//* TIM_IT_CC2 or TIM_IT_Trigger.
//* -TIM10, TIM11, TIM13, TIM14, TIM16 and TIM17 can have TIM_IT_Update or TIM_IT_CC1.
//* -TIM_IT_Break is used only with TIM1, TIM8 and TIM15.
//* -TIM_IT_COM is used only with TIM1, TIM8, TIM15, TIM16 and TIM17.



// 定时器使能，开始启动；
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

//定时器的 状态检查
ITStatus TIM_GetITStatus(TIM_TypeDef* TIMx, uint16_t TIM_IT);
void TIM_ClearITPendingBit(TIM_TypeDef* TIMx, uint16_t TIM_IT);
/*@param  TIM_IT : specifies the pending bit to clear.
* @arg TIM_IT_Update : TIM1 update Interrupt source
* @arg TIM_IT_CC1 : TIM Capture Compare 1 Interrupt source
* @arg TIM_IT_CC2 : TIM Capture Compare 2 Interrupt source
* @arg TIM_IT_CC3 : TIM Capture Compare 3 Interrupt source
* @arg TIM_IT_CC4 : TIM Capture Compare 4 Interrupt source
* @arg TIM_IT_COM : TIM Commutation Interrupt source
* @arg TIM_IT_Trigger : TIM Trigger Interrupt source
* @arg TIM_IT_Break : TIM Break Interrupt source*/



//通过 GPIO到 ETR引脚，接入外部时钟源，mode1与mode2相差一个数据选择器；
void TIM_ETRClockMode2Config(TIM_TypeDef* TIMx, uint16_t TIM_ExtTRGPrescaler,uint16_t TIM_ExtTRGPolarity, uint16_t ExtTRGFilter)
/** @brief  Configures the External clock Mode1
  * @param  TIMx: where x can be  1, 2, 3, 4, 5 or 8 to select the TIM peripheral.
  * 
  * @param  TIM_ExtTRGPrescaler: The external Trigger Prescaler.
  *     @arg TIM_ExtTRGPSC_OFF: ETRP Prescaler OFF.
  *     @arg TIM_ExtTRGPSC_DIV2: ETRP frequency divided by 2.
  *     @arg TIM_ExtTRGPSC_DIV4: ETRP frequency divided by 4.
  *     @arg TIM_ExtTRGPSC_DIV8: ETRP frequency divided by 8.
  * 
  * @param  TIM_ExtTRGPolarity: The external Trigger Polarity.
  *     @arg TIM_ExtTRGPolarity_Inverted: active low or falling edge active.
  *     @arg TIM_ExtTRGPolarity_NonInverted: active high or rising edge active.
  * 
  * @param  ExtTRGFilter: 滤波器频率分频，与输入频率 的分频；
  *   This parameter must be a value between 0x00 and 0x0F
  */

  void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);

//设置 输出比较
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
/*TIM_OCInitStruct.TIM_OCMode = ;
TIM_OCMode_Timing 冻结   TIM_OCMode_Active          TIM_OCMode_Inactive     TIM_OCMode_Toggle 均为一次性使用         
TIM_OCMode_PWM1      TIM_OCMode_PWM2 */                 

/*TIM_OCInitStruct.TIM_OCPolarity = ;
 TIM_OCPolarity_High    TIM_OCPolarity_Low   */           

/*TIM_OCInitStruct.TIM_OutputState = ;
TIM_OutputState_Disable      TIM_OutputState_Enable        */     

TIM_OCInitStruct.TIM_Pulse = ;
// CCR 的值，可以是 0 - 0xffff
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);//其余成员变量默认写入


//在运行中，设置CCR的值
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);



// Input Capture 配置
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
//TIM_ICInitStruct.TIM_Channel = ;
// TIM_Channel_1   TIM_Channel_2    TIM_Channel_3     TIM_Channel_4       （不能 与）          

//TIM_ICInitStruct.TIM_ICFilter = ;输入信号滤波
// This parameter can be a number between 0x0 and 0xF

//TIM_ICInitStruct.TIM_ICPolarity = ;
// TIM_ICPolarity_Rising      TIM_ICPolarity_Falling         TIM_ICPolarity_BothEdge     


//TIM_ICInitStruct.TIM_ICPrescaler = ;输入的PWM是否分频
//TIM_ICPSC_DIV1 不分频  TIM_ICPSC_DIV2      TIM_ICPSC_DIV4        TIM_ICPSC_DIV8


//TIM_ICInitStruct.TIM_ICSelection = ;
//TIM_ICSelection_DirectTI  直连通道 TIM_ICSelection_IndirectTI  交叉通道    TIM_ICSelection_TRC 

void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
//可以快速 在通道1与通道2之间 完成相反的配置；

//主从 触发模式 配置：

void TIM_SelectInputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_InputTriggerSource);
/*@param  TIM_InputTriggerSource : The Input Trigger source.
*@arg TIM_TS_ITR0 : Internal Trigger 0
* @arg TIM_TS_ITR1 : Internal Trigger 1
* @arg TIM_TS_ITR2 : Internal Trigger 2
* @arg TIM_TS_ITR3 : Internal Trigger 3
* @arg TIM_TS_TI1F_ED : TI1 Edge Detector
* @arg TIM_TS_TI1FP1 : Filtered Timer Input 1
* @arg TIM_TS_TI2FP2 : Filtered Timer Input 2
* @arg TIM_TS_ETRF : External Trigger input*/
void TIM_SelectSlaveMode(TIM_TypeDef* TIMx, uint16_t TIM_SlaveMode);
/*@param  TIM_SlaveMode : specifies the Timer Slave Mode.
*@arg TIM_SlaveMode_Reset : 所选触发信号的上升沿(TRGI),初始化计数器并触发寄存器的更新。
* @arg TIM_SlaveMode_Gated : The counter clock is enabled when the trigger signal(TRGI) is high.
* @arg TIM_SlaveMode_Trigger : 计数器从触发TRGI的上升边缘开始。
* @arg TIM_SlaveMode_External1 : Rising edges of the selected trigger(TRGI) clock the counter.*/

void TIM_SelectOutputTrigger(TIM_TypeDef* TIMx, uint16_t TIM_TRGOSource);
/*@param  TIM_TRGOSource : specifies the Trigger Output source.
*-For all TIMx
* @arg TIM_TRGOSource_Reset : The UG bit in the TIM_EGR register is used as the trigger output(TRGO).
* @arg TIM_TRGOSource_Enable : The Counter Enable CEN is used as the trigger output(TRGO).
* @arg TIM_TRGOSource_Update : The update event is selected as the trigger output(TRGO).
*
*-For all TIMx except TIM6 and TIM7
* @arg TIM_TRGOSource_OC1 : The trigger output sends a positive pulse when the CC1IF flag
* is to be set, as soon as a capture or compare match occurs(TRGO).
* @arg TIM_TRGOSource_OC1Ref : OC1REF signal is used as the trigger output(TRGO).
* @arg TIM_TRGOSource_OC2Ref : OC2REF signal is used as the trigger output(TRGO).
* @arg TIM_TRGOSource_OC3Ref : OC3REF signal is used as the trigger output(TRGO).
* @arg TIM_TRGOSource_OC4Ref : OC4REF signal is used as the trigger output(TRGO).*/


//编码器模式 
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
    uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
/*@param  TIM_EncoderMode : specifies the TIMx Encoder Mode.
*@arg TIM_EncoderMode_TI1 : Counter counts on TI1FP1 edge depending on TI2FP2 level.
* @arg TIM_EncoderMode_TI2 : Counter counts on TI2FP2 edge depending on TI1FP1 level.
* @arg TIM_EncoderMode_TI12 : 两路正交信号的上升与下降沿 均 触发，可以抗噪声；

* @param  TIM_IC1Polarity TIM_IC2Polarity: specifies the IC1 Polarity
*@arg TIM_ICPolarity_Falling 
* @arg TIM_ICPolarity_Rising */


//得到 计数寄存器的值：
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);

//ADC校准函数
ADC_ResetCalibration(ADC1);
while (ADC_GetResetCalibrationStatus(ADC1) == SET);
ADC_StartCalibration(ADC1);
while (ADC_GetCalibrationStatus(ADC1) == SET);

//设置ADC通道
ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_7Cycles5);
//软件 触发 ADC中断
ADC_SoftwareStartConvCmd(ADC1, ENABLE);
while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
return ADC_GetConversionValue(ADC1);//转换结束标志EOC将会自动清除；



//USART
FlagStatus USART_GetFlagStatus(USART_TypeDef * USARTx, uint16_t USART_FLAG);
//  USART_FLAG_TXE:  Transmit data register empty flag,  RESET表示 发送中
//  USART_FLAG_RXNE : Receive data register not empty flag
//标志位将 自动 清0；

//开启中断，之后应该设置 NVIC
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//总开关
USART_Cmd(USART1, ENABLE);



//IIC的硬件开启配置

//检测 事件的发生
ErrorStatus I2C_CheckEvent(I2C_TypeDef * I2Cx, uint32_t I2C_EVENT);
/**
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED           : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED              : EV1
  *     @arg I2C_EVENT_SLAVE_TRANSMITTER_SECONDADDRESS_MATCHED     : EV1
  *     @arg I2C_EVENT_SLAVE_RECEIVER_SECONDADDRESS_MATCHED        : EV1
  *     @arg I2C_EVENT_SLAVE_GENERALCALLADDRESS_MATCHED            : EV1
  * 
  *     @arg I2C_EVENT_SLAVE_BYTE_RECEIVED                         : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_DUALF)      : EV2
  *     @arg (I2C_EVENT_SLAVE_BYTE_RECEIVED | I2C_FLAG_GENCALL)    : EV2
  *     @arg I2C_EVENT_SLAVE_BYTE_TRANSMITTED                      : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_DUALF)   : EV3
  *     @arg (I2C_EVENT_SLAVE_BYTE_TRANSMITTED | I2C_FLAG_GENCALL) : EV3
  *     @arg I2C_EVENT_SLAVE_ACK_FAILURE                           : EV3_2
  *     @arg I2C_EVENT_SLAVE_STOP_DETECTED                         : EV4
  *     @arg I2C_EVENT_MASTER_MODE_SELECT                          : EV5
  *     @arg I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED            : EV6
  *     @arg I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED               : EV6
  *     @arg I2C_EVENT_MASTER_BYTE_RECEIVED                        : EV7
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTING                    : EV8
  *     @arg I2C_EVENT_MASTER_BYTE_TRANSMITTED                     : EV8_2
  *     @arg I2C_EVENT_MASTER_MODE_ADDRESS10                       : EV9
  *
  * @retval An ErrorStatus enumeration value:
  * - SUCCESS: Last event is equal to the I2C_EVENT
  * - ERROR: Last event is different from the I2C_EVENT
  */







