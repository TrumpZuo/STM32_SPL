
/**
  * @brief  ������ر�APB2 (High Speed APB)����ʱ�ӿ���.
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
//ʹ�� ʱ�� 
/**
  * @brief  ����ָ����ʼ��GPIOx����GPIO_InitStruct�еĲ�����
  * @param  GPIOx
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure 
  * @retval None
  */




void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct);
//����GPIO_InitStruct�����壺
typedef struct
{
  uint16_t GPIO_Pin;             /* GPIO_Pin_0 GPIO_Pin_15 GPIO_Pin_All */

  GPIOSpeed_TypeDef GPIO_Speed;  /*  GPIO_Speed_10MHz = 1,GPIO_Speed_2MHz, GPIO_Speed_50MHz */

  GPIOMode_TypeDef GPIO_Mode;    /* GPIO_Mode_AIN = 0x0, ADCר��ģʽ
  									GPIO_Mode_IN_FLOATING = 0x04,
 									GPIO_Mode_IPD = 0x28,  
									GPIO_Mode_IPU = 0x48,   input-pull-up �Ͻӵ������������Ĭ��Ϊ�ߣ�����͵�ƽΪ��Ч��
									
  									GPIO_Mode_Out_OD = 0x14, 
									GPIO_Mode_Out_PP = 0x10,
  									GPIO_Mode_AF_OD = 0x1C, 
									GPIO_Mode_AF_PP = 0x18 */
}GPIO_InitTypeDef;

void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin); // �������
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);// ��õ�ǰ ���




//ʹ�� �ⲿ�жϣ�

//AFIO����������ѡ�����������ⲿ�ж��ߣ�ѡ������EXTI�ߵ�GPIO���š�
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)��
//GPIO_PortSourceGPIOA GPIO_PortSourceGPIOE    
// 0x00-0x05,��֧�� �� ����
// GPIO_PinSource0 GPIO_PinSource15  
// 0x00 - 0x0f,��֧�� �� ����

void EXTI_Init(EXTI_InitTypeDef* EXTI_InitStruct);
//EXTI_InitStruct.EXTI_Line : EXTI_Line0 EXTI_Line19
//  ֧�� �� ������
//EXTI_InitStruct.EXTI_LineCmd : ENABLE DISABLE
//EXTI_InitStruct.EXTI_Mode: EXTI_Mode_Interrupt  EXTI_Mode_Event; �����жϡ��¼�
//EXTI_InitStruct.EXTI_Trigger: 
// EXTI_Trigger_Rising   EXTI_Trigger_Falling   EXTI_Trigger_Rising_Falling 


//����NVICͨ�������ȼ�


void NVIC_PriorityGroupConfig(uint32_t NVIC_PriorityGroup);
// NVIC_PriorityGroup NVIC_PriorityGroup_2(����) NVIC_PriorityGroup_4

void NVIC_Init(NVIC_InitTypeDef* NVIC_InitStruct);

//NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE DISABLE
//NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = ;
//NVIC_InitStruct.NVIC_IRQChannelSubPriority = ;

//NVIC_InitStruct.NVIC_IRQChannel    stm32.h �ļ���
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

//�жϺ��� ���� �鿴md.s�������ļ���Handler

void EXTI15_10_IRQHandler(void);
ITStatus EXTI_GetITStatus(uint32_t EXTI_Line);
//EXTI_InitStruct.EXTI_Line : EXTI_Line0 EXTI_Line19
// ITStatus = RESET SET 
void EXTI_ClearITPendingBit(uint32_t EXTI_Line);//�ж�ִ����� ������ã����� һֱ���жϺ�����


//ʹ�ö�ʱ����

void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
//* RCC_APB1Periph_TIM2, RCC_APB1Periph_TIM3, RCC_APB1Periph_TIM4,
//* RCC_APB1Periph_TIM5, RCC_APB1Periph_TIM6, RCC_APB1Periph_TIM7,
//* RCC_APB1Periph_WWDG, RCC_APB1Periph_SPI2, RCC_APB1Periph_SPI3,
//* RCC_APB1Periph_USART2, RCC_APB1Periph_USART3, RCC_APB1Periph_USART4,
//* RCC_APB1Periph_USART5, RCC_APB1Periph_I2C1, RCC_APB1Periph_I2C2,
//* RCC_APB1Periph_USB, RCC_APB1Periph_CAN1, RCC_APB1Periph_BKP,
//* RCC_APB1Periph_PWR, RCC_APB1Periph_DAC, RCC_APB1Periph_CEC,
//* RCC_APB1Periph_TIM12, RCC_APB1Periph_TIM13, RCC_APB1Periph_TIM14

//ѡ��  �ڲ�ʱ��Դ
void TIM_InternalClockConfig(TIM_TypeDef* TIMx);

//����ʱ����Ԫ
void TIM_TimeBaseInit(TIM_TypeDef* TIMx, TIM_TimeBaseInitTypeDef* TIM_TimeBaseInitStruct);

/*TIM_TimeBaseInitStruct.TIM_ClockDivision = ;  ���������˲����� Ƶ�ʣ�
 TIM_CKD_DIV1 ��ʱ�˲�����Ƶ�� �Ͷ�ʱ����Ƶ��һ��        TIM_CKD_DIV2     TIM_CKD_DIV4    */           

//TIM_TimeBaseInitStruct.TIM_CounterMode 
//TIM_CounterMode_Up                TIM_CounterMode_Down               
//TIM_CounterMode_CenterAligned1    TIM_CounterMode_CenterAligned2    TIM_CounterMode_CenterAligned3 

TIM_TimeBaseInitStruct.TIM_Period = ;//�Զ� ��װ��������ֵ
TIM_TimeBaseInitStruct.TIM_Prescaler = ;//Ԥ��Ƶ�� ע���1�Ĳ�����
//TIM_TimeBaseInitStruct.TIM_RepetitionCounter = ;�ظ����������߼���ʱ�����ã�һ����Ϊ 0��


// �ж�ʹ��
void TIM_ITConfig(TIM_TypeDef* TIMx, uint16_t TIM_IT, FunctionalState NewState);
//*@param TIM_IT : ָ��Ҫ���û���õ�TIM�ж�Դ
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



// ��ʱ��ʹ�ܣ���ʼ������
void TIM_Cmd(TIM_TypeDef* TIMx, FunctionalState NewState);

//��ʱ���� ״̬���
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



//ͨ�� GPIO�� ETR���ţ������ⲿʱ��Դ��mode1��mode2���һ������ѡ������
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
  * @param  ExtTRGFilter: �˲���Ƶ�ʷ�Ƶ��������Ƶ�� �ķ�Ƶ��
  *   This parameter must be a value between 0x00 and 0x0F
  */

  void TIM_ClearFlag(TIM_TypeDef* TIMx, uint16_t TIM_FLAG);

//���� ����Ƚ�
void TIM_OC1Init(TIM_TypeDef* TIMx, TIM_OCInitTypeDef* TIM_OCInitStruct);
/*TIM_OCInitStruct.TIM_OCMode = ;
TIM_OCMode_Timing ����   TIM_OCMode_Active          TIM_OCMode_Inactive     TIM_OCMode_Toggle ��Ϊһ����ʹ��         
TIM_OCMode_PWM1      TIM_OCMode_PWM2 */                 

/*TIM_OCInitStruct.TIM_OCPolarity = ;
 TIM_OCPolarity_High    TIM_OCPolarity_Low   */           

/*TIM_OCInitStruct.TIM_OutputState = ;
TIM_OutputState_Disable      TIM_OutputState_Enable        */     

TIM_OCInitStruct.TIM_Pulse = ;
// CCR ��ֵ�������� 0 - 0xffff
void TIM_OCStructInit(TIM_OCInitTypeDef* TIM_OCInitStruct);//�����Ա����Ĭ��д��


//�������У�����CCR��ֵ
void TIM_SetCompare1(TIM_TypeDef* TIMx, uint16_t Compare1);



// Input Capture ����
void TIM_ICInit(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
//TIM_ICInitStruct.TIM_Channel = ;
// TIM_Channel_1   TIM_Channel_2    TIM_Channel_3     TIM_Channel_4       ������ �룩          

//TIM_ICInitStruct.TIM_ICFilter = ;�����ź��˲�
// This parameter can be a number between 0x0 and 0xF

//TIM_ICInitStruct.TIM_ICPolarity = ;
// TIM_ICPolarity_Rising      TIM_ICPolarity_Falling         TIM_ICPolarity_BothEdge     


//TIM_ICInitStruct.TIM_ICPrescaler = ;�����PWM�Ƿ��Ƶ
//TIM_ICPSC_DIV1 ����Ƶ  TIM_ICPSC_DIV2      TIM_ICPSC_DIV4        TIM_ICPSC_DIV8


//TIM_ICInitStruct.TIM_ICSelection = ;
//TIM_ICSelection_DirectTI  ֱ��ͨ�� TIM_ICSelection_IndirectTI  ����ͨ��    TIM_ICSelection_TRC 

void TIM_PWMIConfig(TIM_TypeDef* TIMx, TIM_ICInitTypeDef* TIM_ICInitStruct);
//���Կ��� ��ͨ��1��ͨ��2֮�� ����෴�����ã�

//���� ����ģʽ ���ã�

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
*@arg TIM_SlaveMode_Reset : ��ѡ�����źŵ�������(TRGI),��ʼ���������������Ĵ����ĸ��¡�
* @arg TIM_SlaveMode_Gated : The counter clock is enabled when the trigger signal(TRGI) is high.
* @arg TIM_SlaveMode_Trigger : �������Ӵ���TRGI��������Ե��ʼ��
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


//������ģʽ 
void TIM_EncoderInterfaceConfig(TIM_TypeDef* TIMx, uint16_t TIM_EncoderMode,
    uint16_t TIM_IC1Polarity, uint16_t TIM_IC2Polarity);
/*@param  TIM_EncoderMode : specifies the TIMx Encoder Mode.
*@arg TIM_EncoderMode_TI1 : Counter counts on TI1FP1 edge depending on TI2FP2 level.
* @arg TIM_EncoderMode_TI2 : Counter counts on TI2FP2 edge depending on TI1FP1 level.
* @arg TIM_EncoderMode_TI12 : ��·�����źŵ��������½��� �� ���������Կ�������

* @param  TIM_IC1Polarity TIM_IC2Polarity: specifies the IC1 Polarity
*@arg TIM_ICPolarity_Falling 
* @arg TIM_ICPolarity_Rising */


//�õ� �����Ĵ�����ֵ��
uint16_t TIM_GetCounter(TIM_TypeDef* TIMx);

//ADCУ׼����
ADC_ResetCalibration(ADC1);
while (ADC_GetResetCalibrationStatus(ADC1) == SET);
ADC_StartCalibration(ADC1);
while (ADC_GetCalibrationStatus(ADC1) == SET);

//����ADCͨ��
ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_7Cycles5);
//��� ���� ADC�ж�
ADC_SoftwareStartConvCmd(ADC1, ENABLE);
while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
return ADC_GetConversionValue(ADC1);//ת��������־EOC�����Զ������



//USART
FlagStatus USART_GetFlagStatus(USART_TypeDef * USARTx, uint16_t USART_FLAG);
//  USART_FLAG_TXE:  Transmit data register empty flag,  RESET��ʾ ������
//  USART_FLAG_RXNE : Receive data register not empty flag
//��־λ�� �Զ� ��0��

//�����жϣ�֮��Ӧ������ NVIC
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
//�ܿ���
USART_Cmd(USART1, ENABLE);



//IIC��Ӳ����������

//��� �¼��ķ���
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







