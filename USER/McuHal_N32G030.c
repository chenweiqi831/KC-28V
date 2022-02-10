#include "McuHal_N32G030.h"


/*****************************************************************************
 �� �� ��  : Delay10us
 ��������  : 10us��ʱ����
 �������  : u16 TimeTick  
 �� �� ֵ  : 
*****************************************************************************/
void Delay10us(u32 uldelay)
{
    u32 i, j;
	__NOP();
    for(i = 0; i < uldelay; i++) 		// 48M
	{		
        for(j = 0; j < 62; j++) 	
		{
			;
        }
    }
}

/*****************************************************************************
 �� �� ��  : Delay10us_INT
 ��������  : 10us��ʱ����-�жϺ�����ʹ��
 �������  : u16 TimeTick  
 �� �� ֵ  : 
*****************************************************************************/
void Delay10us_INT(u32 uldelay)
{
    u32 i, j;
	__NOP();
    for(i = 0; i < uldelay; i++) 
	{		
        for(j = 0; j < 62; j++) 	
		{
			;
        }
    }
}

void Delay1ms(u16 TimeTick)
{
	if (TimeTick<1) 
	{
		TimeTick++;
	}
	
	while(TimeTick--) 
	{ 
		HwClrWdt();
		Delay10us(99);
	} 
}

/*****************************************************************************
 �� �� ��  : WdtInit
 ��������  : ���Ź���ʼ��
 			 WdtInit(IWDG_PRESCALER_DIV128, 61)����250MS�Ŀ��Ź���ʱ��
 			 WdtInit(IWDG_PRESCALER_DIV128, 61*4)����1000MS�Ŀ��Ź���ʱ��
 			 �������ʱ��Ҫ��ֹ����.
 �������  :   
 �� �� ֵ  : 
*****************************************************************************/
void WdtInit(void)
{
	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
       dispersion) */
    /* Enable write access to IWDG_PR and IWDG_RLR registers */
    IWDG_WriteConfig(IWDG_WRITE_ENABLE);

    /* IWDG counter clock: LSI/32 */
    IWDG_SetPrescalerDiv(IWDG_PRESCALER_DIV128);

	/* 1000ms */
    IWDG_CntReload(61*4);
    
    /* Reload IWDG counter */
    IWDG_ReloadKey();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
    IWDG_Enable();
}


void HwClrWdt(void)
{
	IWDG_ReloadKey();
}


/*****************************************************************************
 �� �� ��  : ADCInit
 ��������  : ADģ���ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void ADCInit(void)
{
	ADC_InitType ADC_InitStructure;

	/* Enable ADC clocks */
    RCC_EnableAHBPeriphClk(RCC_AHB_PERIPH_ADC, ENABLE);
	/* Enable ADC 1M clock */
	RCC_EnableHsi(ENABLE);
    RCC_ConfigAdc1mClk(RCC_ADC1MCLK_SRC_HSI, RCC_ADC1MCLK_DIV8);
    /* RCC_ADCHCLK_DIV16 */
    ADC_ConfigClk(ADC_CTRL3_CKMOD_AHB, RCC_ADCHCLK_DIV3);
	
     /* ADC configuration ------------------------------------------------------*/
    ADC_InitStructure.MultiChEn      = DISABLE;
    ADC_InitStructure.ContinueConvEn = DISABLE;
    ADC_InitStructure.ExtTrigSelect  = ADC_EXT_TRIGCONV_NONE;
    ADC_InitStructure.DatAlign       = ADC_DAT_ALIGN_R;
    ADC_InitStructure.ChsNumber      = 1;
    ADC_Init(ADC, &ADC_InitStructure);

    /* Enable ADC */
    ADC_Enable(ADC, ENABLE);
    
    /* Check ADC Ready */
    while(ADC_GetFlagStatusNew(ADC,ADC_FLAG_RDY) == RESET)
    {
        ;
    }
	while(ADC_GetFlagStatusNew(ADC,ADC_FLAG_PD_RDY))
	{
        ;                   
	}
}

/*****************************************************************************
 �� �� ��  : ADCDeInit
 ��������  : ADģ��ָ�Ĭ�ϲ���ֵ
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void ADCDeInit(void)
{
    ;
}

/*****************************************************************************
 �� �� ��  : ADRead
 ��������  : AD��ȡ��������10�Σ�ȥ�����Сֵ��ȡ8��ƽ��
 �������  : u8 channel  ADͨ��
 �� �� ֵ  : ��ȡ��ADֵ
*****************************************************************************/
u16 ADRead(u8 channel)
{
    u16 value = 0;
	u8 i = 0;
	u16 Ad_MaxValue = 0;
	u16 Ad_MinValue = 0;
	u16 Ad_Sum = 0;	

	// ����10��,�������ֵ����Сֵ,ʣ��8����ƽ��.
	Ad_MaxValue = 0;
	Ad_MinValue = 0x1000;
	Ad_Sum = 0;

	ADC_ConfigRegularChannel(ADC, channel, 1, ADC_SAMP_TIME_88CYCLES5);
	ADC_ClearFlag(ADC,ADC_FLAG_ENDC);
	ADC_ClearFlag(ADC,ADC_FLAG_STR);

	for (i=0; i<10; i++)
	{	
		/* Start ADC Software Conversion */
		ADC_EnableSoftwareStartConv(ADC,ENABLE);
		while(ADC_GetFlagStatus(ADC,ADC_FLAG_ENDC) == 0);
		ADC_ClearFlag(ADC,ADC_FLAG_ENDC);
		ADC_ClearFlag(ADC,ADC_FLAG_STR);
		
		value = ADC_GetDat(ADC);
		if (value > Ad_MaxValue)
		{
			Ad_MaxValue = value;
		}
		if (value < Ad_MinValue)
		{
			Ad_MinValue = value;
		}
		Ad_Sum += value;
	}
	Ad_Sum = Ad_Sum - Ad_MaxValue - Ad_MinValue;
    value = Ad_Sum/8;
		
    return value;
}

/*****************************************************************************
 �� �� ��  : ClkInit
 ��������  : ϵͳʱ�ӡ���׼ʱ��TIMER0��ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void ClkInit(void)
{
	/* Get SystemCoreClock */    
	SystemCoreClockUpdate();
	
	/* Config 1MS SysTick  */
	SysTick_Config(SystemCoreClock/1000);

	/* Enable peripheral clocks ------------------------------------------------*/
	/* TIM3 clock enable */    
//	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);
	
    /* Enable GPIO clocks */
    RCC_EnableAPB2PeriphClk( RCC_APB2_PERIPH_GPIOA\
    						|RCC_APB2_PERIPH_GPIOB\
    						|RCC_APB2_PERIPH_GPIOC\
    						|RCC_APB2_PERIPH_GPIOF\
    						|RCC_APB2_PERIPH_AFIO,  ENABLE );

	/* Enable PWR Clock */
	RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_PWR, ENABLE);
	DBG_ConfigPeriph(DBG_IWDG_STOP, ENABLE);

}

/*****************************************************************************
 �� �� ��  : PortInit
 ��������  : IO�����ó�ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void PortInit(void)
{	
	GPIO_InitType GPIO_InitStructure;
		
    GPIO_InitStruct(&GPIO_InitStructure);	
    
	/* Configure analog ------------------------*/
	
    /* Configure output -------------------------*/
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pin			  = CON_CTRL_Pin|DSG_CTRL_Pin;
	GPIO_InitPeripheral(CON_CTRL_Port, &GPIO_InitStructure); 

    /* Configure input -------------------------*/

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.Pin			  = UART1_TX_Pin;
	GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStructure.GPIO_Alternate = UART1_TX_AF;
	GPIO_InitPeripheral(UART1_TX_Port, &GPIO_InitStructure); 

	/* Configure USART Rx as alternate function push-pull */
	GPIO_InitStructure.Pin			  = UART1_RX_Pin;
	GPIO_InitStructure.GPIO_Alternate = UART1_RX_AF;
	GPIO_InitPeripheral(UART1_RX_Port, &GPIO_InitStructure);	 
}

/*****************************************************************************
 �� �� ��  : PwmSetFreq
 ��������  : PWM Ƶ������
 �������  : u16 nFre  
 �� �� ֵ  : 
*****************************************************************************/
void PwmSetFreq(u16 prescaler,u16 period)
{
	TIM_TimeBaseInitType TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.Period    = period;		//665;
    TIM_TimeBaseStructure.Prescaler = prescaler;			// PrescalerValue;
    TIM_TimeBaseStructure.ClkDiv    = 0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;

    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
}

/*****************************************************************************
 �� �� ��  : PwmSetDuty
 ��������  : PWM ռ�ձ�����
 �������  : u16 nDuty  
 �� �� ֵ  : 
*****************************************************************************/
void PwmSetDuty(u16 nDuty)
{     
	OCInitType TIM_OCInitStructure;
	/* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = nDuty;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;

    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    TIM_EnableOc1Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);
}

/*****************************************************************************
 �� �� ��  : PwmInit
 ��������  : PWM1��ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void PwmInit(u16 prescaler, u16 period, u16 dutyCycle)
{
	// TIM_TimeBaseInitType TIM_TimeBaseStructure;
	OCInitType TIM_OCInitStructure;
	uint16_t CCR1_Val       = dutyCycle/1;	//333;
	uint16_t CCR2_Val       = dutyCycle/2;	//249;
	uint16_t CCR3_Val       = dutyCycle/4;	//166;
	uint16_t CCR4_Val       = dutyCycle/8;	//83;
	GPIO_InitType GPIO_InitStructure;
		
    GPIO_InitStruct(&GPIO_InitStructure);	
	
	// for pwm Pin Configuration
	
	/* GPIOA Configuration:TIM3 Channel1, 2, 3 and 4 as alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_6;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOA, &GPIO_InitStructure);

    /* GPIOB Configuration:TIM3 Channel3 and 4 as alternate function push-pull */
    GPIO_InitStructure.Pin        = GPIO_PIN_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Current = GPIO_DC_LOW;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.Pin        = GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF2_TIM3;
    GPIO_InitPeripheral(GPIOB, &GPIO_InitStructure);
	
//	uint16_t PrescalerValue = 0;
	/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM3CLK frequency is set to SystemCoreClock (Hz), to get TIM3 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM3CLK / TIM3 counter clock) - 1
    SystemCoreClock is set to 48 MHz for N32G030 device

    The TIM3 is running at 36 KHz: TIM3 Frequency = TIM3 counter clock/(AR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%
    ----------------------------------------------------------------------- */
    /* Compute the prescaler value */
//    PrescalerValue = (uint16_t)(SystemCoreClock / 12000000) - 1;
    /* Time base configuration */
	PwmSetFreq(prescaler,period);
    PwmSetDuty(CCR1_Val);
    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode      = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR1_Val;
    TIM_OCInitStructure.OcPolarity  = TIM_OC_POLARITY_HIGH;

    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    TIM_EnableOc1Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);

    /* PWM1 Mode configuration: Channel2 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR2_Val;

    TIM_InitOc2(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc2Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);

    /* PWM1 Mode configuration: Channel3 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR3_Val;

    TIM_InitOc3(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc3Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);

    /* PWM1 Mode configuration: Channel4 */
    TIM_OCInitStructure.OutputState = TIM_OUTPUT_STATE_ENABLE;
    TIM_OCInitStructure.Pulse       = CCR4_Val;

    TIM_InitOc4(TIM3, &TIM_OCInitStructure);

    TIM_ConfigOc4Preload(TIM3, TIM_OC_PRE_LOAD_ENABLE);

    TIM_ConfigArPreload(TIM3, ENABLE);

    /* TIM3 enable counter */
    TIM_Enable(TIM3, ENABLE);

}

/*****************************************************************************
 �� �� ��  : Timer0Init
 ��������  : Timer0��ʼ����1ms��ʱ�� 
 			 16Mhz, DIV: 12
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void Timer0Init(void)
{ 
	;
}
/*****************************************************************************
 �� �� ��  : Timer1Init
 ��������  : Timer1��ʼ�� 
 			 Tick: 200us
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void Timer1Init(void)
{
	;
}

/*****************************************************************************
 �� �� ��  : Timer2Init
 ��������  : Timer2��ʼ��
 			 Tick: 1ms
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void Timer2Init(void)
{
    ;
}

/*****************************************************************************
 �� �� ��  : Timer3Init
 ��������  : Timer3��ʼ��
 			 Tick: 1ms
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void Timer3Init(void)
{
    TIM_TimeBaseInitType TIM_TimeBaseStructure;
    OCInitType TIM_OCInitStructure;
    NVIC_InitType NVIC_InitStructure;

    /* Enable TIM3 clocks */
    RCC_EnableAPB1PeriphClk(RCC_APB1_PERIPH_TIM3, ENABLE);

    /* TIM3 configuration */
    TIM_TimeBaseStructure.Period    = 0x4AF;
    TIM_TimeBaseStructure.Prescaler = ((SystemCoreClock / 1200) - 1);
    TIM_TimeBaseStructure.ClkDiv    = 0x0;
    TIM_TimeBaseStructure.CntMode   = TIM_CNT_MODE_UP;
    TIM_InitTimeBase(TIM3, &TIM_TimeBaseStructure);
    TIM_InitOcStruct(&TIM_OCInitStructure);

    /* Output Compare Timing Mode configuration: Channel1 */
    TIM_OCInitStructure.OcMode = TIM_OCMODE_TIMING;
    TIM_OCInitStructure.Pulse  = 0x0;
    TIM_InitOc1(TIM3, &TIM_OCInitStructure);

    /* Immediate load of TIM3 Precaler values */
    TIM_ConfigPrescaler(TIM3, ((SystemCoreClock / 1200) - 1), TIM_PSC_RELOAD_MODE_IMMEDIATE);

    /* Clear TIM3 update pending flags */
    TIM_ClearFlag(TIM3, TIM_FLAG_UPDATE);

    /* Enable the TIM3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel                   = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable TIM3 Update interrupts */
    TIM_ConfigInt(TIM3, TIM_INT_UPDATE, ENABLE);

    /* TIM3 enable counters */
    TIM_Enable(TIM3, ENABLE);
}

/*****************************************************************************
 �� �� ��  : I2CInit
 ��������  : I2C��ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
static void I2CInit(void)
{
//	I2C_SoftInit();
}

/*****************************************************************************
 �� �� ��  : UartInit
 ��������  : Uart��ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void UartOpen(USART_Module* USARTn, u32 BaudRate)
{
	USART_InitType USART_InitStructure;

	 /* USART configuration ------------------------------------------------------*/
	USART_InitStructure.BaudRate			= BaudRate;
	USART_InitStructure.WordLength			= USART_WL_8B;
	USART_InitStructure.StopBits			= USART_STPB_1;
	USART_InitStructure.Parity				= USART_PE_NO;
	USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
	USART_InitStructure.Mode				= USART_MODE_RX | USART_MODE_TX;

	/* Configure USART */
	USART_Init(USARTn, &USART_InitStructure);

	/* Enable USART Receive and Transmit interrupts */
	USART_ConfigInt(USARTn, USART_INT_RXDNE, ENABLE);
//	USART_ConfigInt(USARTn, USART_INT_TXC, ENABLE);

	/* Enable the USART */
	USART_Enable(USARTn, ENABLE);
}

void UartInit(void)
{  
	RCC_EnableAPB2PeriphClk(RCC_APB2_PERIPH_USART1, ENABLE);
	UartOpen(USART1, 9600);
}

/*****************************************************************************
 �� �� ��  : UartDeInit
 ��������  : Uart�ָ�Ĭ�ϳ�ʼ��ֵ
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void UartDeInit(void)
{
	;
}

/*****************************************************************************
 �� �� ��  : UartRxDisable
 ��������  : ����ͨѶʱ��RX��ֹ
 �������  :   
 �� �� ֵ  : 
*****************************************************************************/
void UartRxDisable(void)
{
//	USART_ConfigInt(USART2, USART_INT_RXDNE, DISABLE);
//	Delay10us(10);
}

void UartRxEnable(void)
{
//	while (USART_GetFlagStatus(USART2, USART_FLAG_TXC) == RESET);
//	USART_ConfigInt(USART2, USART_INT_RXDNE, ENABLE);
}

/*****************************************************************************
 �� �� ��  : UartSendByte
 ��������  : Uart ����
 �������  : u8 tx_data  
 �� �� ֵ  : 
*****************************************************************************/
void UartSendByte(u8 tx_data) 
{  
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) != RESET);
	USART_SendData(USART1, tx_data);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXDE) == RESET);
}

/*****************************************************************************
 �� �� ��  : UartSendByteIT
 ��������  : Uart ���� --> ���ȴ�������ɱ�־
 �������  : u8 tx_data  
 �� �� ֵ  : 
*****************************************************************************/
void UartSendByteIT(u8 tx_data) 
{  
	USART_SendData(USART1, tx_data);
}

void UartSendBuffer(u8 *pBuf, u8 length)
{
	while(length != 0)
	{
		UartSendByte(*pBuf++);
		length--;
	}
}

/*****************************************************************************
 �� �� ��  : UartReceiveByte
 ��������  : Uart ����
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
u8 UartReceiveByte(void)
{
	return USART_ReceiveData(USART1);   
}

/*****************************************************************************
 �� �� ��  : HwIntDisable
 ��������  : �жϹر�
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwIntDisable(void)
{
	// disable interrupts
	__disable_irq();
}

/*****************************************************************************
 �� �� ��  : HwIntEnable
 ��������  : �жϿ���
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwIntEnable(void)
{
	__enable_irq();
}

/*****************************************************************************
 �� �� ��  : HwMcuReset
 ��������  : MCU�����λ
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwMcuReset(void)
{
	__disable_irq();
	NVIC_SystemReset();
}

/*****************************************************************************
 �� �� ��  : IntPriorityInit
 ��������  : �ж����ȼ�����
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwInterruptConfig(void)
{
	NVIC_InitType NVIC_InitStructure;   

	/*Set systick interrupt priority*/
    NVIC_InitStructure.NVIC_IRQChannel                   = SysTick_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	
	/* Enable the USART2 Interrupt */    
	NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;    
	NVIC_InitStructure.NVIC_IRQChannelPriority           = 1;    
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;    
	NVIC_Init(&NVIC_InitStructure);    
}

/*****************************************************************************
 �� �� ��  : HwDataInit
 ��������  : ���ݼ�¼��ʼ��
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwDataInit(void)
{    

}

/*****************************************************************************
 �� �� ��  : HwDataUnlock
 ��������  : д����ǰ��ִ�н���
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwDataUnlock(void)
{ 
	FLASH_Unlock();	
	__disable_irq();
}

/*****************************************************************************
 �� �� ��  : HwDataLock
 ��������  : д������ɺ���ִ����������
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwDataLock(void)
{ 
	FLASH_Lock();	
	__enable_irq();
}

u8 HwDataRead(u32 Addr, u32 *pData, u16 Len)
{
	u16 i;
	volatile u32* AddrPt;

	if ((Addr&0x0000003) != 0)
	{
		return ERROR;				// request address align 4
	}

	if ((Addr+Len*4) > FLASH_DATA_SIZE)
	{
		return ERROR;				// request data area right
	}

	AddrPt = (u32 *)(FLASH_DATA_BASE_ADDR + Addr);
	for (i=0; i<Len; i++)
	{
		*pData = *AddrPt;
		pData++;
		AddrPt++;
	}

	return SUCCESS;
}

u8 HwDataReadRandom(u32 Addr, u32 *pData, u16 Len)
{
	u16 i;
	volatile u32* AddrPt;

	if ((Addr&0x0000003) != 0)
	{
		return ERROR;				// request address align 4
	}

	AddrPt = (u32 *)Addr;

	for (i=0; i<Len; i++)
	{
		*pData = *AddrPt;
		pData++;
		AddrPt++;
	}

	return SUCCESS;
}

u8 HwDataPageWrite(u32 Addr, u32 *pData, u16 Len)
{
	u8 ret, trycnt;
	u16 i;	
	u32 ProAddr;

	/* check input info */
	if ((Addr&0x00001FF) != 0)
	{
		return ERROR;				// request address if page first address
	}

	if ((Addr+Len*4) > FLASH_DATA_SIZE)
	{
		return ERROR;				// request data area right
	}

	ProAddr = FLASH_DATA_BASE_ADDR + Addr;

	/* do erase */
	trycnt = 0;
	do
	{
		ret = FLASH_EraseOnePage(ProAddr);
	}
	while((ret!=FLASH_COMPL) && (trycnt++<5));
	
	if (ret != FLASH_COMPL)
		return ERROR;

	/* do program */
	for (i=0; i<Len; i++)
	{
		trycnt = 0;
		do
		{
			ret = FLASH_ProgramWord(ProAddr, *pData);
			
			__NOP();__NOP();__NOP();__NOP();
			
			if (*(__IO u32*)ProAddr != *pData)
			{
				ret = FLASH_ERR_STS;
			}
		}while((ret!=FLASH_COMPL) && (trycnt++<5));
		
		if (ret != FLASH_COMPL)
			return ERROR;
		
		ProAddr += 4;
		pData++;
	}
	
	return SUCCESS;
}


u8 HwDataPageWriteRandom(u32 Addr, u32 *pData, u16 Len)
{
	u8 ret, trycnt;
	u16 i;	
	u32 ProAddr;

	/* check input info */
	if ((Addr&0x00001FF) != 0)
	{
		return ERROR;				// request address if page first address
	}

	ProAddr = Addr;

	/* do erase */
	trycnt = 0;
	do
	{
		ret = FLASH_EraseOnePage(ProAddr);
	}
	while((ret!=FLASH_COMPL) && (trycnt++<5));
	
	if (ret != FLASH_COMPL)
		return ERROR;

	/* do program */
	for (i=0; i<Len; i++)
	{
		trycnt = 0;
		do
		{
			ret = FLASH_ProgramWord(ProAddr, *pData);
		}while((ret!=FLASH_COMPL) && (trycnt++<5));
		
		if (ret != FLASH_COMPL)
			return ERROR;
		
		ProAddr += 4;
		pData++;
	}
	
	return SUCCESS;
}

/*****************************************************************************
 �� �� ��  : HwMcuInit
 ��������  : MCUӲ��ģ���ʼ�������
 �������  : void  
 �� �� ֵ  : 
*****************************************************************************/
void HwMcuInit(void)
{
	HwIntDisable();
	ClkInit();							
	WdtInit();							
	PortInit();								
	UartInit();
	HwInterruptConfig();
	HwIntEnable();	
}


//#define SOFTWARE_IIC
#ifdef SOFTWARE_IIC
#define I2C1_SCL_PORT               GPIOB
#define I2C1_SCL_PIN                GPIO_PIN_13

#define I2C1_SDA_PORT               GPIOB
#define I2C1_SDA_PIN                GPIO_PIN_14

#define SCL_H                       {GPIO_SetBits(I2C1_SCL_PORT, I2C1_SCL_PIN);}
#define SCL_L                       {GPIO_ResetBits(I2C1_SCL_PORT, I2C1_SCL_PIN);}
#define SCL_read                    (GPIO_ReadInputDataBit(I2C1_SCL_PORT, I2C1_SCL_PIN))

#define	SDA_H	                    {GPIO_SetBits(I2C1_SDA_PORT, I2C1_SDA_PIN);}
#define SDA_L	                    {GPIO_ResetBits(I2C1_SDA_PORT, I2C1_SDA_PIN);}
#define SDA_Read                    (GPIO_ReadInputDataBit(I2C1_SDA_PORT, I2C1_SDA_PIN))

#define I2C_PageSize 16

void I2C_SoftInit(void)
{
	GPIO_InitType GPIO_InitStructure;

    GPIO_InitStruct(&GPIO_InitStructure);

	// for gpio
	
    /* Configure PA.01 as analog input -------------------------*/
    GPIO_InitStructure.Pin       = I2C1_SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitPeripheral(I2C1_SCL_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin       = I2C1_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitPeripheral(I2C1_SDA_PORT, &GPIO_InitStructure);	
}
void I2C_delay(void)
{
    //set I2C SCL speed ����ͨ���ٶ�
    u8 i = 13; //100;

    while(i) 
	{
        i--;
    }
}

u8 I2C_Start(void)
{
    I2C_delay();
    SDA_H;
    SCL_H;
    I2C_delay();
    if(!SDA_Read) 
	{
		return 0;  //SDA��Ϊ�͵�ƽ����æ���˳�
    }

    SDA_L;
    I2C_delay();
    if(SDA_Read) 
	{
		return 0; //SDA��Ϊ�ߵ�ƽ�����߳����˳�
    }

    SDA_L;
    I2C_delay();
    return 1;
}

void I2C_Stop(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SDA_H;
    I2C_delay();
}

void I2C_Ack(void)
{
    SCL_L;
    I2C_delay();
    SDA_L;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

void I2C_NoAck(void)
{
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    SCL_L;
    I2C_delay();
}

u8 I2C_WaitAck(void)  //����Ϊ��-1��ACK�� =0 ��ACK
{
    bool bstatus;
    SCL_L;
    I2C_delay();
    SDA_H;
    I2C_delay();
    SCL_H;
    I2C_delay();
    if(SDA_Read) 
	{
        bstatus = 0;
    } 
	else 
	{
        bstatus = 1;
    }
    SCL_L;
    return bstatus;
}

void I2C_SendByte(u8 SendByte) //���ݴӸ�λ����λ
{
    u8 i = 8;
    while(i--) 
	{
        SCL_L;
        I2C_delay();
        if(SendByte & 0x80)
        {
            SDA_H;
        }
        else
        {
            SDA_L;
        }
        SendByte <<= 1;
        I2C_delay();
        SCL_H;
        I2C_delay();

    }
    SCL_L;
}

u8 I2C_ReceiveByte(void) //���ݴӸ�λ����λ
{
    u8 i = 8;
    u8 ReceiveByte = 0;

    SDA_H;
    while(i--) 
	{
        ReceiveByte <<= 1;
        SCL_L;
        I2C_delay();
        SCL_H;
        I2C_delay();
        if(SDA_Read) 
		{
            ReceiveByte |= 0x01;
        }
    }
    SCL_L;
    return ReceiveByte;
}

//д��1�ֽ����� ����д�����ݣ���д���ַ���������ͣ�
u8 I2C_ByteWrite(u8 DeviceAddress,u16 WriteAddress,u8 SendByte)
{
    if(!I2C_Start()) return 0;

    I2C_SendByte(((WriteAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE));	//���ø���ʼ��ַ + ������ַ

    if(!I2C_WaitAck()) 
	{
        I2C_Stop();
        return 0;
    }
    I2C_SendByte((u8)(WriteAddress & 0x00FF)); //���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_SendByte(SendByte);
    I2C_WaitAck();
    I2C_Stop();
    //ע�⣺��Ϊ����Ҫ�ȴ�EERPOMд��ɣ����Բ��ò�ѯ ����ʱ��ʽ��10ms��
    delay10us((u32)10*100);
    return 1;
}

//ע�ⲻ�ܿ�ҳд
//д��1�����ݣ���д�������ַ����д�볤�ȣ���д���ַ���������ͣ�
bool I2C_PageWrite(u8 DeviceAddress,u16 WriteAddress,u8 *pBuffer, u8 length)
{
    if((length + WriteAddress % I2C_PageSize) > I2C_PageSize) 
	{
		return 0;
    }
    if(!I2C_Start()) 
	{
		return 0;
	}
    I2C_SendByte(((WriteAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE));	//���ø���ʼ��ַ + ������ַ

    if(!I2C_WaitAck()) 
	{
        I2C_Stop();
        return 0;
    }
    I2C_SendByte((u8)(WriteAddress & 0x00FF)); //���õ���ʼ��ַ
    I2C_WaitAck();

    while(length--) 
	{
        I2C_SendByte(*pBuffer);
        I2C_WaitAck();
        pBuffer++;
    }
    I2C_Stop();
    //ע�⣺��Ϊ����Ҫ�ȴ�EERPOMд��ɣ����Բ��ò�ѯ ����ʱ��ʽ��10ms��
    delay10us((u32)10*100);
    return 1;
}

//��ҳд��1�����ݣ���д�������ַ����д�볤�ȣ���д���ַ���������ͣ�
void I2C_BufferWrite(u8 DeviceAddress,u16 WriteAddress,u8 *pBuffer, u16 length)
{
    u16 i;
    u16 Addr = 0, count = 0;
    Addr = WriteAddress % I2C_PageSize; //д���ַ�ǿ�ʼҳ�ĵڼ�ҳ
    count = I2C_PageSize - Addr; //�ڿ�ʼҳҪд��ĸ���
    if(length <= count) 
	{
        I2C_PageWrite(DeviceAddress,WriteAddress,pBuffer, length); //��дһҳ������
    } 
	else 
	{
        I2C_PageWrite(DeviceAddress,WriteAddress,pBuffer, count); //��д��һҳ������
        if((length - count) <= I2C_PageSize) 
		{
            I2C_PageWrite(DeviceAddress,WriteAddress + count,pBuffer + count, length - count); //����дһҳ�����ݽ���
        } 
		else 
		{
            for(i = 0; i < ((length - count) / I2C_PageSize); i++) 
			{
                I2C_PageWrite(DeviceAddress,WriteAddress + count + i * I2C_PageSize,pBuffer + count + i * I2C_PageSize, I2C_PageSize);
            }
            if( ((length - count) % I2C_PageSize) != 0 ) 
			{
                I2C_PageWrite(DeviceAddress,WriteAddress + count + i * I2C_PageSize,pBuffer + count + i * I2C_PageSize, ((length - count) % I2C_PageSize));
            }
        }
    }
}

//����1�����ݣ���Ŷ������ݣ����������ȣ���������ַ���������ͣ�
u8 I2C_ReadBuffer(u8 DeviceAddress,u16 ReadAddress,u8 *pBuffer, u8 length)
{
    if(!I2C_Start())
	{
		return 0;
    }
    I2C_SendByte(((ReadAddress & 0x0700) >> 7) | (DeviceAddress & 0xFE)); //���ø���ʼ��ַ+������ַ
    if(!I2C_WaitAck()) 
	{
        I2C_Stop();
        return 0;
    }
    I2C_SendByte((u8)(ReadAddress & 0x00FF));//���õ���ʼ��ַ
    I2C_WaitAck();
    I2C_Start();
    I2C_SendByte(((ReadAddress & 0x0700) >> 7) | (DeviceAddress | 0x0001));
    I2C_WaitAck();
    while(length) 
	{
        *pBuffer = I2C_ReceiveByte();
        if(length == 1) 
		{
			I2C_NoAck();
		}
        else 
		{
			I2C_Ack();
		}
        pBuffer++;
        length--;
    }
    I2C_Stop();
    return 1;
}


#endif

