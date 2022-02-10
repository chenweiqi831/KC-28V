#include "main.h"


void SysTick_Handler(void)
{
	if( receiveTimeOut )
	{
		receiveTimeOut -= 1;
	}
	else
	{
		RxFlag = 0;
  }
	BootWaitDelay++;
	if(UartCmdActFlag)
	{
		UartReplyDelay++;
	}
}
	
void USART1_IRQHandler(void)
{	
	if (USART_GetIntStatus(USART1, USART_INT_RXDNE) != RESET)
	{
		Bootload_ReceiveByte(USART_ReceiveData(USART1));		// read rxdata will clear RXDNE flag   
	}

	if (USART_GetIntStatus(USART1, USART_INT_TXC) != RESET)
	{
		USART_ClrFlag(USART1, USART_FLAG_TXC);
	}
}

int main(void)
{
	Bootload_Initialize();			// jump to application while no BootMark  
	
	HwMcuInit();

	POWER_CON_ON();					// con set for Vdd(5V)
	DIS_MOS_ON();					// mos set for GND(communication)

	while( 1 )
	{
		HwClrWdt();
		Bootload_Schedule();
	}
}

