#include <stdio.h>
#include <string.h>
#include "main.h"
#include "bootload.h"
#include "sysDefs.h"


/*******************************************************************************/
#define N32_M0_VCTOR_REG		((volatile uint32_t*)0x40024C30)

#define FLASH_BASE_ADDR		0x08000000
#define PAGE_SIZE 			512
#define SECTOR_SIZE			PAGE_SIZE

#define BOOT_MARK_ADDR		(FLASH_BASE_ADDR + 1024*(32-1) + PAGE_SIZE)
#define PAGE_ADDR_MASK		(FLASH_BASE_ADDR + 1024*32 - PAGE_SIZE)	 			//0x08007E00
#define FLASH_BOOT_WORD		0xAA55F813

#define FLASH_APP_ADDR		(FLASH_BASE_ADDR+0x1400)							// set as first addr of one page



typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
static uint32_t JumpAddress;

/*******************************************************************************/
BootFileMessageStruct BootFileMessage;
ReceiveFileMessageStruct ReceiveFileMessage;

volatile uint16_t receiveTimeOut;
uint16_t receiveCount;
uint16_t BootWaitDelay;
uint8_t eusartInfo[20];
uint8_t eusartBuf[EUSART_BUFFER_SIZE];
volatile uint8_t RxFlag;																			// UART���յ���Ч����ͷ��ʶ
volatile uint8_t UartCmdActFlag;	
uint8_t UartReplyDelay;


/*******************************************************************************/
void Bootload_WriteUpgradeMark(uint8_t mark)
{
	uint32_t boot_info[PAGE_SIZE/4];
	
	HwDataReadRandom((uint32_t)BOOT_MARK_ADDR&PAGE_ADDR_MASK, boot_info, PAGE_SIZE/4);

	if(mark)
	{
		boot_info[0] = FLASH_BOOT_WORD;
	}
	else
	{
		boot_info[0] = 0xFFFFFFFF;
	}
	HwDataUnlock();
	HwDataPageWriteRandom(BOOT_MARK_ADDR&PAGE_ADDR_MASK, boot_info, PAGE_SIZE/4);
	HwDataLock();
}

/*********default write 1024bytes, need reserve enough space for tail programer********/
void Bootload_WriteSector(uint32_t addr, uint8_t *pBuf)
{
	uint32_t writeBuf[PAGE_SIZE/4*2];

	memcpy((uint8_t *)writeBuf, pBuf, PAGE_SIZE*2);

	HwDataUnlock();
	HwDataPageWriteRandom(addr&PAGE_ADDR_MASK,             writeBuf,               PAGE_SIZE/4);
	HwDataPageWriteRandom((addr+PAGE_SIZE)&PAGE_ADDR_MASK, &writeBuf[PAGE_SIZE/4], PAGE_SIZE/4);
	HwDataLock();
}

/*******************************************************************************/
void Bootload_ReceiveByte(uint8_t rxByte)
{
	static uint16_t Data_Len = 0;

	if(UartCmdActFlag)
	{
		return;
	}
	if((RxFlag == 0) && (rxByte == 0xF1))							
	{
		RxFlag = 1;
		receiveCount = 0;	
		receiveTimeOut = 2000;// 2s֮�����û���ٴν��յ�ͷ��ǿ��������ձ�־
		Data_Len = 0;
	}
	if(RxFlag)
	{
		if(receiveCount == (sizeof(eusartBuf)-1))
		{
			receiveCount = 0;
		}
		if(receiveCount < 4)									// ���͵�ַ/���յ�ַ/������/���ݳ���
		{
			eusartBuf[receiveCount] = rxByte;
			receiveCount++;
		}
		else if(receiveCount == 4)										// ���ݳ���
		{
			eusartBuf[receiveCount] = rxByte;
			Data_Len = eusartBuf[3]*256+eusartBuf[4];
			receiveCount++;
		}
		else if(receiveCount < (7+Data_Len))					// ������Data��CRC
		{
			eusartBuf[receiveCount] = rxByte;
			receiveCount++;
		}
		else if(receiveCount == (7+Data_Len))
		{
		  if(rxByte == 0xF2)
			{
				UartCmdActFlag = 1;
		 	}
			eusartBuf[receiveCount] = rxByte;
		  receiveCount++;
		}
		else
		{
			if(!UartCmdActFlag)							// ��ֹ��������������ɺ�,δ����ͱ���� Neiyang->2018.05.09
			{
				RxFlag = 0;
				receiveCount = 0;
			}
		}
	}
}

// *****************************************************************************
void Bootload_Initialize(void)
{
	uint32_t value;

	Delay10us(10);
	value = *(volatile uint32_t *)(BOOT_MARK_ADDR);
	if (value != FLASH_BOOT_WORD)
	{
		Bootload_Required();
	}
}

void Bootload_Required(void)
{
	JumpAddress = *(volatile uint32_t *)FLASH_APP_ADDR;

	/* Judge whether the top of stack address is legal or not */
    if ((JumpAddress & 0x2FFE0000) == 0x20000000)
	{
		__disable_irq();
		
		/* Vector table offset */
		*N32_M0_VCTOR_REG = 0x88000000;										// enable offset
		*N32_M0_VCTOR_REG |= (FLASH_APP_ADDR-FLASH_BASE_ADDR);				// set offset

		Jump_To_Application = (pFunction) *(__IO uint32_t*) (FLASH_APP_ADDR + 4);		

		/* Jump to user application */
		__set_MSP(JumpAddress);		
		Jump_To_Application();
	}
}

void Bootload_Schedule(void)
{
	if (BootWaitDelay > 40000) 			// 40000*1ms 40s��ʱ�������򱨴�  
	{
		//Bootload_Initialize();				// ��ʱ����Ƿ����תapp	
		DIS_MOS_OFF();
		POWER_CON_OFF();
		BootWaitDelay = 0;
		ReceiveFileMessage.StartMark = 0;
	}
	if (UartCmdActFlag == 0)	// about 5 ms delay
	{
		return;
	}
	if(UartReplyDelay < 50)//��ʱ����΢��
	{
		return;
	}
	//��ʱ��������ݴ���
	if (receiveCount >= 8)
	{
		Bootload_Decode(eusartBuf);
	//	BootWaitDelay = 0;
	}
	receiveCount = 0;
	UartCmdActFlag = 0;
	RxFlag = 0;
	UartReplyDelay = 0;
}

uint16_t CRC16_ccitt(uint8_t *pBuf, uint16_t length)  
{  
	uint16_t crc_init = 0x0000;  
	uint16_t crc_poly = 0x1021;  
	uint8_t crc_byte;
	uint8_t i;

	while (length--)     
	{  
		crc_byte = *pBuf++;  
		crc_init ^= (crc_byte << 8);  
		for(i = 0; i < 8; i++)  
		{  
			if(crc_init & 0x8000)  
				crc_init = (crc_init << 1) ^ crc_poly;  
			else  
				crc_init = crc_init << 1;  
		}  
	}  
	return (crc_init) ;  
}  

uint16_t CRC16_CheckMemory(uint32_t checkAddr, uint32_t length)
{
	uint16_t crc_value = 0x0000;  
	uint16_t crc_poly = 0x1021;  
	uint16_t crc_byte;
	uint8_t i, *pos;

	pos = (uint8_t *)checkAddr;
	while (length--)     
	{  
		crc_byte = *pos++;  
		crc_value ^= (crc_byte << 8);  
		for(i = 0; i < 8; i++)  
		{  
			if(crc_value & 0x8000)  
				crc_value = (crc_value << 1) ^ crc_poly;  
			else  
				crc_value = crc_value << 1;  
		}  
	}  
	return (crc_value) ;  
}

void Bootload_Decode(uint8_t *pBuf)
{
	uint32_t write_addr;
	uint16_t offset;
	uint16_t length;
	uint16_t crc, crc_calc;
    uint16_t frame_num;
	uint16_t i;
	uint8_t cmd;
	uint8_t result = 0;

	for (offset = 0; offset < 10; offset++)
	{
		if (*(pBuf + offset) == FRAMEFIRST)
		{
			break;
		}
	}
	if (offset < 10)
	{
		if (*((pBuf + offset) + 1) != IAPADDR)	// ��ַ�ж�
		{
			return;
		}
		cmd = *((pBuf + offset) + 2);
		if (cmd != CMD70 && cmd != CMD71 && cmd != CMD72)	// �������ж�
		{
			return;
		}
		length = *((pBuf + offset) + 3);
		length <<= 8;
		length |= *((pBuf + offset) + 4);	// ���ݳ��ȺϷ����ж�
		if ((length + 8) > receiveCount)	// �����ļ����Ȳ��Ϸ�
		{
			return;
		}
		else if ((cmd == CMD70 || cmd == CMD72) && length != 14)
		{
			return;
		}
		else if (cmd == CMD71 && length - 2 > BootFileMessage.FrameLen)
		{
			return;
		}
		crc = *((pBuf + offset) + 5 + length);
		crc <<= 8;
		crc |= *((pBuf + offset) + 6 + length); // crcУ��
		crc_calc = CRC16_ccitt((pBuf + offset) + 1, length + 4);
		if (*((pBuf + offset) + 7 + length) != FRAMEEND) //֡β
		{
			return;
		}
		else if (crc != 0x55AA) //֡У�����
		{
			if (crc != crc_calc)
			{
				result = 1;
			}
		}
		// ����CMD����֡����
		if (result == 0)
		{
			switch (cmd)
			{
			case CMD70:	// �ļ���Ϣ֡
				for (i = 0; i < INFO_LENGTH; i++)
				{
					eusartInfo[i] = *((pBuf + offset) + 5 + i);
				}
				BootFileMessage.FileSize = ((uint32_t)(*((pBuf + offset) + 5 + 0)) << 24)
					+ ((uint32_t)(*((pBuf + offset) + 5 + 1)) << 16)
					+ ((uint32_t)(*((pBuf + offset) + 5 + 2)) << 8)
					+ (uint32_t)(*((pBuf + offset) + 5 + 3));
				BootFileMessage.FrameLen = ((uint16_t)(*((pBuf + offset) + 5 + 4)) << 8)
					+ (uint16_t)(*((pBuf + offset) + 5 + 5));
				BootFileMessage.FrameNum = ((uint16_t)(*((pBuf + offset) + 5 + 6)) << 8)
					+ (uint16_t)(*((pBuf + offset) + 5 + 7));
				BootFileMessage.FileCrc = ((uint16_t)(*((pBuf + offset) + 5 + 8)) << 8)
					+ (uint16_t)(*((pBuf + offset) + 5 + 9));
				BootFileMessage.SoftVer = ((uint32_t)(*((pBuf + offset) + 5 + 10)) << 24)
					+ ((uint32_t)(*((pBuf + offset) + 5 + 11)) << 16)
					+ ((uint32_t)(*((pBuf + offset) + 5 + 12)) << 8)
					+ (uint32_t)(*((pBuf + offset) + 5 + 13));
				write_addr = (uint32_t)BootFileMessage.FrameLen*BootFileMessage.FrameNum;
				if (BootFileMessage.FileSize > FILESIZE		// �ļ���Ϣ���Ϸ�
					|| BootFileMessage.FrameLen > FRAMELEN || BootFileMessage.FileSize > write_addr)
				{
					result = 2;
				}
				else if (BootFileMessage.SoftVer != SOFTVERSION)	// �汾��һ��
				{
					result = 4;
				}
				else if (result == 0)
				{
					ReceiveFileMessage.FileSize = 0;
					ReceiveFileMessage.FrameNum = 1;
					ReceiveFileMessage.StartMark = 1;
				}
				break;
			case CMD71://�ļ�����֡
				frame_num = *((pBuf + offset) + 5);
				frame_num <<= 8;
				frame_num |= *((pBuf + offset) + 6);
				if (frame_num > ReceiveFileMessage.FrameNum || frame_num > BootFileMessage.FrameNum)//֡û�еݼӻ��������֡�����򱨴�
				{
					result = 2;
				}
				else if (ReceiveFileMessage.StartMark == 0) //��Ҫ���յ�70֡
				{
					result = 0x0A;
				}
				else
				{
					if (frame_num == ReceiveFileMessage.FrameNum) //�ط��Ĳ�����¼
					{
						ReceiveFileMessage.FrameNum += 1;
						ReceiveFileMessage.FileSize += length - 2;
						write_addr = (uint32_t)FLASH_APP_ADDR + (uint32_t)(frame_num - 1)*PACKAGE_LENGTH;
						//���ñ�̴��� ��� ((pBuf+offset)+7)��length
						Bootload_WriteSector(write_addr, ((pBuf + offset) + 7));
					}
					if (frame_num == 1)
					{
						Bootload_WriteUpgradeMark(1);	// ���������б�־
					}
				}
				break;
			case CMD72://����֡
				for (i = 0; i < INFO_LENGTH; i++)
				{
					if (eusartInfo[i] != *((pBuf + offset) + 5 + i))
					{
						break;
					}
				}
				if (i < INFO_LENGTH)	// �ļ���Ϣ��һ��
				{
					result = 0x0A;
				}
				else if (BootFileMessage.FileSize != ReceiveFileMessage.FileSize)	// �ļ�������
//					|| BootFileMessage.FrameNum + 1 != ReceiveFileMessage.FrameNum	// when file size is 256, error framNum
				{
					result = 0x02;
				}
				else if (result == 0)
				{
					//������CRCУ��
					crc_calc = CRC16_CheckMemory(FLASH_APP_ADDR, ReceiveFileMessage.FileSize);
					if (crc_calc != BootFileMessage.FileCrc)
					{
						result = 3; //�ļ�CRCУ�����
					}
					ReceiveFileMessage.FileSize = 0;
					ReceiveFileMessage.FrameNum = 1;
					ReceiveFileMessage.StartMark = 0;
				}
				break;
			}
		}
		//Ӧ����
		*(pBuf + 0) = FRAMEEND;
		*(pBuf + 1) = MYBOOTADDR;
		*(pBuf + 2) = cmd;
		if (result == 0)
		{
			switch (cmd)
			{
			case CMD70:
				*(pBuf + 3) = 0;
				*(pBuf + 4) = 4; //length
				*(pBuf + 5) = (uint8_t)(SOFTVERSION >> 24);
				*(pBuf + 6) = (uint8_t)(SOFTVERSION >> 16);
				*(pBuf + 7) = (uint8_t)(SOFTVERSION >> 8);
				*(pBuf + 8) = (uint8_t)(SOFTVERSION & 0xFF); //CURRENT SOFTVERSION
				//crc����
				crc = CRC16_ccitt((pBuf) + 1, 8);
				//crc = 0x55aa;
				*(pBuf + 9) = (uint8_t)(crc >> 8);
				*(pBuf + 10) = (uint8_t)(crc & 0xFF);
				*(pBuf + 11) = FRAMEFIRST;
				length = 12;
				break;
			case CMD71:
				*(pBuf + 3) = 0;
				*(pBuf + 4) = 0; //length 
				//crc����
				crc = CRC16_ccitt((pBuf) + 1, 4);
				//crc = 0x55aa;
				*(pBuf + 5) = (uint8_t)(crc >> 8);
				*(pBuf + 6) = (uint8_t)(crc & 0xFF);
				*(pBuf + 7) = FRAMEFIRST;
				length = 8;
				break;
			case CMD72:
				*(pBuf + 3) = 0;
				*(pBuf + 4) = 4; //length
				*(pBuf + 5) = (uint8_t)(BootFileMessage.SoftVer >> 24);
				*(pBuf + 6) = (uint8_t)(BootFileMessage.SoftVer >> 16);
				*(pBuf + 7) = (uint8_t)(BootFileMessage.SoftVer >> 8);
				*(pBuf + 8) = (uint8_t)(BootFileMessage.SoftVer & 0xFF); //CURRENT SOFTVERSION
				//crc����
				crc = CRC16_ccitt((pBuf) + 1, 8);
				//crc = 0x55aa;
				*(pBuf + 9) = (uint8_t)(crc >> 8);
				*(pBuf + 10) = (uint8_t)(crc & 0xFF);
				*(pBuf + 11) = FRAMEFIRST;
				length = 12;
				break;
			}
		}
		else //����Ӧ��
		{
			*(pBuf + 2) &= 0X0F;
			*(pBuf + 2) |= 0XA0;
			*(pBuf + 3) = 0;
			*(pBuf + 4) = 1; //length 
			*(pBuf + 5) = result;
			//crc����
			crc = CRC16_ccitt((pBuf) + 1, 5);
			//crc = 0x55aa;
			*(pBuf + 6) = (uint8_t)(crc >> 8);
			*(pBuf + 7) = (uint8_t)(crc & 0xFF);
			*(pBuf + 8) = FRAMEFIRST;
			length = 9;
		}
		UartRxDisable();
		UartSendBuffer(pBuf, length);
		Delay10us(200);
		UartRxEnable();
		if (cmd == CMD72 && result == 0)//��ת��Ӧ�ó���
		{
			Bootload_WriteUpgradeMark(0);		// ���boot��־����
			Delay10us(250);
			HwMcuReset();
		}		
	}
	
	BootWaitDelay = 0;
}

