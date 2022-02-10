#ifndef __BOOTLOAD_H__
#define __BOOTLOAD_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */

#include "sysDefs.h" 


#define EUSART_BUFFER_SIZE		2000	// > 1024 + 8

#define FILESIZE		((32-5-1)*1024)
#define FRAMELEN		(EUSART_BUFFER_SIZE - 8)	// ÿ֡����
#define SOFTVERSION		0x00000000UL				// ����汾

#define INFO_LENGTH		14
#define PACKAGE_LENGTH	1024

#define MYBOOTADDR		0x01	// �豸��ַ
#define FRAMEFIRST		0xF1	// ֡ͷ
#define FRAMEEND		0xF2	// ֡β
#define IAPADDR			0xFA	// IAP��ַ
#define CMD70			0x70	// �ļ���Ϣ������
#define CMD71			0x71	// �ļ�֡������
#define CMD72			0x72	// �������������

typedef struct  
{
	uint32_t FileSize;	// �ļ�����
	uint16_t FrameLen;	// ֡����
	uint16_t FrameNum;	// ֡����
	uint16_t FileCrc;	// �ļ�CRC
	uint32_t SoftVer;	// ����汾
}BootFileMessageStruct;	// ��Ҫ���յĳ����ļ���Ϣ

typedef struct
{
	uint32_t FileSize;		// ���ܵ��ļ�����
	uint16_t FrameNum;		// ���յ��ļ�֡����
	uint16_t  StartMark;	// �������ձ�־
}ReceiveFileMessageStruct;	// ���ܹ������ļ��ۼ���Ϣ

extern volatile uint16_t receiveTimeOut;
extern volatile uint8_t UartCmdActFlag;
extern volatile uint8_t RxFlag;
extern uint16_t BootWaitDelay;
extern uint8_t UartReplyDelay;

void Bootload_Initialize(void);
void Bootload_Required(void);
void Bootload_ReceiveByte(uint8_t rxByte);
void Bootload_Schedule(void);
void Bootload_Decode(uint8_t *pBuf);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


#endif /* __BOOTLOADER_H__ */
