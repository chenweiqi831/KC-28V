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
#define FRAMELEN		(EUSART_BUFFER_SIZE - 8)	// 每帧长度
#define SOFTVERSION		0x00000000UL				// 软件版本

#define INFO_LENGTH		14
#define PACKAGE_LENGTH	1024

#define MYBOOTADDR		0x01	// 设备地址
#define FRAMEFIRST		0xF1	// 帧头
#define FRAMEEND		0xF2	// 帧尾
#define IAPADDR			0xFA	// IAP地址
#define CMD70			0x70	// 文件信息命令码
#define CMD71			0x71	// 文件帧命令码
#define CMD72			0x72	// 升级完成命令码

typedef struct  
{
	uint32_t FileSize;	// 文件长度
	uint16_t FrameLen;	// 帧长度
	uint16_t FrameNum;	// 帧数量
	uint16_t FileCrc;	// 文件CRC
	uint32_t SoftVer;	// 软件版本
}BootFileMessageStruct;	// 需要接收的程序文件信息

typedef struct
{
	uint32_t FileSize;		// 接受的文件长度
	uint16_t FrameNum;		// 接收的文件帧数量
	uint16_t  StartMark;	// 启动接收标志
}ReceiveFileMessageStruct;	// 接受过程中文件累计信息

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
