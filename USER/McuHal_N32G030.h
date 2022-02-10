/******************************************************************************

                  版权所有 (C), 2001-2016, 惠州市蓝微电子有限公司

 ******************************************************************************
  文 件 名   : McuHal_N32G030.h
  版 本 号   : 初稿
  作    者   : 1
  生成日期   : 2020年8月14日 星期五
  最近修改   :
  功能描述   : McuHal_N32G030.c 的头文件
  函数列表   :
  修改历史   :
  1.日    期   : 2020年8月14日 星期五
    作    者   : 1
    修改内容   : 创建文件

******************************************************************************/
#ifndef __MCUHAL_N32G030_H__
#define __MCUHAL_N32G030_H__

#include "SysDefs.h"

#define FLASH_PAGE_SIZE					512
#define FLASH_DATA_BASE_ADDR			(1024UL*(64-2))
#define FLASH_DATA_SIZE					(1024*2)



extern void ADCDeInit(void);
extern u16 ADRead(u8 channel);
extern void Delay10us(u32 uldelay);
extern void Delay10us_INT(u32 uldelay);
extern void Delay1ms(u16 TimeTick);
extern void HwClrWdt(void);
extern void HwDataInit(void);
extern void HwDataLock(void);
extern void HwDataUnlock(void);
extern u8 HwDataRead(u32 Addr, u32 *pData, u16 Len);
extern u8 HwDataReadRandom(u32 Addr, u32 *pData, u16 Len);
extern u8 HwDataPageWrite(u32 Addr, u32 *pData, u16 Len);
extern u8 HwDataPageWriteRandom(u32 Addr, u32 *pData, u16 Len);
extern void HwIntDisable(void);
extern void HwIntEnable(void);
extern void HwMcuInit(void);
extern void HwMcuReset(void);
extern void UartRxDisable(void);
extern void UartRxEnable(void);
extern void UartSendByte(u8 tx_data);
extern void UartSendByteIT(u8 tx_data);
extern void UartSendBuffer(u8 *pBuf, u8 length);
extern u8 UartReceiveByte(void);



#endif /* __MCUHAL_N32G030_H__ */

