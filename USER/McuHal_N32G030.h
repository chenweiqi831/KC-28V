/******************************************************************************

                  ��Ȩ���� (C), 2001-2016, ��������΢�������޹�˾

 ******************************************************************************
  �� �� ��   : McuHal_N32G030.h
  �� �� ��   : ����
  ��    ��   : 1
  ��������   : 2020��8��14�� ������
  ����޸�   :
  ��������   : McuHal_N32G030.c ��ͷ�ļ�
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2020��8��14�� ������
    ��    ��   : 1
    �޸�����   : �����ļ�

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

