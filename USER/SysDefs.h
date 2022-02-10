/******************************************************************************

                  ��Ȩ���� (C), 2001-2016, ��������΢�������޹�˾

 ******************************************************************************
  �� �� ��   : SysDefs.h
  �� �� ��   : ����
  ��    ��   : HuangBingQuan
  ��������   : 2016��11��30�� ������
  ����޸�   :
  ��������   : ϵͳ��Ӳ������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��11��30�� ������
    ��    ��   : HuangBingQuan
    �޸�����   : �����ļ�

******************************************************************************/

#ifndef __SYSDEFS_H_
#define __SYSDEFS_H_

#include "n32g030_conf.h"

#define CPU_CLOCK			48000000UL


// IO ���ܿڶ���
#define UART1_TX_Port        GPIOA
#define UART1_TX_Pin		 GPIO_PIN_9
#define UART1_TX_AF			 GPIO_AF4
#define UART1_RX_Port        GPIOA
#define UART1_RX_Pin         GPIO_PIN_10
#define UART1_RX_AF			 GPIO_AF4


#define DSG_CTRL_Port					GPIOB
#define DSG_CTRL_Pin					GPIO_PIN_1

#define CON_CTRL_Port					GPIOB
#define CON_CTRL_Pin					GPIO_PIN_0

#define DIS_MOS_OFF()                   //{GPIO_ResetBits(DSG_CTRL_Port, DSG_CTRL_Pin);}
#define DIS_MOS_ON()                    //{GPIO_SetBits(DSG_CTRL_Port, DSG_CTRL_Pin);}

#define POWER_CON_OFF()                 {GPIO_ResetBits(CON_CTRL_Port, CON_CTRL_Pin);}
#define POWER_CON_ON()                  {GPIO_SetBits(CON_CTRL_Port, CON_CTRL_Pin);}

/*!< STM8 Standard Peripheral Library old types (maintained for legacy purpose) */

typedef int32_t  		s32;
typedef int16_t 		s16;
typedef int8_t  		s8;

typedef uint32_t  		u32;
typedef uint16_t 		u16;
typedef uint8_t  		u8;
typedef uint8_t			bit;

#define U8_MAX     (255)
#define S8_MAX     (127)
#define S8_MIN     (-128)
#define U16_MAX    (65535u)
#define S16_MAX    (32767)
#define S16_MIN    (-32768)
#define U32_MAX    (4294967295uL)
#define S32_MAX    (2147483647)
#define S32_MIN    (-2147483648uL)

#endif

