/******************************************************************************

                  版权所有 (C), 2001-2016, 惠州市蓝微电子有限公司

 ******************************************************************************
  文 件 名   : SysDefs.h
  版 本 号   : 初稿
  作    者   : HuangBingQuan
  生成日期   : 2016年11月30日 星期三
  最近修改   :
  功能描述   : 系统及硬件定义
  函数列表   :
  修改历史   :
  1.日    期   : 2016年11月30日 星期三
    作    者   : HuangBingQuan
    修改内容   : 创建文件

******************************************************************************/

#ifndef __SYSDEFS_H_
#define __SYSDEFS_H_

#include "n32g030_conf.h"

#define CPU_CLOCK			48000000UL


// IO 功能口定义
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

