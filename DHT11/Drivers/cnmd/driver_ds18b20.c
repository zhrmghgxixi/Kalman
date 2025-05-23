// SPDX-License-Identifier: GPL-3.0-only
/*
 * Copyright (c) 2008-2023 100askTeam : Dongshan WEI <weidongshan@qq.com> 
 * Discourse:  https://forums.100ask.net
 */

 
/*  Copyright (C) 2008-2023 深圳百问网科技有限公司
 *  All rights reserved
 *
 *
 * 免责声明: 百问网编写的文档，仅供学员学习使用，可以转发或引用(请保留作者信息)，禁止用于商业用途！
 * 免责声明: 百问网编写的程序，可以用于商业用途，但百问网不承担任何后果！
 * 
 * 
 * 本程序遵循GPL V3协议，使用请遵循协议许可
 * 本程序所用的开发板：	DShanMCU-F103
 * 百问网嵌入式学习平台：https://www.100ask.net
 * 百问网技术交流社区：	https://forums.100ask.net
 * 百问网官方B站：				https://space.bilibili.com/275908810
 * 百问网官方淘宝：			https://100ask.taobao.com
 * 联系我们(E-mail)：	  weidongshan@qq.com
 *
 * 版权所有，盗版必究。
 *  
 * 修改历史     版本号           作者        修改内容
 *-----------------------------------------------------
 * 2023.08.04      v01         百问科技      创建文件
 *-----------------------------------------------------
 */


#include "driver_ds18b20.h"
#include "driver_lcd.h"
#include "driver_timer.h"
#include "stm32f1xx_hal.h"
#include "onewire.h"
/* rom commands */
#define SEARCH_ROM    0xF0
#define READ_ROM      0x33
#define MATCH_ROM     0x55
#define SKIP_ROM      0xCC
#define ALARM_ROM     0xEC

/* functions commands */
#define CONVERT_TEAMPERATURE 0x44
#define WRITE_SCRATCHPAD     0x4E
#define READ_SCRATCHPAD      0xBE
#define COPY_SCRATCHPAD      0x48
#define RECALL_EEPROM        0xB8
#define READ_POWER_SUPPLY    0xB4


/* 先实现GPIO的基本操作 */
/**********************************************************************
 * 函数名称： DS18B20_PinCfgAsOutput
 * 功能描述： 把DS18B20的数据引脚配置为输出
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_PinCfgAsOutput(void)
{
    /* 对于STM32F103, 已经把DS18B20的引脚配置为"open drain, pull-up" */
}

/**********************************************************************
 * 函数名称： DS18B20_PinCfgAsInput
 * 功能描述： 把DS18B20的数据引脚配置为输入
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_PinCfgAsInput(void)
{
    /* 对于STM32F103, 已经把DS18B20的引脚配置为"open drain, pull-up" 
	* 让它输出1就不会驱动这个引脚, 并且可以读入引脚状态
     */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}


/**********************************************************************
 * 函数名称： DS18B20_PinSet
 * 功能描述： 设置DS18B20的数据引脚的输出值
 * 输入参数： val - 输出电平
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_PinSet(int val)
{
	if (val)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

/**********************************************************************
 * 函数名称： DS18B20_PinRead
 * 功能描述： 读取DS18B20的数据引脚
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 1-高电平, 0-低电平
 * 修改日期        版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int DS18B20_PinRead(void)
{
    if (GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1))
		return 1;
	else
		return 0;
}

/**********************************************************************
 * 函数名称： DS18B20_PinSetForTime
 * 功能描述： 设置DS18B20的数据引脚,并维持一定时间
 * 输入参数： val - 输出电平
 *            us  - 维持多少us
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_PinSetForTime(int val, int us)
{
	DS18B20_PinCfgAsOutput();
	DS18B20_PinSet(val);
	udelay(us);
}

/**********************************************************************
 * 函数名称： DS18B20_PinRelease
 * 功能描述： 释放引脚,就是不再驱动引脚,配置为输入即可
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_PinRelease(void)
{
	DS18B20_PinCfgAsInput();
}

/* ds18b20的代码 */
/**********************************************************************
 * 函数名称： DS18B20_Start
 * 功能描述： 给DS18B20发出启动信号,并返回响应值
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 1-无响应, 0-有响应
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int DS18B20_Start(void)
{
	int val;
	
	DS18B20_PinSetForTime(0, 500);
	DS18B20_PinRelease();
	udelay(80);

	val = DS18B20_PinRead();
	udelay(250);
	return val;
}

/**********************************************************************
 * 函数名称： DS18B20_WriteBit
 * 功能描述： 给DS18B20发送1bit数据
 * 输入参数： val - 数据
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/

float DS18B20_GetTemp(void)
{
    float temperature = 0.0;
    // 实现读取温度的逻辑
    // 比如使用OneWire协议与DS18B20通信并获取温度值

    // 示例代码
    // 假设已实现OneWire协议的初始化和通信
    onewire_t ow;
    ow.pin = GPIO_PIN_9;  // 假设DS18B20连接到PA9
    ow.port = GPIOA;
    onewire_init(&ow);

    // DS18B20启动温度转换
    onewire_reset(&ow);
    onewire_write_byte(&ow, 0xCC);  // Skip ROM命令
    onewire_write_byte(&ow, 0x44);  // Convert T命令

    // 等待转换完成
    HAL_Delay(750);

    // 读取温度寄存器
    onewire_reset(&ow);
    onewire_write_byte(&ow, 0xCC);  // Skip ROM命令
    onewire_write_byte(&ow, 0xBE);  // Read Scratchpad命令

    unsigned char temp_lsb = onewire_read_byte(&ow);
    unsigned char temp_msb = onewire_read_byte(&ow);
    int16_t temp = (temp_msb << 8) | temp_lsb;

    // 转换温度值
    temperature = temp * 0.0625;

    return temperature;
}




static void DS18B20_WriteBit(int val)
{
	if (0 == val)
	{
		DS18B20_PinSetForTime(0, 60);		
		DS18B20_PinRelease();
		udelay(2);
	}
	else
	{
		DS18B20_PinSetForTime(0, 2);		
		DS18B20_PinRelease();
		udelay(60);
	}
}

/**********************************************************************
 * 函数名称： DS18B20_ReadBit
 * 功能描述： 读取DS18B20 1bit数据
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 1/0 - 数据
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int DS18B20_ReadBit(void)
{
	int val;
	
	DS18B20_PinSetForTime(0, 2);		
	DS18B20_PinRelease();
	udelay(10);
	val = DS18B20_PinRead();
	udelay(50);
	return val;
}

/**********************************************************************
 * 函数名称： DS18B20_WriteByte
 * 功能描述： 给DS18B20发送1 byte数据
 * 输入参数： data - 数据
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_WriteByte(unsigned char data)
{
	int i;
	for (i = 0; i < 8; i++)
	{

		DS18B20_WriteBit(data & (1<<i));
	}
}

/**********************************************************************
 * 函数名称： DS18B20_ReadByte
 * 功能描述： 读取DS18B20 1 byte数据
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 数据
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static unsigned char DS18B20_ReadByte(void)
{
	int i;
	unsigned char data = 0;

	for (i = 0; i < 8; i++)
	{
		if (DS18B20_ReadBit() == 1)
			data |= (1<<i);
	}

	return data;
}

/**********************************************************************
 * 函数名称： DS18B20_WriteRomCmd
 * 功能描述： 发送ROM命令
 * 输入参数： cmd - ROM命令
 * 输出参数： 无
 * 返 回 值： 数据
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_WriteRomCmd(unsigned char cmd)
{
	DS18B20_WriteByte(cmd);
}

/**********************************************************************
 * 函数名称： DS18B20_WriteFunctionCmd
 * 功能描述： 发送Function命令
 * 输入参数： cmd - Function命令
 * 输出参数： 无
 * 返 回 值： 数据
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static void DS18B20_WriteFunctionCmd(unsigned char cmd)
{
	DS18B20_WriteByte(cmd);
}


/**********************************************************************
 * 函数名称： DS18B20_WaitReady
 * 功能描述： 等待DS18B20就绪
 * 输入参数： timeout_us - 超时时间(单位us)
 * 输出参数： 无
 * 返 回 值： 0 - 就绪, (-1) - 失败
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int DS18B20_WaitReady(int timeout_us)
{
	while (timeout_us--)
	{
		if (DS18B20_ReadBit() == 1)
			return 0;  /* ok */
		udelay(1);
	}
	return -1;
}

/**********************************************************************
 * 函数名称： DS18B20_StartConvert
 * 功能描述： 启动DS18B20的温度转换
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 0 - 成功, (-1) - 失败
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int DS18B20_StartConvert(void)
{
	if (DS18B20_Start() != 0)
	{
		//printf("DS18B20_Start err!\n\r");
		return -1;
	}

	DS18B20_WriteRomCmd(SKIP_ROM);
	DS18B20_WriteFunctionCmd(CONVERT_TEAMPERATURE);

	/* 等待/判断转换成功 */
	if (0 != DS18B20_WaitReady(1000000))
	{
		//printf("DS18B20_WaitReady err!\n\r");
		return -1;
	}

	return 0;	
}

/**********************************************************************
 * 函数名称： DS18B20_Read_RAM
 * 功能描述： 读取DS18B20的内存数据(9字节)
 * 输入参数： 无
 * 输出参数： ram - 用于保存输出数据的buffer
 * 返 回 值： 0 - 成功, (-1) - 失败
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
static int DS18B20_Read_RAM(unsigned char ram[])
{
	int i;
	
	if (DS18B20_Start() != 0)
	{
		//printf("DS18B20_Start err!\n\r");
		return -1;
	}

	DS18B20_WriteRomCmd(SKIP_ROM);
	DS18B20_WriteFunctionCmd(READ_SCRATCHPAD);

	for (i = 0; i < 9; i++)
	{
		ram[i] = DS18B20_ReadByte();
	}

	return 0;
}

/* 实际操作函数 */
/**********************************************************************
 * 函数名称： DS18B20_Init
 * 功能描述： DS18B20的初始化函数
 * 输入参数： 无
 * 输出参数： 无
 * 返 回 值： 无
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
void DS18B20_Init(void)
{
	DS18B20_PinRelease();
}

/**********************************************************************
 * 函数名称： DS18B20_ReadROM
 * 功能描述： 读取DS18B20的ROM ID
 * 输入参数： 无
 * 输出参数： rom - 用于保存输出数据的buffer
 * 返 回 值： 0 - 成功, (-1) - 失败
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
int DS18B20_ReadROM(unsigned char rom[])
{
	int i;
	
	if (DS18B20_Start() != 0)
	{
		//printf("DS18B20_Start err!\n\r");
		return -1;
	}

	DS18B20_WriteRomCmd(READ_ROM);
	
	for (i = 0; i < 8; i++)
	{
		rom[i] = DS18B20_ReadByte();
	}

	return 0;
}

/**********************************************************************
 * 函数名称： DS18B20_Read_Temperature
 * 功能描述： 读取DS18B20的温度值
 * 输入参数： 无
 * 输出参数： temp - 用于保存温度值
 * 返 回 值： 0 - 成功, (-1) - 失败
 * 修改日期：      版本号     修改人	      修改内容
 * -----------------------------------------------
 * 2023/08/03	     V1.0	  韦东山	      创建
 ***********************************************************************/
int DS18B20_Read_Temperature(double *temp)
{
	int err;
	unsigned char ram[9];
	double val[] = {0.0625, 0.125, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64};
	double sum = 0;
	int i;
	
	err = DS18B20_StartConvert();
	if (err)
		return err;

	err = DS18B20_Read_RAM(ram);
	if (err)
		return err;

	/* 计算温度 */

	/* 先判断精度 */
	if (ram[4] & (3<<5) == 0) /* 精度: 9bit */
		i = 3;
	else if (ram[4] & (3<<5) == (1<<5)) /* 精度: 10bit */
		i = 2;
	else if (ram[4] & (3<<5) == (2<<5)) /* 精度: 11bit */
		i = 1;
	else
		/* 精度是 12 bit */
		i = 0;
	
	for (; i < 8; i++)
	{
		if (ram[0] & (1<<i))
			sum += val[i];
	}

	for (i = 0; i < 3; i++)
	{
		if (ram[1] & (1<<i))
			sum += val[8+i];
	}

	if (ram[1] & (1<<3))
		sum = 0 - sum;

	*temp = sum;
	return 0;
}


/**********************************************************************
 * 函数名称： DS18B20_Test
 * 功能描述： DS18B20测试程序
 * 输入参数： 无
 * 输出参数： 无
 *            无
 * 返 回 值： 无
 * 修改日期        版本号     修改人        修改内容
 * -----------------------------------------------
 * 2023/08/03        V1.0     韦东山       创建
 ***********************************************************************/
void DS18B20_Test(void)
{
	unsigned char rom[8];    
    
	int i;
	double temp;
	int m,n,len;

	DS18B20_Init();
	
	//while (1)
	{
		if (DS18B20_ReadROM(rom) == 0)
		{
			LCD_PrintString(0, 0, "ds18b20 rom:");
			for (i = 0; i < 8; i++)
			{
				LCD_PrintHex(i*2, 2, rom[i], 0);
			}
		}
	}

    LCD_PrintString(0, 4, "Temperature:");
	while (1)
	{
		if (0 == DS18B20_Read_Temperature(&temp))
		{
			m = (int)temp;	/* 3.01, m = 3 */
			temp = temp - m;	/* 小数部分: 0.21 */
			n = temp * 10;  /* n = 2 */
			
			/* 在LCD上打印 */
			//printf("ds18b20 temperature: %d.%04d\n\r", m, n);  /* 3.010v */
			len = LCD_PrintSignedVal(0, 6, m);
            LCD_PutChar(len, 6, '.');
			LCD_PrintSignedVal(len+1, 6, n);
		}
	}
}


