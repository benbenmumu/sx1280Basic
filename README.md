# sx1280Basic
**简介**

sx1280测试程序，包含LORA模式收发测试和测距模式测试

更改main.c文件中的LORAMODE和APPMODE宏定义更改模式与收发

```C
/*
 *\brief 0 for Tx and 1 for Rx
 */
#define LORAMODE             0
/*
 *\brief 0 for LORA and 1 for RANGING
 */
#define APPMODE              1
```

上述代码即为设置测距模式，节点为主节点

**单片机型号：** STM32L431RCT6

**引脚设置：**

- SPI1_MISO：PA6，SPI主入从出
- SPI1_MOSI：PA7，SPI主出从入
- SPI1_SCK：PA5，SPI时钟
- SPI1_NSS：PA4，推挽输出，低速，无上下拉，初始高电平
- BUSY：PB2，输入，无上下拉
- RESET：PB11，推挽输出，低速，无上下拉，初始高电平
- DIO1：PB1，输入，无上下拉
- DIO2：PB0，输入，无上下拉
- DIO3：PC5，输入，无上下拉

**IDE**

使用STM32CubeMX5.1.0构建工程，keil5编译
