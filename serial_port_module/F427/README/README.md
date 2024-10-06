# Robomaster 视觉端与电控端通信示例
## 电控端代码演示
***打开STM32CUBEMX,先配置RCC,时钟树与SYS，之后确定好与视觉端对接的串口，示例代码以Robomaster A板为例，打开USART6,配置波特率为115200,其他配置选择默认的即可。然后点击DMA Settings->Add->选择USART6_RX,DMA的传输模式，传输方向，传输大小默认即可，适当提升对应DMA通道优先级......生成代码。***

***如图：***
![alt text](<屏幕截图 2024-10-03 200428.png>)

STM32CUBEMX|Keil5
-|-

------


### _头文件与全局变量_
```
#include "stdio.h"
#include "string.h"
uint8_t t[9];
uint8_t r[21];
```

### _main.c_
```	
   HAL_UARTEx_ReceiveToIdle_DMA(&huart6,r,21);		
   t[0] = 0x5A;
   float data1 = 4.0;
   float data2 = 5.0;
   // 将 float 转换为字节并存储S
   memcpy(t + 1, &data1, sizeof(data1));
   memcpy(t + 5, &data2, sizeof(data2));
```

### _while_
```
    HAL_UART_Transmit_DMA(&huart6,t,9);	
    //或printf，重定义即可，跳转到[fputc](#重映射非必要)
	HAL_Delay(1000);
```

### _DMA事件反馈函数_
```
void  HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if(huart==&huart6)
  {
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6,r,21);			
  }
	HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
	float pitch, yaw;
    pitch = (float)((r[1] << 24) | (r[2] << 16) | (r[3] << 8)  | (r[4]));
    yaw = (float)((r[5] << 24) | (r[6] << 16) | (r[7] << 8)  | (r[8]));
}
```
### _重映射(非必要)_
_需要加入头文件_``#include "stdio.h"``

```
//加入以下代码,支持printf函数,而不霿要鿉择use MicroLIB	  
//#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)	
#if 1
//#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct _FILE
{ 
	int handle; 
}; 
 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	 HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0x0001);  
	return ch;
}
#endif 
 ```

