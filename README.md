# STM32F373RCT6 多功能嵌入式项目

## 项目概述

本项目基于STM32F373RCT6微控制器，是一个综合性的嵌入式系统，集成了多种外设功能。项目采用STM32CubeMX生成代码框架，支持多通道数据采集、串口通信、I2C通信等功能。

## 系统特性

### 主要功能模块
- **串口通信**: USART1和USART2双串口通信
- **模拟信号采集**: ADC1多通道模拟信号采集
- **Sigma-Delta ADC**: SDADC1和SDADC3高精度数据采集
- **I2C通信**: I2C1总线通信接口
- **GPIO控制**: 多路GPIO输入输出控制

### 系统时钟配置
- **主时钟**: 72MHz (HSE 8MHz × PLL 9倍频)
- **外设时钟**: 独立配置各外设时钟源

## 硬件配置

### 引脚分配

#### GPIO控制引脚
- **S1, S2, S3**: GPIOC引脚1-3 - 南方控制信号
- **E1, E2, E3**: GPIOA引脚0-2 - 东方控制信号  
- **W1, W2, W3**: GPIOA引脚9-11 - 西方控制信号
- **N1, N2, N3**: GPIOC引脚7-9 - 北方控制信号

#### 调整信号引脚
- **SADJ, EADJ, WADJ, NADJ**: GPIOA引脚4-7 - 方向调整信号
- **SADA, SADB, EADA, EADB, NADA, NADB, WADA, WADB**: 多路ADC相关信号

### 串口通信配置
- **USART1**: PB6(TX), PB7(RX) - 使用DMA传输
- **USART2**: PB3(TX), PB4(RX) - 使用中断传输
- **波特率**: 115200
- **数据位**: 8位
- **停止位**: 1位
- **校验位**: 无

### 模拟信号采集
- **ADC1**: 多通道模拟信号采集
- **SDADC1/SDADC3**: Sigma-Delta ADC高精度采集
- **I2C1**: I2C总线通信接口

## 外设初始化

### 系统初始化流程
```c
int main(void)
{
    HAL_Init();                    // HAL库初始化
    SystemClock_Config();         // 系统时钟配置
    
    // 外设初始化
    MX_GPIO_Init();               // GPIO初始化
    MX_DMA_Init();                // DMA初始化
    MX_USART1_UART_Init();        // USART1初始化
    MX_USART2_UART_Init();        // USART2初始化
    MX_I2C1_Init();               // I2C1初始化
    MX_ADC1_Init();               // ADC1初始化
    MX_SDADC1_Init();             // SDADC1初始化
    MX_SDADC3_Init();             // SDADC3初始化
    
    // 启动数据接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuff1, BUFFERSIZE);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, recv_frame2, FRAMESIZE);
}
```

## DMA配置

### USART1 DMA配置
项目为USART1配置了完整的DMA传输功能，实现高效的数据传输：

#### 接收DMA配置 (DMA1 Channel5)
- **模式**: 循环模式 (DMA_CIRCULAR)
- **方向**: 外设到内存 (DMA_PERIPH_TO_MEMORY)
- **数据对齐**: 字节对齐
- **优先级**: 低优先级
- **内存地址递增**: 启用
- **外设地址递增**: 禁用

#### 发送DMA配置 (DMA1 Channel4)  
- **模式**: 普通模式 (DMA_NORMAL)
- **方向**: 内存到外设 (DMA_MEMORY_TO_PERIPH)
- **数据对齐**: 字节对齐
- **优先级**: 低优先级
- **内存地址递增**: 启用
- **外设地址递增**: 禁用

### DMA句柄定义
```c
DMA_HandleTypeDef hdma_usart1_rx;  // USART1接收DMA
DMA_HandleTypeDef hdma_usart1_tx;  // USART1发送DMA
```

## 核心功能

### 1. 缓冲区定义

项目定义了多个缓冲区用于数据存储和处理：

```c
// 缓冲区大小定义
#define BUFFERSIZE 200           // 可以接收的最大字符个数   
#define FRAMESIZE BUFFERSIZE/4   // 数据帧大小

// 串口缓冲区
uint8_t ReceiveBuff1[BUFFERSIZE] = {0};  // USART1接收缓冲区
uint8_t base_addr1 = 0;                  // USART1缓冲区基地址
uint8_t recv_frame1[FRAMESIZE] = {0};   // USART1接收数据帧
uint8_t recv_frame2[FRAMESIZE] = {0};   // USART2接收数据帧
```

### 2. 主要函数

#### 初始化函数
```c
void MX_USART1_UART_Init(void);   // 初始化USART1
void MX_USART2_UART_Init(void);   // 初始化USART2
```

#### 数据帧处理函数
```c
void Uart_Dataframe(UART_HandleTypeDef *huart, uint8_t target);
```

### 3. 数据收发流程

#### 数据接收启动
在main函数中启动数据接收：
```c
// USART1使用DMA接收（循环模式）
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuff1, BUFFERSIZE);

// USART2使用中断接收
HAL_UARTEx_ReceiveToIdle_IT(&huart2, recv_frame2, FRAMESIZE);
```

#### USART1发送数据 (DMA方式)
使用DMA进行高效数据传输：
```c
// DMA发送数据
HAL_UART_Transmit_DMA(&huart1, data, length);

// 检查DMA传输状态
HAL_DMA_GetState(&hdma_usart1_tx);
```

#### USART2发送数据 (中断方式)
使用中断方式进行数据传输：
```c
HAL_UART_Transmit_IT(&huart2, data, length);
```

#### 接收数据特点
- **USART1**: 使用DMA循环模式接收，数据自动存入ReceiveBuff1缓冲区
- **USART2**: 使用中断方式接收数据到固定大小的帧缓冲区
- 缓冲区采用环形队列结构，大小为BUFFERSIZE（200字节）

### 4. 数据帧处理机制

#### 函数原型
```c
void Uart_Dataframe(UART_HandleTypeDef *huart, uint8_t target);
```

#### 参数说明
- `huart`: UART句柄（&huart1 或 &huart2）
- `target`: 当前接收数据的尾地址（下标）

#### 处理逻辑

**情况1：正常情况（base_addr ≤ target）**
```c
memcpy(recv_frame, &ReceiveBuff[base_addr], target - base_addr);
base_addr = target;  // 更新基地址
```

**情况2：环形缓冲区回绕（base_addr > target）**
```c
// 第一段：从base_addr到缓冲区末尾
memcpy(recv_frame, &ReceiveBuff[base_addr], BUFFERSIZE - base_addr);

// 第二段：从缓冲区开头到target
memcpy(&recv_frame[BUFFERSIZE - base_addr], ReceiveBuff, target);

base_addr = target;  // 更新基地址
```

#### 使用示例
```c
// 在接收中断或主循环中调用
uint8_t current_tail = /* 获取当前DMA接收位置 */;
Uart_Dataframe(&huart1, current_tail, size);

// 处理接收到的数据帧
// recv_frame1 中现在包含了从base_addr1到target的数据

// USART1使用DMA自动回送接收到的数据帧
// USART2使用中断方式回送接收到的数据帧
```

### 4. DMA传输优势

#### 性能优势
- **减少CPU占用**: DMA传输期间CPU可执行其他任务
- **高效数据传输**: 支持大块数据连续传输
- **自动缓冲区管理**: 循环模式自动处理缓冲区回绕

#### 配置特点
- **USART1接收**: 循环模式，适合连续数据流
- **USART1发送**: 普通模式，适合单次数据传输
- **自动错误处理**: DMA传输错误自动触发错误回调

## 缓冲区管理

### 环形缓冲区特性
- 缓冲区大小：BUFFERSIZE（200字节）
- 接收帧大小：20字节
- DMA模式：循环接收，自动处理缓冲区回绕
- 数据拷贝：使用memcpy高效拷贝

### 地址管理
- `base_addr`: 当前数据帧的起始位置
- `target`: 新接收数据的结束位置
- 每次调用`Uart_Dataframe`后，base_addr会更新为target

## 注意事项

1. **缓冲区溢出**：确保recv_frame大小（20字节）足够存储需要的数据帧
2. **DMA配置**：接收DMA配置为循环模式，无需手动重启
3. **中断处理**：可根据需要添加接收完成中断处理
4. **线程安全**：在多任务环境中使用时需添加互斥保护

## 项目文件结构

```
Core/
├── Inc/                           // 头文件目录
│   ├── main.h                     // 主程序头文件，包含全局定义和引脚定义
│   ├── adc.h                      // ADC模块头文件
│   ├── dma.h                      // DMA模块头文件
│   ├── gpio.h                     // GPIO模块头文件
│   ├── i2c.h                      // I2C模块头文件
│   ├── sdadc.h                    // Sigma-Delta ADC模块头文件
│   ├── stm32f3xx_hal_conf.h       // HAL库配置文件
│   ├── stm32f3xx_it.h             // 中断服务程序头文件
│   └── usart.h                    // 串口模块头文件
├── Src/                           // 源文件目录
│   ├── main.c                     // 主程序文件，包含系统初始化和主循环
│   ├── adc.c                      // ADC模块实现
│   ├── dma.c                      // DMA模块实现
│   ├── gpio.c                     // GPIO模块实现
│   ├── i2c.c                      // I2C模块实现
│   ├── sdadc.c                    // Sigma-Delta ADC模块实现
│   ├── stm32f3xx_hal_msp.c        // HAL MSP回调函数
│   ├── stm32f3xx_it.c             // 中断服务程序
│   ├── syscalls.c                 // 系统调用
│   ├── sysmem.c                   // 系统内存管理
│   ├── system_stm32f3xx.c         // 系统时钟配置
│   └── usart.c                    // 串口模块实现（包含数据帧处理）
└── Startup/                       // 启动文件目录
    └── startup_stm32f373rctx.s     // STM32启动汇编文件
```

## 扩展建议

1. 可添加数据帧解析函数，对recv_frame中的数据进行协议解析
2. 可增加发送数据帧的函数，封装数据打包和发送过程
3. 可添加错误处理机制，如校验和验证、超时处理等
