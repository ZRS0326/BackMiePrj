# STM32F3 示例项目

## 项目概述

## 硬件配置

### 串口引脚配置
- **USART1**: PA9(TX), PA10(RX)
- **USART2**: PA2(TX), PA3(RX)

### 通信参数
- 波特率: 115200
- 数据位: 8位
- 停止位: 1位
- 校验位: 无
- 流控制: 无

## 核心功能

### 1. 缓冲区定义

在 `usart.c` 中定义了以下全局变量：

```c
uint8_t ReceiveBuff1[BUFFERSIZE] = {0};  // USART1接收缓冲区
uint8_t ReceiveBuff2[BUFFERSIZE] = {0};  // USART2接收缓冲区
uint8_t base_addr1 = 0;                  // USART1缓冲区基地址
uint8_t base_addr2 = 0;                  // USART2缓冲区基地址
uint8_t recv_frame1[20] = {0};           // USART1接收数据帧
uint8_t recv_frame2[20] = {0};           // USART2接收数据帧
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

#### 发送数据
使用HAL库的标准发送函数：
```c
HAL_UART_Transmit(&huart1, data, length, timeout);
HAL_UART_Transmit_DMA(&huart1, data, length);  // DMA方式
```

#### 接收数据
- 接收使用DMA循环模式，数据自动存入对应的ReceiveBuff缓冲区
- 缓冲区采用环形队列结构，大小为BUFFERSIZE（默认为200）

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
Uart_Dataframe(&huart1, current_tail);

// 处理接收到的数据帧
// recv_frame1 中现在包含了从base_addr1到target的数据
```

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

## 文件结构

```
Core/
├── Inc/
│   ├── main.h          // 全局定义和缓冲区声明
│   └── usart.h         // 串口函数声明
├── Src/
│   ├── main.c          // 主程序
│   └── usart.c         // 串口实现（包含数据帧处理）
```

## 扩展建议

1. 可添加数据帧解析函数，对recv_frame中的数据进行协议解析
2. 可增加发送数据帧的函数，封装数据打包和发送过程
3. 可添加错误处理机制，如校验和验证、超时处理等
