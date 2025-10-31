# STM32F373RCT6 多功能嵌入式项目

## 项目概述

本项目基于STM32F373RCT6微控制器，是一个综合性嵌入式控制系统，集成了多路信号采集、串口通信、舵机控制等功能。项目采用STM32CubeMX生成基础框架，结合HAL库实现各外设功能，适用于需要高精度数据采集和多设备控制的应用场景。

## 主要功能模块

### 1. 高精度数据采集

#### SDADC高精度模数转换
- **功能说明**：使用SDADC1和SDADC3进行高精度信号采集，适用于需要高分辨率测量的场景
- **主要特点**：
  - 外部参考电压配置
  - 支持自动校准功能
  - 使用DMA进行高效数据传输
- **使用方法**：
  - 系统初始化时自动启动SDADC校准和数据采集
  - 采集数据自动存入`SDADCBUFF1`和`SDADCBUFF2`缓冲区
  - 调用`get_sdadc_dataframe()`获取处理后的数据帧

#### ADC多通道模拟量采集
- **功能说明**：通过ADC1采集4路模拟信号（方向调整信号），用于控制系统的方向调整
- **采集通道**：
  - PA4(SADJ) - 南方调整信号 (ADC1_IN4)
  - PA5(EADJ) - 东方调整信号 (ADC1_IN5)
  - PA6(WADJ) - 西方调整信号 (ADC1_IN6)
  - PA7(NADJ) - 北方调整信号 (ADC1_IN7)
- **技术参数**：
  - 分辨率：12位
  - 采样时间：239.5个ADC时钟周期
  - 转换模式：连续扫描模式
  - 触发方式：软件触发
- **数据传输**：使用DMA1_Channel1进行高效数据传输
  - 传输模式：循环模式（自动覆盖旧数据）
  - 数据对齐：半字（16位）
  - 内存地址递增：启用
- **数据缓冲区**：
  - `adj_frame[4]` - 存储4个通道的ADC转换结果
- **使用方法**：
  - 系统初始化时自动配置ADC并启动DMA传输
  - 无需手动触发转换，数据自动通过DMA存入`adj_frame`数组
  - 可直接读取`adj_frame`数组获取最新的ADC采样值

### 2. 串口通信

#### USART1 (DMA模式)
- **功能说明**：主要用于舵机控制指令发送和高速数据传输
- **引脚定义**：PB6(TX)、PB7(RX)
- **通信参数**：115200bps，8N1
- **使用方法**：
  - 系统自动初始化并启动DMA接收
  - 使用`HAL_UART_Transmit_DMA(&huart1, data, length)`发送数据
  - 接收到的数据可通过`Uart_Dataframe()`函数处理

#### USART2 (中断模式)
- **功能说明**：用于辅助通信和调试
- **引脚定义**：PB3(TX)、PB4(RX)
- **通信参数**：115200bps，8N1
- **使用方法**：
  - 系统自动初始化并启动中断接收
  - 使用`HAL_UART_Transmit_IT(&huart2, data, length)`发送数据

### 3. 舵机控制系统

#### Fashion Driver舵机驱动
- **功能说明**：提供舵机控制指令封装，支持舵机状态检测和角度控制
- **主要函数**：
  - `fashion_send_ping(servo_id)` - 检测舵机是否在线
  - `fashion_send_single_angle(servo_id, angle, time_ms)` - 控制舵机转动到指定角度
- **使用示例**：
  ```c
  // 检测舵机1是否在线
  fashion_send_ping(1);
  HAL_Delay(1000);
  
  // 控制舵机1在500ms内转动到90度
  fashion_send_single_angle(1, 900, 500); // 角度单位为0.1度
  ```

### 4. GPIO控制

#### 方向控制信号
- **功能说明**：控制四个方向（东南西北）的多路信号
- **引脚分配**：
  - 南方控制：S1-S3 (GPIOC引脚1-3)
  - 东方控制：E1-E3 (GPIOA引脚0-2)
  - 西方控制：W1-W3 (GPIOA引脚9-11)
  - 北方控制：N1-N3 (GPIOC引脚7-9)
- **使用方法**：通过HAL库GPIO函数直接控制，如`HAL_GPIO_WritePin()`

### 5. I2C通信接口
- **功能说明**：提供I2C总线通信能力，可连接各类I2C设备
- **时钟源**：使用HSI时钟源

## 系统初始化与使用流程

### 基本初始化流程

1. **系统启动过程**：
   - HAL库初始化
   - 系统时钟配置（72MHz）
   - 各外设初始化（GPIO、DMA、USART、ADC、SDADC等）
   
2. **关键初始化步骤**：
   ```c
   // 初始化串口接收
   HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuff1, BUFFERSIZE);
   HAL_UARTEx_ReceiveToIdle_IT(&huart2, recv_frame2, FRAMESIZE);
   
   // SDADC校准
   HAL_SDADC_CalibrationStart(&hsdadc1, SDADC_CALIBRATION_SEQ_1);
   HAL_SDADC_PollForCalibEvent(&hsdadc1, HAL_MAX_DELAY);
   HAL_SDADC_CalibrationStart(&hsdadc3, SDADC_CALIBRATION_SEQ_1);
   HAL_SDADC_PollForCalibEvent(&hsdadc3, HAL_MAX_DELAY);
   
   // 启动SDADC DMA采集
   HAL_SDADC_InjectedStart_DMA(&hsdadc1, SDADCBUFF1[0], 20);
   HAL_SDADC_InjectedStart_DMA(&hsdadc3, SDADCBUFF2[0], 12);
   ```

### 数据处理与使用

#### 串口数据帧处理
- **功能说明**：处理接收到的串口数据，支持环形缓冲区管理
- **使用方法**：
  ```c
  // 处理USART1接收到的数据
  Uart_Dataframe(&huart1, current_position, data_length);
  // 处理后的数据存储在recv_frame1中
  
  // 处理USART2接收到的数据
  Uart_Dataframe(&huart2, 0, data_length);
  // 处理后的数据存储在recv_frame2中
  ```

#### SDADC数据获取
- **功能说明**：获取并处理SDADC采集的数据
- **使用方法**：
  ```c
  // 获取一帧SDADC数据
  get_sdadc_dataframe();
  // 处理后的数据存储在data_frame数组中
  ```

#### ADC方向调整信号获取
- **功能说明**：获取4路方向调整信号的ADC采样值
- **使用方法**：
  ```c
  // 直接读取ADC采样值
  uint16_t south_value = adj_frame[0];  // 南方调整信号
  uint16_t east_value = adj_frame[1];   // 东方调整信号
  uint16_t west_value = adj_frame[2];   // 西方调整信号
  uint16_t north_value = adj_frame[3];  // 北方调整信号
  
  // 转换为电压值 (假设参考电压为3.3V)
  float south_voltage = (float)south_value * 3.3f / 4095.0f;
  
  // 根据ADC值控制方向信号
  if (south_value > threshold) {
    // 执行南方方向控制逻辑
  }
  ```

## 开发与使用指南

### 硬件连接

1. **串口连接**：
   - USART1 (PB6/PB7)：连接舵机控制器或主通信设备
   - USART2 (PB3/PB4)：连接调试终端或辅助设备

2. **模拟信号输入**：
   - PA4-PA7：连接需要采集的模拟信号
   - SDADC通道：根据硬件设计连接相应传感器

### 功能扩展方法

1. **添加新的传感器**：
   - 在ADC或SDADC中配置新的通道
   - 添加相应的数据处理函数

2. **控制更多舵机**：
   - 使用`fashion_send_single_angle()`函数控制多个舵机
   - 可通过循环或数组批量控制

3. **自定义通信协议**：
   - 在`Uart_Dataframe()`基础上扩展协议解析功能
   - 添加协议验证和错误处理

## 注意事项

1. **SDADC校准**：系统启动时会自动进行SDADC校准，请确保有足够的校准时间

2. **缓冲区大小**：
   - USART1接收缓冲区大小为200字节
   - 确保处理数据的速度不低于接收速度，避免缓冲区溢出

3. **舵机控制**：
   - 舵机角度单位为0.1度（如900表示90.0度）
   - 控制时间参数以毫秒为单位

4. **电源供应**：确保为舵机提供足够的电源电流，避免因电流不足导致控制不稳定

## 项目文件结构

```
Core/
├── Inc/                           // 头文件目录
│   ├── main.h                     // 主程序头文件，全局定义和引脚定义
│   ├── adc.h                      // ADC模块头文件
│   ├── dma.h                      // DMA模块头文件
│   ├── fashion_driver.h           // 舵机驱动头文件
│   ├── gpio.h                     // GPIO模块头文件
│   ├── i2c.h                      // I2C模块头文件
│   ├── sdadc.h                    // SDADC模块头文件
│   └── usart.h                    // 串口模块头文件
├── Src/                           // 源文件目录
│   ├── main.c                     // 主程序，系统初始化和主循环
│   ├── adc.c                      // ADC模块实现
│   ├── dma.c                      // DMA模块实现
│   ├── fashion_driver.c           // 舵机驱动实现
│   ├── fashion_driver_example.c   // 舵机驱动使用示例
│   ├── gpio.c                     // GPIO模块实现
│   ├── i2c.c                      // I2C模块实现
│   ├── sdadc.c                    // SDADC模块实现
│   └── usart.c                    // 串口模块实现
```

## 快速开始

1. **基本使用**：
   - 系统启动后自动初始化所有外设并开始数据采集
   - 可在主循环中添加自定义处理逻辑

2. **舵机控制示例**：
   ```c
   // 在main.c中包含头文件
   #include "fashion_driver.h"
   
   // 在while循环中添加
   void loop() {
     // 舵机控制示例
     fashion_send_ping(1);          // 检测舵机
     HAL_Delay(1000);
     fashion_send_single_angle(1, 0, 500);     // 转到0度
     HAL_Delay(1000);
     fashion_send_single_angle(1, 1800, 500);  // 转到180度
     HAL_Delay(2000);
   }
   ```

3. **数据采集使用**：
   - SDADC数据自动通过DMA采集到缓冲区
   - 调用`get_sdadc_dataframe()`获取处理后的完整数据帧
