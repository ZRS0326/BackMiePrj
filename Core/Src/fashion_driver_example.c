#include "fashion_driver.h"
#include "main.h"

/**
 * @brief Fashion Driver使用示例
 * 这个文件展示了如何使用Fashion Driver来控制舵机
 */

// 示例：在main函数中使用
void fashion_driver_demo(void)
{
    // 1. 检查舵机是否在线
    fashion_send_ping(1);
    
    // 等待一段时间（实际应用中应该等待响应）
    HAL_Delay(1000);
    
    // 2. 控制舵机转动到0度
    fashion_send_single_angle(1, 0, 500);
    HAL_Delay(1000);
    
    // 3. 控制舵机转动到90度
    fashion_send_single_angle(1, 900, 500);
    HAL_Delay(1000);
    
    // 4. 控制舵机转动到180度
    fashion_send_single_angle(1, 1800, 500);
    HAL_Delay(1000);
    
    // 5. 读取舵机当前角度
    // 注意：需要在串口接收中断中处理返回的数据
    fashion_read_servo_angle(1);
    HAL_Delay(500); // 等待响应
}

/**
 * @brief 在main.c中的使用示例
 * 在main函数的while循环中添加以下代码：
 * 
 * // 在USER CODE BEGIN 2区域添加
 * #include "fashion_driver.h"
 * 
 * // 在while循环中调用
 * fashion_driver_demo();
 * HAL_Delay(2000); // 每2秒执行一次演示
 */