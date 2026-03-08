#ifndef __LED_CONTROL_H
#define __LED_CONTROL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// LED警示模式
typedef enum {
    LED_MODE_OFF = 0,       // 关闭
    LED_MODE_STANDBY,       // 待机（慢呼吸蓝）
    LED_MODE_CAUTION,       // 注意（慢闪黄）
    LED_MODE_WARNING,       // 警告（快闪橙）
    LED_MODE_DANGER,        // 危险（常亮红）
    LED_MODE_ALARM,         // 紧急（快速闪烁红）
    LED_MODE_INIT,          // 初始化（绿色呼吸）
    LED_MODE_ERROR          // 错误（红蓝交替）

} LedAlertMode_t;

// 距离警示级别
typedef enum {
    DISTANCE_SAFE = 0,      // 安全（>1500mm）
    DISTANCE_CAUTION,       // 注意（1000-1500mm）
    DISTANCE_WARNING,       // 警告（500-1000mm）
    DISTANCE_DANGER,        // 危险（200-500mm）
    DISTANCE_CRITICAL       // 紧急（<200mm）

} DistanceAlertLevel_t;

// 初始化函数
void LED_Control_Init(void);
void LED_Control_Update(bool obstacle_detected, uint16_t distance_mm);
void LED_Control_Run(void);
void LED_Control_Set_Mode(LedAlertMode_t mode);
void LED_Control_Test_Pattern(void);
void LED_Control_Distance_Gradient(uint16_t distance_mm);

#endif /* __LED_CONTROL_H */