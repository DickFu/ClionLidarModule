#include "led_control.h"
#include "ws2812.h"
#include "math.h"
#include <stdio.h>
#include "lidar.h"
// LED状态
static struct {
    LedAlertMode_t current_mode;
    LedAlertMode_t previous_mode;
    bool obstacle_detected;
    uint16_t obstacle_distance;
    DistanceAlertLevel_t alert_level;
    
    // 动画状态
    uint32_t last_update_time;
    uint8_t animation_step;
    uint16_t rainbow_hue;
} led_state = {0};

// 距离阈值配置（单位：mm）
static const uint16_t distance_thresholds[] = {
    1500,  // DISTANCE_SAFE -> DISTANCE_CAUTION
    1000,  // DISTANCE_CAUTION -> DISTANCE_WARNING
    500,   // DISTANCE_WARNING -> DISTANCE_DANGER
    200    // DISTANCE_DANGER -> DISTANCE_CRITICAL
};

/**
  * @brief  初始化LED控制
  */
void LED_Control_Init(void)
{
    ws2812_Clear();
    ws2812_Show();
    
    led_state.current_mode = LED_MODE_INIT;
    led_state.previous_mode = LED_MODE_OFF;
    led_state.obstacle_detected = false;
    led_state.obstacle_distance = 0;
    led_state.alert_level = DISTANCE_SAFE;

    led_state.last_update_time = HAL_GetTick();
    led_state.animation_step = 0;
    led_state.rainbow_hue = 0;

    printf("[LED] Control initialized\r\n");
}

/**
  * @brief  更新LED状态
  */
void LED_Control_Update(bool obstacle_detected, uint16_t distance_mm)
{
    led_state.obstacle_detected = obstacle_detected;
    led_state.obstacle_distance = distance_mm;
    
    if (!obstacle_detected || distance_mm == 0)
    {
        led_state.alert_level = DISTANCE_SAFE;
        LED_Control_Set_Mode(LED_MODE_STANDBY);
    }
    else
    {
        // 根据距离确定警示级别
        if (distance_mm > distance_thresholds[0])
            led_state.alert_level = DISTANCE_SAFE;
        else if (distance_mm > distance_thresholds[1])
            led_state.alert_level = DISTANCE_CAUTION;
        else if (distance_mm > distance_thresholds[2])
            led_state.alert_level = DISTANCE_WARNING;
        else if (distance_mm > distance_thresholds[3])
            led_state.alert_level = DISTANCE_DANGER;
        else
            led_state.alert_level = DISTANCE_CRITICAL;
        
        // 设置对应的LED模式
        switch (led_state.alert_level)
        {
            case DISTANCE_SAFE:
                LED_Control_Set_Mode(LED_MODE_STANDBY);
                break;
            case DISTANCE_CAUTION:
                LED_Control_Set_Mode(LED_MODE_CAUTION);
                break;
            case DISTANCE_WARNING:
                LED_Control_Set_Mode(LED_MODE_WARNING);
                break;
            case DISTANCE_DANGER:
                LED_Control_Set_Mode(LED_MODE_DANGER);
                break;
            case DISTANCE_CRITICAL:
                LED_Control_Set_Mode(LED_MODE_ALARM);
                break;
        }
    }
}

/**
  * @brief  设置LED模式
  */
void LED_Control_Set_Mode(LedAlertMode_t mode)
{
    if (mode == led_state.current_mode)
        return;
    
    led_state.previous_mode = led_state.current_mode;
    led_state.current_mode = mode;
    led_state.animation_step = 0;
}

/**
  * @brief  LED控制主循环
  */
void LED_Control_Run(void)
{
    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - led_state.last_update_time;
    
    // 100Hz更新频率
    if (elapsed >10) {
        led_state.last_update_time = current_time;
    led_state.animation_step++;

    switch (led_state.current_mode)
    {
        case LED_MODE_OFF:
            ws2812_Clear();
            ws2812_Show();
            break;

        case LED_MODE_STANDBY:
            // 蓝色呼吸效果（2秒周期）
            {
                uint8_t brightness = (uint8_t)(127.5f * (sinf(2 * 3.14159f * led_state.animation_step / 200.0f) + 1.0f));
                ws2812_SetAll(0, 0, brightness);
                ws2812_Show();
            }
            break;

        case LED_MODE_CAUTION:
            // 黄色闪烁（1Hz）
            {
                bool on_phase = (led_state.animation_step % 100) < 50;
                if (on_phase)
                {
                    ws2812_SetAll(255, 150, 0);  // 黄色
                }
                else
                {
                    ws2812_SetAll(30, 15, 0);    // 暗黄色
                }
                ws2812_Show();
            }
            break;

        case LED_MODE_WARNING:
            // 橙色闪烁（2Hz）
            {
                bool on_phase = (led_state.animation_step % 50) < 25;
                if (on_phase)
                {
                    ws2812_SetAll(255, 80, 0);   // 橙色
                }
                else
                {
                    ws2812_SetAll(50, 15, 0);    // 暗橙色
                }
                ws2812_Show();
            }
            break;

        case LED_MODE_DANGER:
            // 红色常亮
            ws2812_SetAll(255, 0, 0);
            ws2812_Show();
            break;

        case LED_MODE_ALARM:
            // 红色快速闪烁（4Hz）
            {
                bool on_phase = (led_state.animation_step % 25) < 13;
                if (on_phase)
                {
                    ws2812_SetAll(255, 0, 0);    // 亮红色
                }
                else
                {
                    ws2812_Clear();              // 熄灭
                }
                ws2812_Show();
            }
            break;

        case LED_MODE_INIT:
            // 绿色呼吸效果
            {
                uint8_t brightness = (uint8_t)(127.5f * (sinf(2 * 3.14159f * led_state.animation_step / 150.0f) + 1.0f));
                ws2812_SetAll(0, brightness, 0);
                ws2812_Show();
            }
            break;

        case LED_MODE_ERROR:
            // 红蓝交替闪烁（2Hz）
            {
                uint8_t phase = (led_state.animation_step % 100) / 25;
                switch (phase)
                {
                    case 0:  // 红色
                        ws2812_SetAll(255, 0, 0);
                        break;
                    case 1:  // 熄灭
                        ws2812_Clear();
                        break;
                    case 2:  // 蓝色
                        ws2812_SetAll(0, 0, 255);
                        break;
                    case 3:  // 熄灭
                        ws2812_Clear();
                        break;
                }
                ws2812_Show();
            }
            break;
    }
    }

    

}

/**
  * @brief  测试模式
  */
void LED_Control_Test_Pattern(void)
{
    printf("[LED] Starting test pattern...\r\n");
    
    // 保存当前模式
    LedAlertMode_t saved_mode = led_state.current_mode;
    
    // 测试所有颜色
    uint8_t colors[][3] = {
        {255, 0, 0},    // 红
        {0, 255, 0},    // 绿
        {0, 0, 255},    // 蓝
        {255, 255, 0},  // 黄
        {255, 0, 255},  // 紫
        {0, 255, 255},  // 青
        {255, 255, 255} // 白
    };
    
    for (int i = 0; i < 7; i++)
    {
        ws2812_SetAll(colors[i][0], colors[i][1], colors[i][2]);
        ws2812_Show();
        HAL_Delay(500);
    }
    
    // 流水灯测试
    for (int i = 0; i < NUM_LEDS; i++)
    {
        ws2812_Clear();
        ws2812_SetPixel(i, 255, 255, 255);
        ws2812_Show();
        HAL_Delay(50);
    }
    
    ws2812_Clear();
    ws2812_Show();
    
    // 恢复之前模式
    LED_Control_Set_Mode(saved_mode);
    
    printf("[LED] Test pattern completed\r\n");
}

/**
  * @brief  距离梯度显示
  */
void LED_Control_Distance_Gradient(uint16_t distance_mm)
{
    if (distance_mm == 0 || distance_mm > 2000)
    {
        ws2812_Clear();
        ws2812_Show();
        return;
    }
    
    // 计算点亮的LED数量（距离越近，点亮越多）
    uint8_t active_leds = (2000 - distance_mm) * NUM_LEDS / 2000;
    if (active_leds > NUM_LEDS) active_leds = NUM_LEDS;
    
    // 清空灯带
    ws2812_Clear();
    
    // 根据距离设置颜色梯度
    for (int i = 0; i < active_leds; i++)
    {
        float ratio = (float)i / NUM_LEDS;
        
        if (distance_mm < 300)
        {
            // 红色到橙色
            uint8_t r = 255;
            uint8_t g = (uint8_t)(85 * ratio);
            uint8_t b = 0;
            ws2812_SetPixel(i, r, g, b);
        }
        else if (distance_mm < 800)
        {
            // 橙色到黄色
            uint8_t r = 255;
            uint8_t g = (uint8_t)(85 + 170 * ratio);
            uint8_t b = 0;
            ws2812_SetPixel(i, r, g, b);
        }
        else
        {
            // 黄色到绿色
            uint8_t r = (uint8_t)(255 * (1.0f - ratio));
            uint8_t g = 255;
            uint8_t b = 0;
            ws2812_SetPixel(i, r, g, b);
        }
    }
    
    ws2812_Show();
}