//
// Created by FuShenzhe on 2026/2/12.
//

#ifndef LIDARMODULE_LIDAR_H
#define LIDARMODULE_LIDAR_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "usart.h"

// 数据包定义
#define LD14_HEADER     0x54    // LD14/LD14P 统一帧头
#define LD14_LENGTH     0x2C    // 帧长度字段
#define LD14_FRAME_LEN  47      // 一帧固定47字节
#define LD14_TIMEOUT_MS 200     //雷达超时时间（200ms）

// 雷达单点数据结构体
typedef struct {
    uint16_t distance;   // 距离(mm)
    uint8_t confidence;  // 置信度
} LidarRawPointDef;

// 雷达完整帧结构体
typedef struct {
    uint8_t header;              // 帧头 0x54
    uint8_t ver_len;             // 版本+长度 0x2C
    uint16_t speed;              // 转速 (0.1°/s)
    uint16_t start_angle;        // 起始角度 (0.01°)
    LidarRawPointDef point[12]; // 12个点数据
    uint16_t end_angle;          // 结束角度 (0.01°)
    uint16_t timestamp;          // 时间戳 (ms)
    uint8_t crc8;                // CRC8校验
} LiDARFrameTypeDef;


// 处理后的点数据
typedef struct {
    float angle;           // 角度
    uint16_t distance;     //距离
    uint8_t confidence;    //置信度
} LidarPointStructDef;

// 外部变量声明
extern LiDARFrameTypeDef Pack_Data;
extern LidarPointStructDef Dataprocess[800];
extern LidarPointStructDef PointDataProcess[800];
extern volatile uint8_t data_process_flag, data_flag;
extern volatile uint16_t data_cnt, receive_cnt;
// 函数声明
HAL_StatusTypeDef lidar_uart3_init(uint32_t bound);
void LIDAR_Init(void);
void lidar_send_data(uint8_t* data, uint16_t size);
void lidar_parse_data(uint8_t* data, uint16_t len);
uint8_t lidar_data_ready(void);
void lidar_reset_data_flag(void);
void lidar_data_process(void);
float float_abs(float input);
#endif //LIDARMODULE_LIDAR_H