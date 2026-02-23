//
// Created by FuShenzhe on 2026/2/12.
//
/***********************************************
 * LD14雷达驱动 - HAL库版本
 * 硬件：USART3, PB10(TX), PB11(RX)
 * 适配：CubeMX生成的DMA+Idle中断框架
 * 核心修改：1. 匹配lidar.h结构体 2. 修复解析偏移 3. 温和过滤噪声
 ***********************************************/
#include "lidar.h"
#include <string.h>


// 雷达数据存储
LiDARFrameTypeDef Pack_Data;
LidarPointStructDef Dataprocess[800];
LidarPointStructDef PointDataProcess[800];

// 全局变量
volatile uint8_t data_process_flag = 0, data_flag = 0;
volatile uint16_t data_cnt = 0;
volatile uint16_t receive_cnt = 0;

// CRC校验表（保持不变）
static const uint8_t CrcTable[256] =
{
 0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3,
 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8,
 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77,
 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55,
 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4,
 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f,
 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff,
 0xb2, 0x1c, 0x51, 0x86, 0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2,
 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12,
 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99,
 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14,
 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36,
 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9,
 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72,
 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2,
 0xef, 0x41, 0x0c, 0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1,
 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71,
 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa,
 0xb7, 0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35,
 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};//用于crc校验的数组



/**
  * @brief  初始化UART3用于LD14雷达
  * @param  bound: 波特率
  * @retval HAL状态
  */
HAL_StatusTypeDef lidar_uart3_init(uint32_t bound)
{
    LIDAR_Init();
    return HAL_OK;
}

/**
  * @brief  雷达变量初始化
  */
void LIDAR_Init(void)
{
    // 简单初始化变量
    data_cnt = 0;
    receive_cnt = 0;
    data_flag = 0;
    data_process_flag = 0;
    memset(&Pack_Data, 0, sizeof(Pack_Data));
    memset(Dataprocess, 0, sizeof(Dataprocess));
    memset(PointDataProcess, 0, sizeof(PointDataProcess));
}

void LIDAR_ReceiveCallback(uint8_t data)
{
    // 空实现，因为你的数据处理在中断中直接进行
    // 这个函数不会被调用
}

/**
  * @brief  STM32向雷达发送数据（PB10 → 雷达RX）
  * @param  data: 指令数组
  * @param  size: 指令长度
  */
void lidar_send_data(uint8_t* data, uint16_t size)
{
    HAL_UART_Transmit(&huart3, data, size, 500); // 超时500ms，避免阻塞
}

/**
  * @brief  私有函数：计算CRC8校验值
  * @param  data: 待校验数据
  * @param  len: 数据长度
  * @retval CRC8结果
  */
static uint8_t lidar_calc_crc8(uint8_t* data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = CrcTable[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/**
  * @brief  解析DMA接收的雷达数据（核心修改：批量解析）
  * @param  data: DMA接收缓冲区（uart3_rx_buffer）
  * @param  len: Idle中断返回的接收长度
  */
void lidar_parse_data(uint8_t* data, uint16_t len)
{
    if (len < LD14_FRAME_LEN) return; // 长度不足47字节，直接返回

    // 遍历缓冲区，寻找所有合法帧
    for (uint16_t i = 0; i <= len - LD14_FRAME_LEN; i++) {
        // 1. 校验帧头（统一用LD14_HEADER，兼容LD14/LD14P）
        if (data[i] != LD14_HEADER) continue;
        // 2. 校验长度字段
        if (data[i+1] != LD14_LENGTH) continue;

        // 3. CRC校验（前46字节 → 第47字节）
        uint8_t calc_crc = lidar_calc_crc8(&data[i], LD14_FRAME_LEN - 1);
        if (calc_crc != data[i + LD14_FRAME_LEN - 1]) continue;

        // 4. 解析帧数据（严格匹配lidar.h的LiDARFrameTypeDef）
        Pack_Data.header = data[i];
        Pack_Data.ver_len = data[i+1];
        Pack_Data.speed = (uint16_t)data[i+3] << 8 | data[i+2];          // 转速：高8+低8
        Pack_Data.start_angle = (uint16_t)data[i+5] << 8 | data[i+4];    // 起始角度：高8+低8

        // 解析12个原始点（仅距离+置信度，角度在data_process中计算）
        for (int j = 0; j < 12; j++) {
            uint16_t offset = i + 6 + j*3; // 每个点3字节：距离低8、高8、置信度
            Pack_Data.point[j].distance = (uint16_t)data[offset+1] << 8 | data[offset];
            Pack_Data.point[j].confidence = data[offset+2];
        }

        // 解析结束角度、时间戳、CRC
        Pack_Data.end_angle = (uint16_t)data[i+43] << 8 | data[i+42];    // 结束角度：高8+低8
        Pack_Data.timestamp = (uint16_t)data[i+45] << 8 | data[i+44];    // 时间戳：高8+低8
        Pack_Data.crc8 = data[i+46];

        // 5. 调用数据处理（计算角度+存储）
        lidar_data_process();
        receive_cnt++;
        data_process_flag = 1;
        break; // 找到一帧后退出，避免重复解析
    }
}

/**
  * @brief  原有数据处理逻辑（保持不变）
  */
void lidar_data_process(void)
{
    int n;
    float start_angle = Pack_Data.start_angle / 100.0f; // 转换为°（原始是0.01°单位）
    float end_angle = Pack_Data.end_angle / 100.0f;
    float area_angle[12] = {0};

    // 角度跨360°处理（比如起始350°，结束10°）
    if (start_angle > end_angle) {
        end_angle += 360.0f;
    }

    data_process_flag = 0;

    // 计算每个点的实际角度
    for (int m = 0; m < 12; m++) {
        area_angle[m] = start_angle + (end_angle - start_angle) / 12.0f * m;
        if (area_angle[m] > 360.0f) {
            area_angle[m] -= 360.0f;
        }
    }

    //存储解析后的点数据
    for (n = 0; n < 12; n++) {
        Dataprocess[data_cnt + n].angle = area_angle[n];

        // 温和过滤：只剔除无效点（减少散点）
        if (Pack_Data.point[n].distance > 0 && Pack_Data.point[n].confidence > 20) {
            Dataprocess[data_cnt + n].distance = Pack_Data.point[n].distance;
            Dataprocess[data_cnt + n].confidence = Pack_Data.point[n].confidence;
        } else {
            Dataprocess[data_cnt + n].distance = 0;
            Dataprocess[data_cnt + n].confidence = 0;
        }
    }
    // for (n = 0; n < 12; n++) {
    //     Dataprocess[data_cnt + n].angle = area_angle[n];
    //     Dataprocess[data_cnt + n].distance = Pack_Data.point[n].distance;
    //     Dataprocess[data_cnt + n].confidence = Pack_Data.point[n].confidence;
    // }

    data_cnt += 12;
    if (data_cnt >= 720) {  // 一圈约720个点（60帧×12点）
        data_cnt = 0;
        data_flag = 1; // 标记：一圈数据就绪
    }
}

/**
  * @brief  获取雷达数据就绪标志
  * @retval 1=一圈数据就绪，0=未就绪
  */
uint8_t lidar_data_ready(void)
{
    return data_flag;
}

/**
  * @brief  重置雷达数据标志
  */
void lidar_reset_data_flag(void)
{
    data_flag = 0;
}

// /**
//   * @brief  基于状态机实现的串口接收中断处理函数
//   */
// void lidar_uart_rx_handler(void)
// {
//     static uint8_t state = 0;     // 状态机状态
//     static uint8_t crc = 0;       // 校验和
//     static uint8_t cnt = 0;       // 点数据计数
//     uint8_t temp_data;
//
//     // 检查是否有接收中断
//     if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET) {
//         // 读取数据
//         temp_data = (uint8_t)(huart3.Instance->DR & 0xFF);
//
//         // 状态机处理（保持原有的逻辑）
//         if (state > 5) {
//             if (state < 42) {
//                 if (state % 3 == 0) {  // 距离低8位
//                     Pack_Data.point[cnt].distance = (uint16_t)temp_data;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                 }
//                 else if (state % 3 == 1) {  // 距离高8位
//                     Pack_Data.point[cnt].distance = ((uint16_t)temp_data << 8) +
//                                                      Pack_Data.point[cnt].distance;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                 }
//                 else {  // 置信度
//                     Pack_Data.point[cnt].confidence = temp_data;
//                     cnt++;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                 }
//             } else {
//                 switch (state) {
//                     case 42:  // 结束角度低8位
//                         Pack_Data.end_angle = (uint16_t)temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                         break;
//
//                     case 43:  // 结束角度高8位
//                         Pack_Data.end_angle = ((uint16_t)temp_data << 8) +
//                                               Pack_Data.end_angle;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                         break;
//
//                     case 44:  // 时间戳低8位
//                         Pack_Data.timestamp = (uint16_t)temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                         break;
//
//                     case 45:  // 时间戳高8位
//                         Pack_Data.timestamp = ((uint16_t)temp_data << 8) +
//                                               Pack_Data.timestamp;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                         break;
//
//                     case 46:  // CRC校验
//                         Pack_Data.crc8 = temp_data;
//                         if (Pack_Data.crc8 == crc) {  // 校验正确
//                             lidar_data_process();  // 处理数据
//                             receive_cnt++;
//                             data_process_flag = 1;
//                         } else {
//                             memset(&Pack_Data, 0, sizeof(Pack_Data));  // 清零
//                         }
//                         crc = 0;
//                         state = 0;
//                         cnt = 0;
//                         break;
//
//                     default:
//                         break;
//                 }
//             }
//         } else {
//             switch (state) {
//                 case 0:  // 帧头
//                     if (temp_data == HEADER) {
//                         Pack_Data.header = temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                     } else {
//                         state = 0;
//                         crc = 0;
//                     }
//                     break;
//
//                 case 1:  // 长度
//                     if (temp_data == LENGTH) {
//                         Pack_Data.ver_len = temp_data;
//                         state++;
//                         crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                     } else {
//                         state = 0;
//                         crc = 0;
//                     }
//                     break;
//
//                 case 2:  // 转速低8位
//                     Pack_Data.speed = (uint16_t)temp_data;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                     break;
//
//                 case 3:  // 转速高8位
//                     Pack_Data.speed = ((uint16_t)temp_data << 8) + Pack_Data.speed;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                     break;
//
//                 case 4:  // 开始角度低8位
//                     Pack_Data.start_angle = (uint16_t)temp_data;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                     break;
//
//                 case 5:  // 开始角度高8位
//                     Pack_Data.start_angle = ((uint16_t)temp_data << 8) +
//                                            Pack_Data.start_angle;
//                     state++;
//                     crc = CrcTable[(crc ^ temp_data) & 0xFF];
//                     break;
//
//                 default:
//                     break;
//             }
//         }
//     }
// }

/**
  * @brief  绝对值函数
  */
float float_abs(float input)
{
    return (input < 0) ? -input : input;
}
