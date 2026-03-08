#ifndef WS2812_H
#define WS2812_H

#include "main.h"

#define NUM_LEDS    5

void ws2812_Init(void);
void ws2812_SetPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
void ws2812_SetAll(uint8_t red, uint8_t green, uint8_t blue);
void ws2812_Clear(void);
void ws2812_Show(void);

void ws2812_RunningLightR(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms);
void ws2812_RunningLightG(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms);
void ws2812_RunningLightW(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms);
void ws2812_RainbowRunningLight(uint16_t delay_ms);

#endif