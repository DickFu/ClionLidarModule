#include "tim.h"
#include "stdint.h"
#include "string.h"
#include "ws2812.h"

#define Code0       30
#define Code1       60
#define CodeReset   0   

void ws2812_Update0()
{
    static uint16_t data[]={
        Code0, Code0, Code0, Code0, Code0, Code0, Code0, Code0, 
        Code0, Code0, Code0, Code0, Code0, Code0, Code0, Code0, 
        Code1, Code1, Code1, Code1, Code1, Code1, Code1, Code1, 
				CodeReset
    };
			HAL_TIM_PWM_Start_DMA(&htim4,TIM_CHANNEL_1,(uint32_t*)data,sizeof(data)/sizeof(uint16_t));
		
	}


static uint8_t led_buffer[NUM_LEDS][3]; 


static uint16_t dma_buffer[24 * NUM_LEDS + 50];  



void ws2812_SetPixel(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
    if(index < NUM_LEDS)
    {
        led_buffer[index][0] = green; 
        led_buffer[index][1] = red;
        led_buffer[index][2] = blue;
    }
}


void ws2812_SetAll(uint8_t red, uint8_t green, uint8_t blue)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        ws2812_SetPixel(i, red, green, blue);
    }
}


void ws2812_Clear(void)
{
    memset(led_buffer, 0, sizeof(led_buffer));
}


static void colorToBits(uint8_t color_value, uint16_t *buffer)
{
    for(int i = 7; i >= 0; i--)
    {
        if(color_value & (1 << i))
        {
            *buffer++ = Code1;  
        }
        else
        {
            *buffer++ = Code0;  
        }
    }
}


void ws2812_Show(void)
{
    uint16_t *buffer_ptr = dma_buffer;
    
    for(int led = 0; led < NUM_LEDS; led++)
    {
        colorToBits(led_buffer[led][0], buffer_ptr); 
        buffer_ptr += 8;
        
        colorToBits(led_buffer[led][1], buffer_ptr);  
        buffer_ptr += 8;
        
        colorToBits(led_buffer[led][2], buffer_ptr);
        buffer_ptr += 8;
    }
    
    for(int i = 0; i < 50; i++)
    {
        *buffer_ptr++ = CodeReset;
    }
    
    HAL_TIM_PWM_Start_DMA(&htim4, TIM_CHANNEL_1,
                         (uint32_t*)dma_buffer, 
                         (24 * NUM_LEDS) + 50);
}

void ws2812_RunningLight(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        ws2812_Clear();  
        ws2812_SetPixel(i, red, green, blue); 
        ws2812_Show(); 
        HAL_Delay(delay_ms);
    }
    
    for(int i = NUM_LEDS - 1; i >= 0; i--)
    {
        ws2812_SetPixel(i, 0, 0, 0); 
        ws2812_Show(); 
        HAL_Delay(delay_ms); 
    }
}


void ws2812_RainbowRunningLight(uint16_t delay_ms)
{
    static uint8_t hue = 0;
    
    for(int i = 0; i < NUM_LEDS; i++)
    {
        uint8_t r, g, b;
        uint8_t h = (hue + i * 30) % 255; 
        
        if(h < 85)
        {
            r = 255 - h * 3;
            g = h * 3;
            b = 0;
        }
        else if(h < 170)
        {
            h -= 85;
            r = 0;
            g = 255 - h * 3;
            b = h * 3;
        }
        else
        {
            h -= 170;
            r = h * 3;
            g = 0;
            b = 255 - h * 3;
        }
        
        ws2812_Clear(); 
        ws2812_SetPixel(i, r, g, b); 
        ws2812_Show();
        HAL_Delay(delay_ms); 
    }
    
    hue += 10;
}
//彩虹
void test_Rainbow(void)
{
    for(int i = 0; i < 10; i++)
    {
        ws2812_RainbowRunningLight(80); 
    }
}
//流水灯红
void test_RunningLightsR(void)
{
    for(int i = 0; i < 3; i++)
    {
        ws2812_RunningLight(255, 0, 0, 100);  
    }
    
}
//流水灯绿
void test_RunningLightsG(void)
{
    
    for(int i = 0; i < 3; i++)
    {
        ws2812_RunningLight(0, 255, 0, 100);  
    }
    
}
//流水灯白
void test_RunningLightsW(void)
{
    for(int i = 0; i < 3; i++)
    {
        ws2812_RunningLight(255, 255, 255, 100); 
    }
}

/**
  * @brief  WS2812更新函数（兼容性）
  */
void ws2812_Update(void)
{
    ws2812_Show();
}

// 修改函数名以匹配头文件声明
void ws2812_RunningLightR(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms)
{
    ws2812_RunningLight(red, green, blue, delay_ms);
}

void ws2812_RunningLightG(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms)
{
    ws2812_RunningLight(red, green, blue, delay_ms);
}

void ws2812_RunningLightW(uint8_t red, uint8_t green, uint8_t blue, uint16_t delay_ms)
{
    ws2812_RunningLight(red, green, blue, delay_ms);
}
