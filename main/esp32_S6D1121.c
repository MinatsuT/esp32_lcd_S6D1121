/*****************************************************
 * ESP32 S6D1121 LCD Driver
 * Written by MinatsuT
 ****************************************************/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/cpu.h"
#include "esp32_S6D1121.h"

/* i2s_lcd.c */
void i2s_lcd_test(void);

#define CMD 0
#define DATA 1

#define RGB(r,g,b) ( (((r>>3)&0b11111)<<11) |  (((g>>2)&0b111111)<<5) |  (((b>>3)&0b11111)<<0) )

static void strobe(gpio_num_t GPIO_NUM, uint32_t s_trig, uint32_t s_normal);
static void initGPIO(void);
static void setWriteDir(void);
static void setReadDir(void);
static void writeLCD(uint8_t VL, uint8_t isData);
static uint16_t LCD_Read_DATA(uint8_t reg);
static void LCD_Write_COM(uint8_t reg);
static void LCD_Write_DATA(uint16_t dat);
static void LCD_Write_COM_DATA(uint8_t reg, uint16_t dat);
static void initLCD(void);
static void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void setXY(uint8_t x1, uint8_t y1);

/*****************************************************
 * GPIO functions
 ****************************************************/
static void initGPIO() {
    gpio_pad_select_gpio(D0);
    gpio_pad_select_gpio(D1);
    gpio_pad_select_gpio(D2);
    gpio_pad_select_gpio(D3);
    gpio_pad_select_gpio(D4);
    gpio_pad_select_gpio(D5);
    gpio_pad_select_gpio(D6);
    gpio_pad_select_gpio(D7);

    gpio_pad_select_gpio(RD);
    gpio_pad_select_gpio(RS);
    gpio_pad_select_gpio(WR);
    gpio_pad_select_gpio(RS);

    gpio_set_direction(RD, GPIO_MODE_OUTPUT);
    gpio_set_direction(RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(WR, GPIO_MODE_OUTPUT);
    gpio_set_direction(RS, GPIO_MODE_OUTPUT);

    gpio_set_level(RD, 1);
    gpio_set_level(RST, 1);
    gpio_set_level(WR, 1);
    gpio_set_level(RS, 1);

    setWriteDir();
}

static void strobe(gpio_num_t GPIO_NUM, uint32_t s_trig, uint32_t s_normal) {
    gpio_set_level(GPIO_NUM, s_trig);
    gpio_set_level(GPIO_NUM, s_normal);
}

static void setWriteDir() {
    gpio_set_direction(D0, GPIO_MODE_OUTPUT);
    gpio_set_direction(D1, GPIO_MODE_OUTPUT);
    gpio_set_direction(D2, GPIO_MODE_OUTPUT);
    gpio_set_direction(D3, GPIO_MODE_OUTPUT);
    gpio_set_direction(D4, GPIO_MODE_OUTPUT);
    gpio_set_direction(D5, GPIO_MODE_OUTPUT);
    gpio_set_direction(D6, GPIO_MODE_OUTPUT);
    gpio_set_direction(D7, GPIO_MODE_OUTPUT);
}

static void setReadDir() {
    gpio_set_direction(D0, GPIO_MODE_INPUT);
    gpio_set_direction(D1, GPIO_MODE_INPUT);
    gpio_set_direction(D2, GPIO_MODE_INPUT);
    gpio_set_direction(D3, GPIO_MODE_INPUT);
    gpio_set_direction(D4, GPIO_MODE_INPUT);
    gpio_set_direction(D5, GPIO_MODE_INPUT);
    gpio_set_direction(D6, GPIO_MODE_INPUT);
    gpio_set_direction(D7, GPIO_MODE_INPUT);

    gpio_pullup_dis(D0);
    gpio_pullup_dis(D1);
    gpio_pullup_dis(D2);
    gpio_pullup_dis(D3);
    gpio_pullup_dis(D4);
    gpio_pullup_dis(D5);
    gpio_pullup_dis(D6);
    gpio_pullup_dis(D7);

    gpio_pulldown_dis(D0);
    gpio_pulldown_dis(D1);
    gpio_pulldown_dis(D2);
    gpio_pulldown_dis(D3);
    gpio_pulldown_dis(D4);
    gpio_pulldown_dis(D5);
    gpio_pulldown_dis(D6);
    gpio_pulldown_dis(D7);
}


/*****************************************************
 * Primitive read/write functions
 ****************************************************/
static void writeByte(uint8_t VL, uint8_t isData) {
    gpio_set_level(RS, isData);

    gpio_set_level(D0, (VL>>0) & 1);
    gpio_set_level(D1, (VL>>1) & 1);
    gpio_set_level(D2, (VL>>2) & 1);
    gpio_set_level(D3, (VL>>3) & 1);
    gpio_set_level(D4, (VL>>4) & 1);
    gpio_set_level(D5, (VL>>5) & 1);
    gpio_set_level(D6, (VL>>6) & 1);
    gpio_set_level(D7, (VL>>7) & 1);

    strobe(WR,0,1);
}

static uint8_t readByte() {
    gpio_set_level(RS, 1); // set RS to Data

    gpio_set_level(RD, 0);
    uint32_t dat=
	(gpio_get_level(D0) << 0) |
	(gpio_get_level(D1) << 1) |
	(gpio_get_level(D2) << 2) |
	(gpio_get_level(D3) << 3) |
	(gpio_get_level(D4) << 4) |
	(gpio_get_level(D5) << 5) |
	(gpio_get_level(D6) << 6) |
	(gpio_get_level(D7) << 7) ;
    gpio_set_level(RD, 1);

    return dat;
}

/*****************************************************
 * LCD Read/Write functions
 ****************************************************/
static uint16_t LCD_Read_DATA(uint8_t reg) {
    LCD_Write_COM(reg);

    setReadDir();
    uint16_t VH=readByte();
    uint16_t VL=readByte();
    setWriteDir();

    return VH<<8 | VL;
}

static void LCD_Write_COM(uint8_t reg) {
    writeByte(0x00,CMD);
    writeByte(reg,CMD);
}

static void LCD_Write_DATA(uint16_t dat) {
    writeByte(dat >> 8, DATA);
    writeByte(dat & 0xff ,DATA);
}

static void LCD_Write_COM_DATA(uint8_t reg, uint16_t dat) {
    LCD_Write_COM(reg);
    LCD_Write_DATA(dat);
}

/*****************************************************
 * LCD commands
 ****************************************************/
static void initLCD() {
    LCD_Write_COM_DATA(0x11,0x2004);		
    LCD_Write_COM_DATA(0x13,0xCC00);		
    LCD_Write_COM_DATA(0x15,0x2600);	
    LCD_Write_COM_DATA(0x14,0x252A);	
    LCD_Write_COM_DATA(0x12,0x0033);		
    LCD_Write_COM_DATA(0x13,0xCC04);		
    LCD_Write_COM_DATA(0x13,0xCC06);		
    LCD_Write_COM_DATA(0x13,0xCC4F);		
    LCD_Write_COM_DATA(0x13,0x674F);
    LCD_Write_COM_DATA(0x11,0x2003);
    LCD_Write_COM_DATA(0x30,0x2609);		
    LCD_Write_COM_DATA(0x31,0x242C);		
    LCD_Write_COM_DATA(0x32,0x1F23);		
    LCD_Write_COM_DATA(0x33,0x2425);		
    LCD_Write_COM_DATA(0x34,0x2226);		
    LCD_Write_COM_DATA(0x35,0x2523);		
    LCD_Write_COM_DATA(0x36,0x1C1A);		
    LCD_Write_COM_DATA(0x37,0x131D);		
    LCD_Write_COM_DATA(0x38,0x0B11);		
    LCD_Write_COM_DATA(0x39,0x1210);		
    LCD_Write_COM_DATA(0x3A,0x1315);		
    LCD_Write_COM_DATA(0x3B,0x3619);		
    LCD_Write_COM_DATA(0x3C,0x0D00);		
    LCD_Write_COM_DATA(0x3D,0x000D);		
    LCD_Write_COM_DATA(0x16,0x0007);		
    LCD_Write_COM_DATA(0x02,0x0013);		
    LCD_Write_COM_DATA(0x03,0x0003);		
    LCD_Write_COM_DATA(0x01,0x0127);		
    LCD_Write_COM_DATA(0x08,0x0303);		
    LCD_Write_COM_DATA(0x0A,0x000B);		
    LCD_Write_COM_DATA(0x0B,0x0003);   
    LCD_Write_COM_DATA(0x0C,0x0000);   
    LCD_Write_COM_DATA(0x41,0x0000);    
    LCD_Write_COM_DATA(0x50,0x0000);   
    LCD_Write_COM_DATA(0x60,0x0005);    
    LCD_Write_COM_DATA(0x70,0x000B);    
    LCD_Write_COM_DATA(0x71,0x0000);    
    LCD_Write_COM_DATA(0x78,0x0000);    
    LCD_Write_COM_DATA(0x7A,0x0000);   
    LCD_Write_COM_DATA(0x79,0x0007);		
    LCD_Write_COM_DATA(0x07,0x0051);   
    LCD_Write_COM_DATA(0x07,0x0053);		
    LCD_Write_COM_DATA(0x79,0x0000);
    LCD_Write_COM(0x22);
}

static void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    LCD_Write_COM_DATA(0x46,(x2 << 8) | x1);
    LCD_Write_COM_DATA(0x47,y2);
    LCD_Write_COM_DATA(0x48,y1);
    LCD_Write_COM(0x22);
}

static void setXY(uint8_t x1, uint8_t y1) {
    LCD_Write_COM_DATA(0x20,x1);
    LCD_Write_COM_DATA(0x21,y1);
    LCD_Write_COM(0x22);
}

/*****************************************************
 * Test codes
 ****************************************************/
void lcd_test() {
    // Init GPIO
    initGPIO();

    // Reset LCD
    gpio_set_level(RST, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);
    gpio_set_level(RST, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    initLCD();

    printf("Reading ID...\n");
    uint16_t V = LCD_Read_DATA(0);
    printf("ID=%04x\n",V);

    setWindow(0,0,240-1,320-1);
    setXY(0,0);

    printf("Running GPIO test 1\n");
    for(int f=0;f<4;f++) {
	if (!(f%10)) printf("Count %d\n",f);

	for(int i=0;i<1;i++) {
	    for(int y=0;y<320;y++) {
		for(int x=0;x<240;x++) {
		    LCD_Write_DATA(RGB(y*255/320,x*255/0xf0,0));
		}
	    }
	    for(int y=0;y<320;y++) {
		for(int x=0;x<240;x++) {
		    LCD_Write_DATA(RGB(0,y*255/320,x*255/0xf0));
		}
	    }
	    for(int y=0;y<320;y++) {
		for(int x=0;x<240;x++) {
		    LCD_Write_DATA(RGB(x*255/0xf0,0,y*255/320));
		}
	    }
	}
    }
}
