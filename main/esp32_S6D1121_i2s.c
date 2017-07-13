/*****************************************************
 * ESP32 I2S LCD Mode Library for S6D2212
 *
 * Originally provided by ESP_Sprite
 * https://www.esp32.com/download/file.php?id=656&sid=97b350bd63f99f0b4d11b951087f2244
 *
 * Modified by MinatsuT
 ****************************************************/

#include <esp_types.h>
#include <stdio.h>
#include "rom/ets_sys.h"
#include "rom/lldesc.h"
#include "rom/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/xtensa_api.h"

#include "soc/uart_reg.h"
#include "soc/dport_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/gpio_reg.h"
#include "soc/i2s_reg.h"

#include <math.h>

#include "driver/i2s.h"
#include "driver/gpio.h"

/* DMA related definitions. */
#define DMALEN 512 //Bug: >64 samples b0rks the i2s module for now.

uint16_t buf1[DMALEN];
uint16_t buf2[DMALEN];
uint16_t *buf=buf1;
uint16_t buf_cnt=0;

/* Prototypes. */
static void initI2SLcdMode(void);
static void LCD_Write_COM_DATA(int reg, int val);
static void LCD_Write_COM(uint16_t reg);
static void LCD_Write_DATA(uint16_t val);
static void writeWord(uint16_t dh, uint16_t dl);
static void writeByte(uint16_t dl);

static void sendBufDma(uint16_t *buf, int len);
static void finishDma(void);
static void flushDma(void);

static void initLCD(void);
static void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static void setXY(uint8_t x1, uint8_t y1);

/*****************************************************
 * I2S LCD Mode settings
 ****************************************************/
#define DPORT_I2S0_CLK_EN   (BIT(4))
#define DPORT_I2S0_RST   (BIT(4))
static void initI2SLcdMode() {
    /* Set IO_MUX function to GPIO */
    PIN_FUNC_SELECT(GPIO_PIN_REG_15,PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_2, PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_0, PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_4, PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_16,PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_17,PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_5, PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_18,PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_22,PIN_FUNC_GPIO);
    PIN_FUNC_SELECT(GPIO_PIN_REG_23,PIN_FUNC_GPIO);

    /* Assign I2S peripheral output to GPIO */
    gpio_matrix_out(15,I2S0O_DATA_OUT15_IDX,0,0); // MSB
    gpio_matrix_out( 2,I2S0O_DATA_OUT14_IDX,0,0);
    gpio_matrix_out( 0,I2S0O_DATA_OUT13_IDX,0,0);
    gpio_matrix_out( 4,I2S0O_DATA_OUT12_IDX,0,0);
    gpio_matrix_out(16,I2S0O_DATA_OUT11_IDX,0,0);
    gpio_matrix_out(17,I2S0O_DATA_OUT10_IDX,0,0);
    gpio_matrix_out( 5,I2S0O_DATA_OUT9_IDX ,0,0);
    gpio_matrix_out(18,I2S0O_DATA_OUT8_IDX ,0,0); // LSB
    gpio_matrix_out(22,I2S0O_WS_OUT_IDX    ,1,0); // WR (Write-strobe in 8080 mode, Active-low)
    gpio_matrix_out(23,I2S0O_DATA_OUT16_IDX,0,0); // RS (Command/Data select: 0=CMD, 1=DATA)

    /* Set I2S0 CLK Enable, and RST Disable */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG,DPORT_I2S0_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG,DPORT_I2S0_RST);

    //Reset I2S subsystem
    I2S0.conf.rx_reset=1; I2S0.conf.tx_reset=1;
    I2S0.conf.rx_reset=0; I2S0.conf.tx_reset=0;

    I2S0.conf2.val=0;
    I2S0.conf2.lcd_en=1;

    I2S0.sample_rate_conf.rx_bits_mod=16;
    I2S0.sample_rate_conf.tx_bits_mod=16;
    I2S0.sample_rate_conf.rx_bck_div_num=3;
    I2S0.sample_rate_conf.tx_bck_div_num=3;

    I2S0.clkm_conf.val=0;
    I2S0.clkm_conf.clka_en=1;
    I2S0.clkm_conf.clkm_div_a=0;
    I2S0.clkm_conf.clkm_div_b=0;
    I2S0.clkm_conf.clkm_div_num=8;

    //I2S0.fifo_conf.val=0;
    I2S0.fifo_conf.rx_fifo_mod=1;
    I2S0.fifo_conf.tx_fifo_mod=1;
    I2S0.fifo_conf.dscr_en=0;
    I2S0.fifo_conf.rx_data_num=32;
    I2S0.fifo_conf.tx_data_num=32;

    I2S0.conf1.val=0;
    I2S0.conf1.tx_stop_en=1;
    I2S0.conf1.tx_pcm_bypass=1;

    I2S0.conf_chan.val=0;
    I2S0.conf_chan.tx_chan_mod=1;
    I2S0.conf_chan.rx_chan_mod=1;

    /* WR edge should rise at the beginning of 2nd byte */
    I2S0.conf.tx_right_first=1;
    I2S0.conf.rx_right_first=1;
	
    I2S0.timing.val=0;
}

/*****************************************************
 * Primitive write functions
 ****************************************************/
//Add data into DMA buffer.
static void writeByte(uint16_t dl) {
    buf[buf_cnt++]=dl;
    if (buf_cnt==DMALEN) {
	flushDma();
    }
}

//Send two bytes.
//Byte order will be swapped because right channel is sent first.
static void writeWord(uint16_t dh, uint16_t dl) {
    writeByte(dl);
    writeByte(dh);
}

/*****************************************************
 * LCD Write functions
 ****************************************************/
//Set the index register in the LCD controller
static void LCD_Write_COM(uint16_t reg) {
    writeWord((reg>>8) & 0xff, reg & 0xff);
}

//Send data to the LCD controller
//high byte first, low second
static void LCD_Write_DATA(uint16_t val) {
    writeWord((val>>8)|0x100, (val & 0xff)|0x100);
}

//Set a register in the LCD controller
static void LCD_Write_COM_DATA(int reg, int val) {
    LCD_Write_COM(reg);
    LCD_Write_DATA(val);
}

/*****************************************************
 * DMA functions
 ****************************************************/
static volatile lldesc_t dmaDesc;
//Send a buffer to the LCD using DMA. Not very intelligent, sends only one buffer per time, doesn't
//chain descriptors.
//We should end up here after an lcdFlush, with the I2S peripheral reset and clean but configured for
//FIFO operation.
static void sendBufDma(uint16_t *buf, int len) {
    //Fill DMA descriptor
    dmaDesc.length=len*2;
    dmaDesc.size=len*2;
    dmaDesc.owner=1;
    dmaDesc.sosf=0;
    dmaDesc.buf=(uint8_t *)buf;
    dmaDesc.offset=0; //unused in hw
    dmaDesc.empty=0;
    dmaDesc.eof=1;

    //Reset DMA
    I2S0.lc_conf.in_rst=1; I2S0.lc_conf.out_rst=1; I2S0.lc_conf.ahbm_rst=1; I2S0.lc_conf.ahbm_fifo_rst=1;
    I2S0.lc_conf.in_rst=0; I2S0.lc_conf.out_rst=0; I2S0.lc_conf.ahbm_rst=0; I2S0.lc_conf.ahbm_fifo_rst=0;
	
    //Reset I2S FIFO
    I2S0.conf.tx_reset=1; I2S0.conf.tx_fifo_reset=1; I2S0.conf.rx_fifo_reset=1; 
    I2S0.conf.tx_reset=0; I2S0.conf.tx_fifo_reset=0; I2S0.conf.rx_fifo_reset=0; 

    //Set desc addr
    I2S0.out_link.addr=((uint32_t)(&dmaDesc))&I2S_OUTLINK_ADDR;

    I2S0.fifo_conf.dscr_en=1;

    //Enable and configure DMA
    I2S0.lc_conf.val=I2S_OUT_DATA_BURST_EN | I2S_CHECK_OWNER | I2S_OUT_EOF_MODE | I2S_OUTDSCR_BURST_EN|I2S_OUT_DATA_BURST_EN;
	

    //Start transmission
    I2S0.out_link.start=1;

    I2S0.conf.tx_start=1;
    //Clear int flags
    I2S0.int_clr.val=0xFFFFFFFF;
}

//Wait till current DMA transfer done
static void finishDma() {
    //No need to finish if no DMA transfer going on
    if (!I2S0.fifo_conf.dscr_en) return;

    //Wait till fifo done
    while(!(I2S0.int_raw.tx_rempty)) ;
    //Wait for last bytes to leave i2s xmit thing
    //ToDo: poll bit in next hw
    while (!(I2S0.state.tx_idle)) ;
	
    //Reset I2S for next transfer
    I2S0.conf.tx_start=0;
    I2S0.out_link.start=0;
	
    I2S0.conf.rx_reset=1; I2S0.conf.tx_reset=1;
    I2S0.conf.rx_reset=0; I2S0.conf.tx_reset=0;

    while(I2S0.state.tx_fifo_reset_back);
    I2S0.fifo_conf.dscr_en=0; //Disable DMA mode
}

//Flush current DMA ransfer, then switch to the next buffer.
static void flushDma() {
    finishDma();
    sendBufDma(buf, buf_cnt);
    if (buf==buf1) {
	buf=buf2;
    } else {
	buf=buf1;
    }
    buf_cnt=0;
}

/*****************************************************
 * LCD commands
 ****************************************************/
// Initialize
static void initLCD() {
    flushDma();

    // Reset LCD
    gpio_set_level(GPIO_NUM_19, 0);
    gpio_set_level(GPIO_NUM_19, 1);
    vTaskDelay(10/portTICK_PERIOD_MS);

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
    flushDma();

    setWindow(0,0,240-1,320-1);
    setXY(0,0);
    flushDma();
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
static const int qsintab[256]={
    0x8000,0x80c9,0x8192,0x825b,0x8324,0x83ee,0x84b7,0x8580,
    0x8649,0x8712,0x87db,0x88a4,0x896c,0x8a35,0x8afe,0x8bc6,
    0x8c8e,0x8d57,0x8e1f,0x8ee7,0x8fae,0x9076,0x913e,0x9205,
    0x92cc,0x9393,0x945a,0x9521,0x95e7,0x96ad,0x9773,0x9839,
    0x98fe,0x99c4,0x9a89,0x9b4d,0x9c12,0x9cd6,0x9d9a,0x9e5e,
    0x9f21,0x9fe4,0xa0a7,0xa169,0xa22b,0xa2ed,0xa3af,0xa470,
    0xa530,0xa5f1,0xa6b1,0xa770,0xa830,0xa8ef,0xa9ad,0xaa6b,
    0xab29,0xabe6,0xaca3,0xad5f,0xae1b,0xaed7,0xaf92,0xb04d,
    0xb107,0xb1c0,0xb27a,0xb332,0xb3ea,0xb4a2,0xb559,0xb610,
    0xb6c6,0xb77c,0xb831,0xb8e5,0xb999,0xba4d,0xbb00,0xbbb2,
    0xbc64,0xbd15,0xbdc6,0xbe76,0xbf25,0xbfd4,0xc082,0xc12f,
    0xc1dc,0xc288,0xc334,0xc3df,0xc489,0xc533,0xc5dc,0xc684,
    0xc72c,0xc7d3,0xc879,0xc91f,0xc9c3,0xca67,0xcb0b,0xcbae,
    0xcc4f,0xccf1,0xcd91,0xce31,0xced0,0xcf6e,0xd00b,0xd0a8,
    0xd144,0xd1df,0xd279,0xd313,0xd3ac,0xd443,0xd4db,0xd571,
    0xd606,0xd69b,0xd72f,0xd7c2,0xd854,0xd8e5,0xd975,0xda05,
    0xda93,0xdb21,0xdbae,0xdc3a,0xdcc5,0xdd4f,0xddd9,0xde61,
    0xdee9,0xdf6f,0xdff5,0xe07a,0xe0fd,0xe180,0xe202,0xe283,
    0xe303,0xe382,0xe400,0xe47d,0xe4fa,0xe575,0xe5ef,0xe668,
    0xe6e0,0xe758,0xe7ce,0xe843,0xe8b7,0xe92b,0xe99d,0xea0e,
    0xea7e,0xeaed,0xeb5b,0xebc8,0xec34,0xec9f,0xed09,0xed72,
    0xedda,0xee41,0xeea7,0xef0b,0xef6f,0xefd1,0xf033,0xf093,
    0xf0f2,0xf150,0xf1ad,0xf209,0xf264,0xf2be,0xf316,0xf36e,
    0xf3c4,0xf41a,0xf46e,0xf4c1,0xf513,0xf564,0xf5b3,0xf602,
    0xf64f,0xf69b,0xf6e6,0xf730,0xf779,0xf7c1,0xf807,0xf84d,
    0xf891,0xf8d4,0xf916,0xf956,0xf996,0xf9d4,0xfa11,0xfa4d,
    0xfa88,0xfac1,0xfafa,0xfb31,0xfb67,0xfb9c,0xfbd0,0xfc02,
    0xfc33,0xfc63,0xfc92,0xfcc0,0xfcec,0xfd17,0xfd42,0xfd6a,
    0xfd92,0xfdb8,0xfdde,0xfe01,0xfe24,0xfe46,0xfe66,0xfe85,
    0xfea3,0xfec0,0xfedb,0xfef5,0xff0e,0xff26,0xff3c,0xff52,
    0xff66,0xff79,0xff8a,0xff9b,0xffaa,0xffb8,0xffc4,0xffd0,
    0xffda,0xffe3,0xffeb,0xfff1,0xfff6,0xfffa,0xfffd,0xffff,
};

//Returns value -32K...32K
static const int isin(int i) {
    i=(i&1023);
    if (i>=512) return -isin(i-512);
    if (i>=256) i=(511-i);
    return qsintab[i]-0x8000;
}

static const int icos(int i) {
    return isin(i+256);
}


/*
  static int rotGetPixel(int x, int y, int rot) {
  int rx=(x*icos(rot)-y*isin(rot))>>16;
  int ry=(y*icos(rot)+x*isin(rot))>>16;

  if (((rx>>4)+(ry>>4))&1) return -1; else return 0xffff;
  }
*/

static void lcd_test1() {
    int i=0, f=0, x, y, p=0;
    int rcos, rsin;
    int zoom; //4.4 fixed point
    int px=-110;
    int py=-110;
    int cx, cxx, dxx, dxy, cy, cxy, dyx, dyy;

    printf("Running I2S test 1\n");

    for(f=0;f<200;f++) {
	if (!(f%10)) printf("Count %d\n",f);

	rcos=icos(f*8);
	rsin=isin(f*8);
	zoom=((isin(f*16)+0x8000)>>9)+32;
	px=(isin(f*3)>>7)-110;
	py=(icos(f*3)>>7)-110;

	LCD_Write_COM_DATA(0x20, 0); //H addr
	LCD_Write_COM_DATA(0x21, 0); //V addr1
	LCD_Write_COM(0x22);
	flushDma();
	i=0;

	cxx=(zoom*((px)*rcos-(py)*rsin))>>(22-8);
	cxy=(zoom*((py)*rcos+(px)*rsin))>>(22-8);

	dxx=(zoom*((1)*rcos-(0)*rsin))>>(22-8);
	dxy=(zoom*((0)*rcos+(1)*rsin))>>(22-8);
	dyx=(zoom*((0)*rcos-(1)*rsin))>>(22-8);
	dyy=(zoom*((1)*rcos+(0)*rsin))>>(22-8);

	for (y=0; y<320; y++) {
	    cx=cxx;
	    cy=cxy;
	    for (x=0; x<240; x++) {
		cx+=dyx;
		cy+=dyy;
		if ((cx^cy)&0x1000) p=0x1f; else p=0;
		if ((cx^cy)&0x2000) p|=0x3f<<5;
		if ((cx^cy)&0x4000) p|=0x1f<<11;
		LCD_Write_DATA(p);
	    }
	    cxx+=dxx;
	    cxy+=dxy;
	}
    }
}

#define RGB(r,g,b) ( ((((r)>>3)&0b11111)<<11) |  ((((g)>>2)&0b111111)<<5) |  ((((b)>>3)&0b11111)<<0) )
static void lcd_test2() {
    printf("Running I2S test 2\n");

    for(int f=0;f<200*2;f+=2) {
	if (!(f%10)) printf("Count %d\n",f);

	for(int y=0;y<0x140;y++) {
	    for(int x=0;x<0xF0;x++) {
		if ( ((0x140 + ((y-f)%0x140))%0x140) <8 || ( (0xF0 + ((x-f)%0xF0))%0xF0 ) <8 ) {
		    LCD_Write_DATA(0xffff);
		} else {
		    LCD_Write_DATA(RGB(x+y+f*1,x+f*2,y+f*3));
		}
	    }
	}
    }
}

static void lcd_test3() {
    printf("Running I2S test 3\n");

    for(int f=0;f<60;f++) {
	if (!(f%10)) printf("Count %d\n",f);

	for(int y=0;y<0x140;y++) {
	    for(int x=0;x<0xF0;x++) {
		LCD_Write_DATA(RGB(y*255/0x140,x*255/0xf0,0));
	    }
	}

	for(int y=0;y<0x140;y++) {
	    for(int x=0;x<0xF0;x++) {
		LCD_Write_DATA(RGB(0,y*255/0x140,x*255/0xf0));
	    }
	}

	for(int y=0;y<0x140;y++) {
	    for(int x=0;x<0xF0;x++) {
		LCD_Write_DATA(RGB(x*255/0xf0,0,y*255/0x140));
	    }
	}
    }
}

void i2s_lcd_test() {
    // Init I2S LCD Mode
    initI2SLcdMode();

    initLCD();
    flushDma();
    
    setWindow(0,0,240-1,320-1);
    setXY(0,0);

    lcd_test1();
    lcd_test2();
    lcd_test3();
}
