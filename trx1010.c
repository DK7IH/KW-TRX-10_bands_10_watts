///////////////////////////////////////////////////////////////////
//           Software for TRX110 (10 band SW radio)              //  
//           MCU: STM32F411 MCU                                  //
//           ILI9341 SPI                                         //
//           8 bit parallel mode, 4 CTRLlines                    //
//           DDS: AD9951 (75 * 4 MHz clock)                      //
///////////////////////////////////////////////////////////////////
//                                                               //
//  Compiler:         GCC (GNU AVR C-Compiler)                   //
//  Author:           Peter Baier (DK7IH)                        //
//                    http;//dk7ih.de                            //
//                    OCT-2019                                   // 
///////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
 
//Ports
//PA0..PA3:        LCD                        (checked)
//PA4:             ADC Keys input
//PA5:             TEMP PA  meas.
//PA6:             VDD meas.
//PA7:             reserved ADC input
//PA8..PA12, PA15: reserved
//PB0, PB1:        Rotary encoder             (checked)
//PB2..PB5:        Relay drivers A0..A3       (checked)
//PB6, PB9:        I2C
//PB7:             TX detect 
//PB8:             Reserved
//PB10:            Reserved
//PB12..PB15:      DDS						  (checked)
//PC14..PC15:      Reserved

//Radio defines
//Modes, bands etc.
#define MAXVFO 12
#define MAXMODES 2
#define MAXBANDS 10
#define INTERFREQUENCY 9000000

//I2C
#define EEPROM_ADR   0xA0
#define MCP4725_ADR  0xC0; //C0 with "ADDR" on chip open or GND, C1 with tied to VDD;
  
////////////////
//Declarations//
//////////////// 
//MISC
int main(void);

//ADC defines
#define KEYS  1 
#define VDD   2 
#define TMP   3

/*
// PIN definitions of Nokia 5110 lines on PORT D
#define LCDGPIO GPIOA
#define RST 3 //yellow
#define DC  2 //grey
#define DIN 1 //green
#define CLK 0 //blue
*/

//SPI AD9951 defines
#define DDS_GPIO GPIOB
#define DDS_IO_UD   12   //violet
#define DDS_SDIO    13   //green
#define DDS_SCLK    14   //white
#define DDS_RESET   15   //grey

//LCD ST7735
//SPI defines
#define LCD_GPIO GPIOA
#define SCK 0 //blue  "SCK" 
#define SDA 1 //green "SDI"
#define DC  2 //gray 
#define RES 3 //yellow
 
#define LCD_CMD   0
#define LCD_DATA  1

//Colors RGB 565
//http://www.barth-dev.de/online/rgb565-color-picker/
#define LCD_WIDTH   160
#define LCD_HEIGHT  128	

#define WHITE        0xFFFF
#define BLACK        0x0000
#define GRAY         0x94B2
#define LIGHTGRAY    0xC5D7
#define LIGHTBLUE    0x755C
#define BLUE         0x3C19
#define DARKBLUE     0x0887
#define DARKBLUE2    0x0844
#define LIGHTRED     0xF802
#define RED          0xD126
#define DARKRED      0xC124
#define LIGHTGREEN   0x27E0
#define GREEN        0x1CC5
#define DARKGREEN    0x1A04
#define LIGHTVIOLET  0xAC19
#define LIGHTVIOLET2 0x9BD9
#define VIOLET       0x71B6
#define DARKVIOLET   0x48AF
#define YELLOW       0xFFC3
#define DARKYELLOW   0xDF41
#define LIGHTORANGE  0xFB80
#define LIGHTYELLOW  0xF7D1
#define LIGHTBROWN   0xF64F
#define BROWN        0x9323
#define DARKBROWN    0x6222
int bgcolor = BLACK; //DARKBLUE2; //Standard Background Color

//ILI9341 LCD Basic Functions
void lcd_init(void);
void lcd_send(int, int);                           //Write a byte of data via SPI
void lcd_set_xy(int, int);
void lcd_cls(int);
void lcd_draw_pixel(int);
void lcd_putchar(int, int, int, int, int, int);
int lcd_putnumber(int, int, long, int, int, int, int);
void lcd_putstring(int, int, char*, int, int, int);

//DATA DISPLAY ROUTINES
void show_frequency(unsigned long);
void show_sideband(int); 
void show_band(int); 
void show_vfo(int);
void show_pa_temp(int);
void show_vdd(int);
void show_txrx(int);
void show_tunspeed(int);
void show_msg(char*);

//STRING FUNCTIONS
int int2asc(long, int, char*, int);

//ADC
int get_adc(int);
int get_keys(void);
int get_pa_temp(void);
int get_vdd(void);
int get_txpwr(void);
int get_txrx(void);

//Radio
void spi_send_byte(unsigned int);
void set_frequency(long);
void set_clock_multiplier(void);
void set_band_relay(int);

//Interrupt handlers
extern "C" void EXTI0_IRQHandler(void);
extern "C" void TIM2_IRQHandler(void);

//EEPROM etc.
int eeprom_read(int mem_address);
void eeprom_write(int, int);
uint8_t eeprom_read(uint16_t);
void store_frequency(long, int, int);
long load_frequency(int, int);
int load_last_band(void);
void store_last_band(int);
void store_last_vfo(int);
int load_last_vfo(void);
void eeprom_erase(void);
int is_freq_ok(long, int);

//I2C
void i2c_start(void);
void i2c_stop(void);
void i2c_write_byte1(uint8_t, uint8_t, int);
void i2c_write_byte2(uint8_t*, uint8_t, int);
int16_t i2c_read1(uint8_t, int); 
int16_t i2c_read2(uint16_t, int); 
void mcp4725(uint8_t, uint8_t, uint8_t);
 

//MISC
static void delay(unsigned int);

  /////////////////////////
 //     Variables       //
/////////////////////////
//Font
#define FONTWIDTH 12 
#define FONTHEIGHT 16
//Font 12x16 vert. 
const char xchar[][24]={
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x3    ,0xf0   ,0xc    ,0xc    ,0x10   ,0x2    ,0x11   ,0x32   ,0x22   ,0x31   ,0x22   ,0x1    ,0x22   ,0x31   ,0x11   ,0x32   ,0x10   ,0x2    ,0xc,0xc     ,0x3    ,0xf0   },
{0x3    ,0xf0   ,0xf    ,0xfc   ,0x1f   ,0xfe   ,0x1e   ,0xce   ,0x3d   ,0xcf   ,0x3d   ,0xff   ,0x3d   ,0xcf   ,0x1e   ,0xce   ,0x1f   ,0xfe   ,0xf,0xfc    ,0x3    ,0xf0   },
{0x0    ,0x0    ,0x0    ,0xf0   ,0x1    ,0xf8   ,0x3    ,0xf8   ,0x7    ,0xf0   ,0xf    ,0xe0   ,0x7    ,0xf0   ,0x3    ,0xf8   ,0x1    ,0xf8   ,0x0,0xf0    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x80   ,0x1    ,0xc0   ,0x3    ,0xe0   ,0x7    ,0xf0   ,0xf    ,0xf8   ,0x7    ,0xf0   ,0x3    ,0xe0   ,0x1    ,0xc0   ,0x0,0x80    ,0x0    ,0x0    },
{0x3    ,0x80   ,0x7    ,0xc0   ,0x7    ,0xc0   ,0x13   ,0xb8   ,0x1b   ,0xfc   ,0x1f   ,0xfc   ,0x1b   ,0xfc   ,0x13   ,0xb8   ,0x7    ,0xc0   ,0x7,0xc0    ,0x3    ,0x80   },
{0x0    ,0x0    ,0x3    ,0x80   ,0x7    ,0xc0   ,0x17   ,0xe0   ,0x1b   ,0xf0   ,0x1f   ,0xfc   ,0x1b   ,0xf0   ,0x17   ,0xe0   ,0x7    ,0xc0   ,0x3,0x80    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x78   ,0x0    ,0x18   ,0x3    ,0xa8   ,0x7    ,0xc8   ,0xc    ,0x60   ,0x8    ,0x20   ,0x8    ,0x20   ,0xc    ,0x60   ,0x7,0xc0    ,0x3    ,0x80   },
{0x0    ,0x0    ,0x0    ,0x70   ,0x8    ,0xf8   ,0x9    ,0x8c   ,0x3f   ,0x4    ,0x3f   ,0x4    ,0x9    ,0x8c   ,0x8    ,0xf8   ,0x0    ,0x70   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x3f   ,0xf0   ,0x79   ,0x98   ,0x78   ,0xcc   ,0x30   ,0x66   ,0x0    ,0x33   ,0x7    ,0xff   ,0xf    ,0x0    ,0xf,0x0     ,0x6    ,0x0    },
{0x0    ,0x80   ,0x9    ,0xc8   ,0x7    ,0xf0   ,0x6    ,0x30   ,0xc    ,0x18   ,0x3c   ,0x1e   ,0xc    ,0x18   ,0x6    ,0x30   ,0x7    ,0xf0   ,0x9,0xc8    ,0x0    ,0x80   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x80   ,0x1    ,0xc0   ,0x3    ,0xe0   ,0x7    ,0xf0   ,0xf    ,0xf8   ,0x1f   ,0xfc   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x1f   ,0xfc   ,0xf    ,0xf8   ,0x7    ,0xf0   ,0x3    ,0xe0   ,0x1    ,0xc0   ,0x0    ,0x80   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x4    ,0x10   ,0xc    ,0x18   ,0x1c   ,0x1c   ,0x3f   ,0xfe   ,0x1c   ,0x1c   ,0xc    ,0x18   ,0x4    ,0x10   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x37   ,0xfe   ,0x37   ,0xfe   ,0x0    ,0x0    ,0x0    ,0x0    ,0x37   ,0xfe   ,0x37   ,0xfe   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x2    ,0x3f   ,0xfe   ,0x3f   ,0xfe   ,0x0    ,0x2    ,0x3f   ,0xfe   ,0x3f   ,0xfe   ,0x0    ,0x82   ,0x0    ,0xc6   ,0x0,0x7c    ,0x0    ,0x38   },
{0x0    ,0x0    ,0x0    ,0x0    ,0xe    ,0xc4   ,0x1f   ,0xe6   ,0x11   ,0x22   ,0x11   ,0x22   ,0x11   ,0x22   ,0x11   ,0x22   ,0x19   ,0xfe   ,0x8,0xdc    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x44   ,0x8    ,0x4c   ,0xc    ,0x5c   ,0xe    ,0x7f   ,0xff   ,0x5c   ,0xe    ,0x4c   ,0xc    ,0x44   ,0x8    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x10   ,0x0    ,0x18   ,0x0    ,0x1c   ,0x3f   ,0xfe   ,0x0    ,0x1c   ,0x0    ,0x18   ,0x0    ,0x10   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x4    ,0x0    ,0xc    ,0x0    ,0x1c   ,0x0    ,0x3f   ,0xfe   ,0x1c   ,0x0    ,0xc    ,0x0    ,0x4    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x80   ,0x1    ,0xc0   ,0x3    ,0xe0   ,0x7    ,0xf0   ,0x0    ,0x80   ,0x0    ,0x80   ,0x0    ,0x80   ,0x0    ,0x80   ,0x0,0x80    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x80   ,0x0    ,0x80   ,0x0    ,0x80   ,0x0    ,0x80   ,0x0    ,0x80   ,0x7    ,0xf0   ,0x3    ,0xe0   ,0x1    ,0xc0   ,0x0,0x80    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20   ,0x0    ,0x20,0x0    ,0x3f   ,0x0    },
{0x0    ,0x80   ,0x1    ,0xc0   ,0x3    ,0xe0   ,0x7    ,0xf0   ,0x0    ,0x80   ,0x0    ,0x80   ,0x0    ,0x80   ,0x7    ,0xf0   ,0x3    ,0xe0   ,0x1,0xc0    ,0x0    ,0x80   },
{0x4    ,0x0    ,0x6    ,0x0    ,0x7    ,0x0    ,0x7    ,0x80   ,0x7    ,0xc0   ,0x7    ,0xe0   ,0x7    ,0xc0   ,0x7    ,0x80   ,0x7    ,0x0    ,0x6,0x0     ,0x4    ,0x0    },
{0x0    ,0x20   ,0x0    ,0x60   ,0x0    ,0xe0   ,0x1    ,0xe0   ,0x3    ,0xe0   ,0x7    ,0xe0   ,0x3    ,0xe0   ,0x1    ,0xe0   ,0x0    ,0xe0   ,0x0,0x60    ,0x0    ,0x20   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x7c   ,0x33   ,0xff   ,0x33   ,0xff   ,0x0    ,0x7c   ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3c   ,0x0    ,0x3c   ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3c   ,0x0    ,0x3c   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x10   ,0x0    ,0x1e   ,0x2    ,0x7e   ,0x3    ,0xf0   ,0x1f   ,0x90   ,0x1e   ,0x1e   ,0x2    ,0x7e   ,0x3    ,0xf0   ,0x1f   ,0x90   ,0x1e,0x10   ,0x2    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x7    ,0x88   ,0xf    ,0xcc   ,0xc    ,0xcc   ,0x3f   ,0xff   ,0x3f   ,0xff   ,0xc    ,0xcc   ,0xc    ,0xfc   ,0x4,0x78    ,0x0    ,0x0    },
{0x0    ,0x1c   ,0x38   ,0x38   ,0x38   ,0x70   ,0x38   ,0xe0   ,0x1    ,0xc0   ,0x3    ,0x80   ,0x7    ,0x0    ,0xe    ,0x38   ,0x1c   ,0x38   ,0x38,0x38   ,0x30   ,0x0    },
{0x0    ,0x0    ,0x22   ,0x0    ,0x36   ,0x0    ,0x1c   ,0x1c   ,0x1e   ,0x3e   ,0x37   ,0xe2   ,0x21   ,0xc6   ,0x31   ,0xfc   ,0x3f   ,0xb8   ,0x1f,0x0    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x1f   ,0x0    ,0x3f   ,0x0    ,0x27   ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x20   ,0x1    ,0x20   ,0x1    ,0x38   ,0x7    ,0x1f   ,0xfe   ,0xf    ,0xfc   ,0x3    ,0xf0   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3    ,0xf0   ,0xf    ,0xfc   ,0x1f   ,0xfe   ,0x38   ,0x7    ,0x20   ,0x1    ,0x20   ,0x1    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0xc    ,0x98   ,0xe    ,0xb8   ,0x3    ,0xe0   ,0xf    ,0xf8   ,0xf    ,0xf8   ,0x3    ,0xe0   ,0xe    ,0xb8   ,0xc,0x98    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x1    ,0x80   ,0x1    ,0x80   ,0x1    ,0x80   ,0xf    ,0xf0   ,0xf    ,0xf0   ,0x1    ,0x80   ,0x1    ,0x80   ,0x1,0x80    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x78   ,0x0    ,0xf8   ,0x0    ,0xb8   ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x1    ,0x80   ,0x1    ,0x80   ,0x1    ,0x80   ,0x1    ,0x80   ,0x1    ,0x80   ,0x1    ,0x80   ,0x1    ,0x80   ,0x1,0x80    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0xe    ,0x0    ,0x1c   ,0x0    ,0x38   ,0x0    ,0x70   ,0x0    ,0xe0   ,0x1    ,0xc0   ,0x3    ,0x80   ,0x7    ,0x0    ,0xe    ,0x0    ,0x1c,0x0    ,0x18   ,0x0    },
{0x7    ,0xf8   ,0x1f   ,0xfe   ,0x18   ,0x1e   ,0x30   ,0x33   ,0x30   ,0x63   ,0x30   ,0xc3   ,0x31   ,0x83   ,0x33   ,0x3    ,0x1e   ,0x6    ,0x1f,0xfe   ,0x7    ,0xf8   },
{0x0    ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x30   ,0xe    ,0x30   ,0xc    ,0x30   ,0xc    ,0x0,0x0     ,0x0    ,0x0    },
{0x30   ,0x1c   ,0x30   ,0x3e   ,0x30   ,0x77   ,0x30   ,0xe3   ,0x31   ,0xc3   ,0x33   ,0x83   ,0x37   ,0x3    ,0x3e   ,0x3    ,0x3c   ,0x7    ,0x38,0x1e   ,0x30   ,0x1c   },
{0xe    ,0x3c   ,0x1f   ,0x7e   ,0x39   ,0xe7   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x38   ,0x7    ,0x1c,0xe    ,0xc    ,0xc    },
{0x3    ,0x0    ,0x3    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x3    ,0x7    ,0x3    ,0xe    ,0x3    ,0x1c   ,0x3    ,0x38   ,0x3    ,0x70   ,0x3,0xe0    ,0x3    ,0xc0   },
{0xf    ,0x83   ,0x1f   ,0xc3   ,0x38   ,0xe3   ,0x30   ,0x63   ,0x30   ,0x63   ,0x30   ,0x63   ,0x30   ,0x63   ,0x30   ,0x63   ,0x38   ,0x63   ,0x1c,0x7f   ,0xc    ,0x3f   },
{0xf    ,0x0    ,0x1f   ,0x80   ,0x39   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc7   ,0x30   ,0xce   ,0x30   ,0xdc   ,0x39   ,0xf8   ,0x1f,0xf0   ,0xf    ,0xc0   },
{0x0    ,0x3    ,0x0    ,0xf    ,0x0    ,0x3f   ,0x0    ,0xf3   ,0x3    ,0xc3   ,0xf    ,0x3    ,0x3c   ,0x3    ,0x30   ,0x3    ,0x0    ,0x3    ,0x0,0x3     ,0x0    ,0x3    },
{0xf    ,0x0    ,0x1f   ,0xbc   ,0x39   ,0xfe   ,0x30   ,0xe7   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xe7   ,0x39   ,0xfe   ,0x1f,0xbc   ,0xf    ,0x0    },
{0x0    ,0xfc   ,0x3    ,0xfe   ,0x7    ,0xe7   ,0xe    ,0xc3   ,0x1c   ,0xc3   ,0x38   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xe7   ,0x0,0x7e    ,0x0    ,0x3c   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x1c   ,0x70   ,0x1c   ,0x70   ,0x1c   ,0x70   ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x7c   ,0x70   ,0xfc   ,0x70   ,0x9c   ,0x70   ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x3    ,0x38   ,0x7    ,0x1c   ,0xe    ,0xe    ,0x1c   ,0x7    ,0x38   ,0x3    ,0xf0   ,0x1    ,0xe0   ,0x0,0xc0    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x6    ,0x60   ,0x6    ,0x60   ,0x6    ,0x60   ,0x6    ,0x60   ,0x6    ,0x60   ,0x6    ,0x60   ,0x6    ,0x60   ,0x6    ,0x60   ,0x6,0x60    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0xc0   ,0x1    ,0xe0   ,0x3    ,0xf0   ,0x7    ,0x38   ,0xe    ,0x1c   ,0x1c   ,0xe    ,0x38   ,0x7    ,0x30,0x3    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x1c   ,0x0    ,0x3e   ,0x0    ,0x77   ,0x0    ,0xe3   ,0x37   ,0xc3   ,0x37   ,0x83   ,0x0    ,0x3    ,0x0    ,0x7    ,0x0,0x1e    ,0x0    ,0x1c   },
{0x1    ,0xf8   ,0x3    ,0xfe   ,0x36   ,0x7    ,0x37   ,0xfb   ,0x37   ,0xfb   ,0x36   ,0x1b   ,0x37   ,0xfb   ,0x33   ,0xf3   ,0x18   ,0x7    ,0x1f,0xfe   ,0xf    ,0xf8   },
{0x0    ,0x0    ,0x38   ,0x0    ,0x3f   ,0x0    ,0x7    ,0xe0   ,0x6    ,0xfc   ,0x6    ,0x1f   ,0x6    ,0x1f   ,0x6    ,0xfc   ,0x7    ,0xe0   ,0x3f,0x0    ,0x38   ,0x0    },
{0x0    ,0x0    ,0xf    ,0x0    ,0x1f   ,0xbc   ,0x39   ,0xfe   ,0x30   ,0xe7   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0xc    ,0xc    ,0x1c   ,0xe    ,0x38   ,0x7    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x38   ,0x7    ,0x1c   ,0xe    ,0xf,0xfc    ,0x3    ,0xf0   },
{0x0    ,0x0    ,0x3    ,0xf0   ,0xf    ,0xfc   ,0x1c   ,0xe    ,0x38   ,0x7    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x0    ,0x3    ,0x0    ,0x3    ,0x0    ,0xc3   ,0x0    ,0xc3   ,0x0    ,0xc3   ,0x0    ,0xc3   ,0x0    ,0xc3   ,0x0    ,0xc3   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x3f   ,0xc6   ,0x3f   ,0xc7   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0x3    ,0x38   ,0x7    ,0x1c   ,0xe    ,0xf,0xfc    ,0x3    ,0xf0   },
{0x0    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x3    ,0x30   ,0x3    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x30   ,0x3    ,0x30   ,0x3    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x7    ,0xff   ,0x1f   ,0xff   ,0x38   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x38   ,0x0    ,0x1e,0x0    ,0xe    ,0x0    },
{0x0    ,0x0    ,0x30   ,0x3    ,0x38   ,0x7    ,0x1c   ,0xe    ,0xe    ,0x1c   ,0x7    ,0x38   ,0x3    ,0xf0   ,0x1    ,0xe0   ,0x0    ,0xc0   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x0    ,0x1e   ,0x0    ,0x78   ,0x1    ,0xe0   ,0x1    ,0xe0   ,0x0    ,0x78   ,0x0    ,0x1e   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x1c   ,0x0    ,0x7    ,0x0    ,0x3    ,0xc0   ,0x0    ,0xf0   ,0x0    ,0x38   ,0x0    ,0xe    ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x3    ,0xf0   ,0xf    ,0xfc   ,0x1c   ,0xe    ,0x38   ,0x7    ,0x30   ,0x3    ,0x30   ,0x3    ,0x38   ,0x7    ,0x1c   ,0xe    ,0xf,0xfc    ,0x3    ,0xf0   },
{0x0    ,0x0    ,0x0    ,0x7c   ,0x0    ,0xfe   ,0x1    ,0xc7   ,0x1    ,0x83   ,0x1    ,0x83   ,0x1    ,0x83   ,0x1    ,0x83   ,0x1    ,0x83   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x33   ,0xf0   ,0x3f   ,0xfc   ,0x1c   ,0xe    ,0x3e   ,0x7    ,0x36   ,0x3    ,0x30   ,0x3    ,0x38   ,0x7    ,0x1c   ,0xe    ,0xf,0xfc    ,0x3    ,0xf0   },
{0x0    ,0x0    ,0x30   ,0x7c   ,0x38   ,0xfe   ,0x1d   ,0xc7   ,0xf    ,0x83   ,0x7    ,0x83   ,0x3    ,0x83   ,0x1    ,0x83   ,0x1    ,0x83   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0xf    ,0xc    ,0x1f   ,0x8e   ,0x39   ,0xc7   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x30   ,0xc3   ,0x38   ,0xe7   ,0x1c,0x7e   ,0xc    ,0x3c   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3    ,0x0    ,0x3    ,0x0    ,0x3    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x0    ,0x3    ,0x0    ,0x3    ,0x0,0x3     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x7    ,0xff   ,0x1f   ,0xff   ,0x38   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x38   ,0x0    ,0x1f,0xff   ,0x7    ,0xff   },
{0x0    ,0x0    ,0x0    ,0x7    ,0x0    ,0x3f   ,0x1    ,0xf8   ,0xf    ,0xc0   ,0x3e   ,0x0    ,0x3e   ,0x0    ,0xf    ,0xc0   ,0x1    ,0xf8   ,0x0,0x3f    ,0x0    ,0x7    },
{0x0    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x1c   ,0x0    ,0x6    ,0x0    ,0x3    ,0x80   ,0x3    ,0x80   ,0x6    ,0x0    ,0x1c   ,0x0    ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x30   ,0x3    ,0x3c   ,0xf    ,0xe    ,0x1c   ,0x3    ,0x30   ,0x1    ,0xe0   ,0x1    ,0xe0   ,0x3    ,0x30   ,0xe    ,0x1c   ,0x3c,0xf    ,0x30   ,0x3    },
{0x0    ,0x0    ,0x0    ,0x3    ,0x0    ,0xf    ,0x0    ,0x3c   ,0x0    ,0xf0   ,0x3f   ,0xc0   ,0x3f   ,0xc0   ,0x0    ,0xf0   ,0x0    ,0x3c   ,0x0,0xf     ,0x0    ,0x3    },
{0x0    ,0x0    ,0x30   ,0x3    ,0x30   ,0xf    ,0x30   ,0x1f   ,0x30   ,0x33   ,0x30   ,0xe3   ,0x31   ,0xc3   ,0x33   ,0x3    ,0x3e   ,0x3    ,0x3c,0x3    ,0x30   ,0x3    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x0,0x0     ,0x0    ,0x0    },
{0x18   ,0x0    ,0x1c   ,0x0    ,0xe    ,0x0    ,0x7    ,0x0    ,0x3    ,0x80   ,0x1    ,0xc0   ,0x0    ,0xe0   ,0x0    ,0x70   ,0x0    ,0x38   ,0x0,0x1c    ,0x0    ,0xe    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x30   ,0x3    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x60   ,0x0    ,0x70   ,0x0    ,0x38   ,0x0    ,0x1c   ,0x0    ,0xe    ,0x0    ,0x7    ,0x0    ,0xe    ,0x0    ,0x1c   ,0x0    ,0x38   ,0x0,0x70    ,0x0    ,0x60   },
{0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0   ,0x0    ,0xc0,0x0    ,0xc0   ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x4e   ,0x0    ,0x7e   ,0x0    ,0x3e   ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x3f   ,0xc0   ,0x3f   ,0xe0   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x3e,0x40   ,0x1c   ,0x0    },
{0x0    ,0x0    ,0xf    ,0x80   ,0x1f   ,0xc0   ,0x38   ,0xe0   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0xc0   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x8    ,0x80   ,0x18   ,0xc0   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x38   ,0xe0   ,0x1f,0xc0   ,0xf    ,0x80   },
{0x0    ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x30   ,0xc0   ,0x30   ,0xe0   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x38   ,0xe0   ,0x1f,0xc0   ,0xf    ,0x80   },
{0x0    ,0x0    ,0x1    ,0x80   ,0x13   ,0xc0   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x3b   ,0xe0   ,0x1f,0xc0   ,0xf    ,0x80   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3    ,0x0    ,0xc3   ,0x0    ,0xc3   ,0x0    ,0xc7   ,0x3f   ,0xfe   ,0x3f   ,0xfc   ,0x0,0xc0    ,0x0    ,0xc0   },
{0x0    ,0x0    ,0x3f   ,0xe0   ,0x7f   ,0xe0   ,0xe6   ,0x60   ,0xcc   ,0x60   ,0xcc   ,0x60   ,0xcc   ,0x60   ,0xcc   ,0x60   ,0xce   ,0xe0   ,0xc7,0xc0   ,0x3    ,0x80   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x3f   ,0x80   ,0x3f   ,0xc0   ,0x0    ,0xe0   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0xc0   ,0x3f,0xff   ,0x3f   ,0xff   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x3f   ,0xec   ,0x3f   ,0xec   ,0x30   ,0x60   ,0x30   ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x7f   ,0xec   ,0xff   ,0xec   ,0xc0   ,0x60   ,0xc0   ,0x0    ,0xe0   ,0x0    ,0x60   ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x0    ,0x38   ,0x60   ,0x1c   ,0xe0   ,0xf    ,0xc0   ,0x7    ,0x80   ,0x3    ,0x0    ,0x3f   ,0xff   ,0x3f,0xff   ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x3f   ,0xff   ,0x3f   ,0xff   ,0x30   ,0x3    ,0x30   ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x3f   ,0x80   ,0x3f   ,0xc0   ,0x0    ,0xe0   ,0x0    ,0xe0   ,0x3f   ,0xc0   ,0x3f   ,0xc0   ,0x0    ,0xe0   ,0x0    ,0xe0   ,0x3f,0xc0   ,0x3f   ,0xe0   },
{0x0    ,0x0    ,0x3f   ,0x80   ,0x3f   ,0xc0   ,0x0    ,0xe0   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0x60   ,0x3f   ,0xe0   ,0x3f,0xe0   ,0x0    ,0x0    },
{0x0    ,0x0    ,0xf    ,0x80   ,0x1f   ,0xc0   ,0x38   ,0xe0   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x38   ,0xe0   ,0x1f,0xc0   ,0xf    ,0x80   },
{0x0    ,0x0    ,0x7    ,0x80   ,0xf    ,0xc0   ,0x1c   ,0xe0   ,0x18   ,0x60   ,0x18   ,0x60   ,0x18   ,0x60   ,0x18   ,0x60   ,0xc    ,0x60   ,0xff,0xe0   ,0xff   ,0xe0   },
{0x0    ,0x0    ,0xff   ,0xe0   ,0xff   ,0xe0   ,0xc    ,0x60   ,0x18   ,0x60   ,0x18   ,0x60   ,0x18   ,0x60   ,0x18   ,0x60   ,0x1c   ,0xe0   ,0xf,0xc0    ,0x7    ,0x80   },
{0x0    ,0x0    ,0x0    ,0xc0   ,0x0    ,0xe0   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0x60   ,0x0    ,0xc0   ,0x3f   ,0xe0   ,0x3f,0xe0   ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x1e   ,0x40   ,0x3f   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33   ,0x60   ,0x33,0xe0   ,0x11   ,0xc0   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x0    ,0x30   ,0x60   ,0x30   ,0x60   ,0x30   ,0x60   ,0x3f   ,0xfe   ,0x1f   ,0xfe   ,0x0,0x60    ,0x0    ,0x60   },
{0x0    ,0x0    ,0x3f   ,0xe0   ,0x3f   ,0xe0   ,0x18   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x30   ,0x0    ,0x38   ,0x0    ,0x1f,0xe0   ,0xf    ,0xe0   },
{0x0    ,0x0    ,0x0    ,0x60   ,0x1    ,0xe0   ,0x7    ,0x80   ,0x1e   ,0x0    ,0x38   ,0x0    ,0x38   ,0x0    ,0x1e   ,0x0    ,0x7    ,0x80   ,0x1,0xe0    ,0x0    ,0x60   },
{0x0    ,0x0    ,0x7    ,0xe0   ,0x1f   ,0xe0   ,0x38   ,0x0    ,0x1c   ,0x0    ,0xf    ,0xe0   ,0xf    ,0xe0   ,0x1c   ,0x0    ,0x38   ,0x0    ,0x1f,0xe0   ,0x7    ,0xe0   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x60   ,0x38   ,0xe0   ,0x1d   ,0xc0   ,0xf    ,0x80   ,0x7    ,0x0    ,0xf    ,0x80   ,0x1d   ,0xc0   ,0x38,0xe0   ,0x30   ,0x60   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x60   ,0x1    ,0xe0   ,0x7    ,0x80   ,0x1e   ,0x0    ,0x7e   ,0x0    ,0xe7   ,0x80   ,0x81   ,0xe0   ,0x0,0x60    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x30   ,0x20   ,0x30   ,0x60   ,0x30   ,0xe0   ,0x31   ,0xe0   ,0x33   ,0x60   ,0x36   ,0x60   ,0x3c   ,0x60   ,0x38,0x60   ,0x30   ,0x60   },
{0x0    ,0x0    ,0x0    ,0x0    ,0x60   ,0x3    ,0x60   ,0x3    ,0x60   ,0x3    ,0x70   ,0x7    ,0x3f   ,0x7e   ,0x1f   ,0xfc   ,0x1    ,0xc0   ,0x0,0x80    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x3f   ,0xbf   ,0x3f   ,0xbf   ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x80   ,0x1    ,0xc0   ,0x1f   ,0xfc   ,0x3f   ,0x7e   ,0x70   ,0x7    ,0x60   ,0x3    ,0x60   ,0x3    ,0x60,0x3    ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x4    ,0x0    ,0xc    ,0x0    ,0x18   ,0x0    ,0x10   ,0x0    ,0x18   ,0x0    ,0xc    ,0x0    ,0x4    ,0x0    ,0xc    ,0x0,0x18    ,0x0    ,0x10   },
{0x0    ,0x0    ,0xf    ,0x0    ,0xf    ,0x80   ,0xc    ,0xc0   ,0xc    ,0x60   ,0xc    ,0x30   ,0xc    ,0x30   ,0xc    ,0x60   ,0xc    ,0xc0   ,0xf,0x80    ,0xf    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x78   ,0x0    ,0xfc   ,0x0    ,0xcc   ,0x0    ,0xcc   ,0x0    ,0xfc   ,0x0    ,0x78   ,0x0,0x0     ,0x0    ,0x0    },
{0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0    ,0x0,0x0     ,0x0    ,0x0    },
};

//S-Meter max value
int smax = 0;

//Tuning & seconds counting
int tuning = 0;
long pulses = 0;
long runsecs = 0;
long runsecs_msg = 0;
long runsecs_smax = 0;
long runsecs_mtr = 0;

//VFO data & frequencies
int cur_vfo;
int cur_band;
int sideband;
long f_lo[2];
long f_vfo[MAXBANDS][MAXVFO];
	                        
int pref_sideband[] = {0, 0, 1, 0, 1, 1, 1, 1, 1, 1}; //Preferred sideband for each ham band
long f_cntr[] =  {1840000, 3650000, 5366500, 7100000, 10125000, 14175000, 18132000, 21225000, 24910000, 28500000};  //Center freq
long band_f0[] = {1810000, 3500000, 5351500, 7000000, 10100000, 14000000, 18065000, 21000000, 24890000, 28000000};  //Band start
long band_f1[] = {2000000, 3800000, 5366500, 7200000, 10150000, 14350000, 18165000, 21465000, 24990000, 29700000};  //Band end


  /////////////////////////////
 //     INT Handlers        //
/////////////////////////////
//EXTI0
extern "C" void EXTI0_IRQHandler(void)
{
    uint16_t state; 
       
    //Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0))
    {   
		state = GPIOB->IDR & 0x03; //Read pins
        if(state & 1)
        {
			pulses++;
            if(state & 2)
            {
                tuning = 1;
            }
            else
            {
                tuning = -1;
            }
        }

        //Clear bit
        EXTI->PR = (1 << 0);
    }
}

//TIM2
extern "C" void TIM2_IRQHandler(void)
{	
	if(TIM2->SR & TIM_SR_UIF)       
    {
		pulses = 0;
		runsecs++;
    }
    TIM2->SR = 0x00;  //Reset status register  
}

  //////////////////
 //     MISC     //
//////////////////
// Quick and dirty delay
static void delay(unsigned int time) 
{
    for (unsigned int i = 0; i < time; i++)
    {
        for (volatile unsigned int j = 0; j < 2000; j++);
    }    
}

  //////////////////
 //    L  C  D   //
//////////////////

//SPI write data or command to LCD
void lcd_send(int dc, int value)
{
	int t1;
		
	if(!dc) //Cmd (0) or Data(1)?
	{
	    LCD_GPIO->ODR &= ~(1 << DC);  //Cmd=0
	}
	else
	{
	    LCD_GPIO->ODR |= 1 << DC;     //Data=1
	}
		
	for(t1 = 7; t1 >= 0; t1--)
	{
		if(value & (1 << t1)) 
	    {
	        LCD_GPIO->ODR |= 1 << SDA;     //1
	    }
	    else
	    {
	        LCD_GPIO->ODR &= ~(1 << SDA);  //0
	    }
	    	    
	    LCD_GPIO->ODR |= 1 << SCK;     //SCK hi
	    LCD_GPIO->ODR &= ~(1 << SCK);  //SCK lo
	}
}	

//Init LCD to vertical alignement and 16-bit color mode
void lcd_init(void)
{
    lcd_send(LCD_CMD,  0xCB);
    lcd_send(LCD_DATA, 0x39);
    lcd_send(LCD_DATA, 0x2C);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x34);
    lcd_send(LCD_DATA, 0x02);

    lcd_send(LCD_CMD, 0xCF);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0XC1);
    lcd_send(LCD_DATA, 0X30);

    lcd_send(LCD_CMD,  0xE8);
    lcd_send(LCD_DATA, 0x85);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x78);

    lcd_send(LCD_CMD,  0xEA);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x00);

    lcd_send(LCD_CMD,  0xED);
    lcd_send(LCD_DATA, 0x64);
    lcd_send(LCD_DATA, 0x03);
    lcd_send(LCD_DATA, 0X12);
    lcd_send(LCD_DATA, 0X81);

    lcd_send(LCD_CMD,  0xF7);
    lcd_send(LCD_DATA, 0x20);

    lcd_send(LCD_CMD,  0xC0); // Power control
    lcd_send(LCD_DATA, 0x23); // VRH[5:0]

    lcd_send(LCD_CMD,  0xC1); // Power control
    lcd_send(LCD_DATA, 0x10); // SAP[2:0];BT[3:0]

    lcd_send(LCD_CMD,  0xC5); // VCM control
    lcd_send(LCD_DATA, 0x3e);
    lcd_send(LCD_DATA, 0x28);

    lcd_send(LCD_CMD,  0xC7); // VCM control2
    lcd_send(LCD_DATA, 0x86);

    lcd_send(LCD_CMD,  0x36); // Memory Access Control
    lcd_send(LCD_DATA, 0x88); // C8

    lcd_send(LCD_CMD,  0x3A);
    lcd_send(LCD_DATA, 0x55);

    lcd_send(LCD_CMD,  0xB1);
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x18);

    lcd_send(LCD_CMD,  0xB6); // Display Function Control
    lcd_send(LCD_DATA, 0x08);
    lcd_send(LCD_DATA, 0x82);
    lcd_send(LCD_DATA, 0x27);

    lcd_send(LCD_CMD,  0xF2); // 3Gamma Function Disable
    lcd_send(LCD_DATA, 0x00);

    lcd_send(LCD_CMD,  0x26); // Gamma curve selected
    lcd_send(LCD_DATA, 0x01);

    lcd_send(LCD_CMD,  0xE0); // Set Gamma
    lcd_send(LCD_DATA, 0x0F);
    lcd_send(LCD_DATA, 0x31);
    lcd_send(LCD_DATA, 0x2B);
    lcd_send(LCD_DATA, 0x0C);
    lcd_send(LCD_DATA, 0x0E);
    lcd_send(LCD_DATA, 0x08);
    lcd_send(LCD_DATA, 0x4E);
    lcd_send(LCD_DATA, 0xF1);
    lcd_send(LCD_DATA, 0x37);
    lcd_send(LCD_DATA, 0x07);
    lcd_send(LCD_DATA, 0x10);
    lcd_send(LCD_DATA, 0x03);
    lcd_send(LCD_DATA, 0x0E);
    lcd_send(LCD_DATA, 0x09);
    lcd_send(LCD_DATA, 0x00);

    lcd_send(LCD_CMD,  0xE1); // Set Gamma
    lcd_send(LCD_DATA, 0x00);
    lcd_send(LCD_DATA, 0x0E);
    lcd_send(LCD_DATA, 0x14);
    lcd_send(LCD_DATA, 0x03);
    lcd_send(LCD_DATA, 0x11);
    lcd_send(LCD_DATA, 0x07);
    lcd_send(LCD_DATA, 0x31);
    lcd_send(LCD_DATA, 0xC1);
    lcd_send(LCD_DATA, 0x48);
    lcd_send(LCD_DATA, 0x08);
    lcd_send(LCD_DATA, 0x0F);
    lcd_send(LCD_DATA, 0x0C);
    lcd_send(LCD_DATA, 0x31);
    lcd_send(LCD_DATA, 0x36);
    lcd_send(LCD_DATA, 0x0F);

    lcd_send(LCD_CMD,  0x11); // Sleep out
    delay(120);
    lcd_send(LCD_CMD, 0x2c);  
        
    lcd_send(LCD_CMD, 0x29); // Display on 
    lcd_send(LCD_CMD, 0x2c);	
	
}	

void lcd_drawhline(int x0, int x1, int y0, int fcolor)
{
	int x;
	
	for(x = x0; x < x1; x++)
	{
		lcd_set_xy(x, y0);
		lcd_draw_pixel(fcolor);
	}	
}
	
void lcd_set_xy(int x, int y)
{
	//X
	lcd_send(LCD_CMD, 0x2B);
    lcd_send(LCD_DATA, x >> 8);
    lcd_send(LCD_DATA, x & 0xFF);
    lcd_send(LCD_CMD, 0x2c);

    //Y 
    lcd_send(LCD_CMD, 0x2A);
    lcd_send(LCD_DATA, y >> 8);
    lcd_send(LCD_DATA, y & 0xFF);
    lcd_send(LCD_CMD, 0x2c);
}
 
void lcd_draw_pixel(int color)
{
    lcd_send(LCD_DATA, color >> 8);
    lcd_send(LCD_DATA, color & 0xFF);
}

void lcd_cls(int bcolor)
{
	int x;
	unsigned char y;
	
	lcd_set_xy(0, 0);
	for(x = 0; x < LCD_WIDTH; x++)
	{
        for(y = 0; y < LCD_HEIGHT; y++)
        {
			lcd_draw_pixel(bcolor);
		}	
	}
}		

//Write character from font set to destination on screen
void lcd_putchar(int x, int y, int c, int size, int fcolor, int bcolor)
{
    int x0;
    int t0, t1, t2, t3, u;
       
    x0 = 120 - x;
    for(t0 = 0; t0 < FONTWIDTH * 2; t0 += 2)
    { 
		for(t1 = 0; t1 < size; t1++)
		{
		    u = xchar[c][t0 + 1] + (xchar[c][t0] << 8);
		    lcd_set_xy(x0, y);
		    for(t2 = 16; t2 >= 0; t2--)
		    {
			    if(u & (1 << t2))
			    {
				    for(t3 = 0; t3 < size; t3++)
				    {
		                lcd_draw_pixel(fcolor);
		            }
		        }    
		        else
		        {
		            for(t3 = 0; t3 < size; t3++)
				    {
		                lcd_draw_pixel(bcolor);
		            }
		        }
		    }
		    x0++;
		}    
	}	
}	

//Print String to LCD
void lcd_putstring(int x, int y, char *text, int size, int fc, int bc)
{
	int t1 = 0, x0;
	int ypos = y;
	
	x0 = x;
	while(text[t1])
	{
		lcd_putchar(x0, ypos, text[t1], size, fc, bc);
		x0 += (size * FONTWIDTH);
		t1++;
	}
}		

//Convert a number to a string and print it
//col, row: Coordinates, Num: int or long to be displayed
//dec: Set position of decimal separator
//
//inv: Set to 1 if inverted charactor is required
int lcd_putnumber(int x, int y, long num, int dec, int lsize, int fc, int bc)
{
	int lx = 0, t1;
    char *s = (char*)malloc(16);
    	
	if(s != NULL)
	{
		for(t1 = 0; t1 < 16; t1++)
        {
		    *(s +t1) = 0;
	    }
	
	    int2asc(num, dec, s, 16);
	    lx = strlen(s);
	    lcd_putstring(x, y, s, lsize, fc, bc);
	    free(s);
	}	
	else
	{
		lcd_putstring(x, y, (char*)"Error", lsize, fc, bc);
	}	
	return lx;
	
}

  /////////////////
 //   GRAPHICS  //
/////////////////

  //////////////////////
 //  RADIO DISPLAY   //
//////////////////////
//Current frequency (double letter height)
void show_frequency(unsigned long f)
{
	int xpos = 0 * FONTWIDTH + 14, ypos = 3 * FONTHEIGHT - 10;
	int fcolor0 = WHITE;
	int fcolor1 = LIGHTYELLOW;
	
	if(!is_freq_ok(f, cur_band))
	{
		fcolor0 = RED;
	    fcolor1 = RED;
	}
	
	if(!f)
	{
	    lcd_putstring(xpos, ypos, (char*)"     ", 2, fcolor0, bgcolor);
	    lcd_putstring(FONTWIDTH, FONTHEIGHT + 4, (char*)"         ", 1, fcolor1, bgcolor);
	}	
	
	if(f < 10000000)
	{
		xpos = 1 * FONTWIDTH  + 14;
		//lcd_putstring(xpos, ypos, (char*)" ", 2, fcolor, bgcolor);
	}	
	
	lcd_putnumber(xpos, ypos, f / 1000, -1, 2, fcolor0, bgcolor);
	
	if(f < 10000000)
	{
		xpos = 2 * FONTWIDTH;
	}	    
	else
	{
		xpos = 1 * FONTWIDTH;
	}	
    
    ypos = 1 * FONTHEIGHT + 4;
    
    lcd_putnumber(xpos, ypos, f, 3, 1, fcolor1, bgcolor);    
}

void show_band(int b)
{
	int xpos = 0 * FONTWIDTH, ypos = 7 * FONTHEIGHT;
		
    char *bnd[] = {(char*)"160m", (char*)"80m ", (char*)"60m ", (char*)"40m ", (char*)"30m ", (char*)"20m ", (char*)"17m ", (char*)"15m ", (char*)"12m ", (char*)"10m "};
	
	unsigned int fcolor[] = {LIGHTGRAY, LIGHTVIOLET, LIGHTGREEN, LIGHTBLUE, LIGHTBROWN, LIGHTYELLOW, LIGHTGRAY, LIGHTGREEN, LIGHTBROWN, YELLOW};
	//Write string to position
	lcd_putstring(xpos, ypos, bnd[b], 1, fcolor[b], bgcolor);
}

void show_sideband(int sb)
{
	int xpos = 5 * FONTWIDTH, ypos = 7 * FONTHEIGHT;
    char *sb_str[] = {(char*)"LSB", (char*)"USB"};
	int fcolor = LIGHTBLUE;	
	
	if(sb)
	{
		fcolor = LIGHTORANGE;
	}	
	//Write string to position
	lcd_putstring(xpos, ypos, sb_str[sb], 1, fcolor, bgcolor);
}

void show_vfo(int n_vfo)
{
	int xpos = 9 * FONTWIDTH, ypos = 7 * FONTHEIGHT;
	
	//lcd_putstring(xpos, ypos, (char*)"V", 1, WHITE, bgcolor);
	//Write string to position
	lcd_putchar(xpos, ypos, n_vfo + 65, 1, LIGHTGRAY, bgcolor);
	//lcd_putchar((xpos + 5), ypos, 32, 1, WHITE, bgcolor);
}	

void show_pa_temp(int patemp)
{
    int p;
	int xpos = 0 * FONTWIDTH, ypos = 6 * FONTHEIGHT - 2;
	int fcolor = WHITE;
		
	if(patemp  > -10)
	{
	    fcolor = BLUE;
	}	
	
	if(patemp > 0)
	{
		fcolor = LIGHTBLUE;
	}	
	
	if(patemp > 10)
	{
		fcolor = LIGHTGREEN;
	}	
	
	if(patemp > 20)
	{
		fcolor = YELLOW;
	}	
	
	if(patemp > 40)
	{
		fcolor = RED;
	}	
	if(patemp > 60)
	{
		fcolor = LIGHTRED;
	}	
		
	p = lcd_putnumber(xpos * FONTWIDTH, ypos, patemp, -1, 1, fcolor, bgcolor);
	lcd_putchar((xpos + p) * FONTWIDTH, ypos, 0x80, 1, fcolor, bgcolor); //Deg. symbol
	lcd_putstring((xpos + p + 1) * FONTWIDTH, ypos, (char*)"C", 1, fcolor, bgcolor);
	
	return;
}

void show_vdd(int v1)
{
    char *buf;
	int t1, p;
	int fcolor = WHITE;
	int xpos = 5 * FONTWIDTH, ypos = 6 * FONTHEIGHT - 2;
	
	buf = (char*)malloc(10);
	
	//Init buffer string
	for(t1 = 0; t1 < 10; t1++)
	{
	    *(buf + t1) = 0;
	}
	
	if(v1 < 160)
	{
		fcolor = LIGHTRED;
	}
	
	if(v1 < 140)
	{
		fcolor = WHITE;
	}
	if(v1 < 120)
	{
		fcolor = LIGHTBLUE;
	}
	if(v1 < 100)
	{
		fcolor = BLUE;
	}	
	
	int2asc(v1, 1, buf, 6);
	p = strlen(buf);
	lcd_putstring(xpos, ypos, (char*)"     ", 1, fcolor, bgcolor);
    lcd_putstring(xpos, ypos, buf, 1, fcolor, bgcolor);
    lcd_putstring(xpos + p * FONTWIDTH, ypos, (char*)"V", 1, fcolor, bgcolor);
	free(buf);
}

void show_txrx(int t)
{
	int xpos = 8 * FONTWIDTH, ypos = 5 * FONTHEIGHT - 4;
	
	if(t)
	{
		lcd_putstring(xpos, ypos, (char*)"TX", 1, LIGHTRED, YELLOW);
	}
	else	
	{
		lcd_putstring(xpos, ypos, (char*)"RX", 1, LIGHTGREEN, bgcolor);
	}
}

void show_tunspeed(int speed)
{
	int xpos = 0 * FONTWIDTH, ypos = 5 * FONTHEIGHT - 4;
	
	switch(speed)
	{
	    case 0: lcd_putstring(xpos, ypos, (char*)"NORM ", 1, LIGHTGRAY, bgcolor);
	            break;
	    case 1: lcd_putstring(xpos, ypos, (char*)"FAST ", 1, GREEN, bgcolor);
	            break;
	    case 2: lcd_putstring(xpos, ypos, (char*)"SFAST", 1, YELLOW, bgcolor);
	            break;        
	    case 3: lcd_putstring(xpos, ypos, (char*)"XFAST", 1, LIGHTRED, WHITE);
	            break;                
	}   
}

void show_msg(char *msg)
{
	int xpos = 0 * FONTWIDTH, ypos = 0 * FONTHEIGHT;
	lcd_putstring(xpos, ypos, (char*)msg, 1,  LIGHTGRAY, bgcolor);
}	

  //////////////////////
 // STRING FUNCTIONS //
//////////////////////
//INT 2 ASC
int int2asc(long num, int dec, char *buf, int buflen)
{
    int i, c, xp = 0, neg = 0;
    long n, dd = 1E09;

    if(!num)
	{
	    *buf++ = '0';
		*buf = 0;
		return 1;
	}	
		
    if(num < 0)
    {
     	neg = 1;
	    n = num * -1;
    }
    else
    {
	    n = num;
    }

    //Fill buffer with \0
    for(i = 0; i < buflen; i++)
    {
	    *(buf + i) = 0;
    }

    c = 9; //Max. number of displayable digits
    while(dd)
    {
	    i = n / dd;
	    n = n - i * dd;
	
	    *(buf + 9 - c + xp) = i + 48;
	    dd /= 10;
	    if(c == dec && dec)
	    {
	        *(buf + 9 - c + ++xp) = '.';
	    }
	    c--;
    }

    //Search for 1st char different from '0'
    i = 0;
    while(*(buf + i) == 48)
    {
	    *(buf + i++) = 32;
    }

    //Add minus-sign if neccessary
    if(neg)
    {
	    *(buf + --i) = '-';
    }

    //Eleminate leading spaces
    c = 0;
    while(*(buf + i))
    {
	    *(buf + c++) = *(buf + i++);
    }
    *(buf + c) = 0;
	
	return c;
}

  ///////////////////////
 //    A   D   C      //     
///////////////////////
//Read ADC value
int get_adc(int adc_channel)
{
	int adc_val = 0;
	ADC1->SQR3 &= ~(0x3FFFFFFF);
	
	switch(adc_channel)
	{
		case KEYS:  ADC1->SQR3 |= (4 << 0); //PA8
		            break;
		case TMP:   ADC1->SQR3 |= (5 << 0); //PA5
		            break;                                        	                
		case VDD:   ADC1->SQR3 |= (6 << 0); //PA6
  	                break;  

	}
	
    ADC1->CR2 |= (1 << 30);         //Start conversion SWSTART bit 
    while(!(ADC1->SR & (1 << 1)));  //Wait until conversion is complete
    adc_val = ADC1->DR;             //Read value from reg	           
			
	return adc_val;
}	

//Read keys on ADC channel 5
int get_keys(void)
{
    int key_value[] = {0, 365, 675, 935, 1159};
    int t1;
    int adcval0;
    
    adcval0 = get_adc(KEYS);
    
    while(get_adc(KEYS) < 4000);
            	
    //lcd_putstring(0, 4, (char*)"    ", 0, 0);
    //lcd_putnumber(0, 4, adcval0, -1, 0, 0);   
    
   	for(t1 = 0; t1 < 5; t1++)
    {
		if((adcval0 > (key_value[t1] - 10)) && (adcval0 < (key_value[t1] + 10)))
		{
			return t1;
		}    
    }
    return -1;
}

//Measure voltage
int get_vdd(void)
{
	int r1 = 6800, r2 = 1000;
	
	double v1 = (double) get_adc(VDD) / 4096 * 3.3 * (r1 + r2) / r2;
    
	return (int) (v1 * 10);
}


//KTY81-210	analog read
//R.VDD = 2.7k
int get_pa_temp(void)
{
	int adc = get_adc(TMP);
	double ux = (double) (3.3 * adc) / 4096;
	double rx = 2700 / (3.3 / ux  - 1);
    double temp = (rx - 1630) / 17.62;  //slope and y0 for KTY81-210
	
	return (int) temp;
}	

int get_txrx(void)
{
    int pin_input = ~GPIOB->IDR; //"0" means "pressed"!
	if(pin_input & (1 << 3))
	{
        return 1;
    }	
    else
	{
        return 0;
    }	    
}

  ///////////////////////
 //    R A D I O      //     
///////////////////////
  /////////////////
 //  SPI  DDS   // 
/////////////////
void spi_send_byte(unsigned int sbyte)
{
    int t1, x = (1 << 7);
	
	for(t1 = 0; t1 < 8; t1++)
	{
	    DDS_GPIO->ODR &= ~(1 << DDS_SCLK);  //DDS_SCLK lo
    	
        //Bit set or erase
	    if(sbyte & x)
	    {
		    DDS_GPIO->ODR |= (1 << DDS_SDIO);  
	    }
	    else
	    {
		    DDS_GPIO->ODR &= ~(1 << DDS_SDIO);  
	    }	
        DDS_GPIO->ODR |= (1 << DDS_SCLK); //DDS_SCLK hi
		x >>= 1;
	}	
}

//Set frequency for AD9951 DDS
void set_frequency(long frequency)
{
    //unsigned long interfreq = 10E06; //Interfrequency of radio in Hz
    unsigned long f;
    unsigned long fword;
    int t1, shiftbyte = 24, resultbyte;
    unsigned long comparebyte = 0xFF000000;
    
	f = frequency + INTERFREQUENCY; //Offset because of inaccuracy of crystal oscillator
	
    //Calculate frequency word
    //2³² / fClk = ----
    
    //Clock rate =  75MHz
    //fword = (unsigned long) f * 57.266230613;
    
    //Clock rate =  100MHz
    //fword = (unsigned long) f * 42.94967296;
        
    //Clock rate =  110MHz
    //fword = (unsigned long) f * 39.045157236;
    
    //Clock rate =  125MHz
    //fword = (unsigned long) f * 34.358675; 
        	
	//Clock rate =  200MHz
    //fword = (unsigned long) f * 21.47478;  
    
    //Clock rate =  300MHz
    //fword = (unsigned long) f * 14.316557653;
    
    //Clock rate =  400MHz
    fword = (unsigned long) f * 10.73741824;
		
    //Start transfer to DDS
    DDS_GPIO->ODR &= ~(1 << DDS_IO_UD); //DDS_IO_UD lo
    
	//Send instruction bit to set fequency by frequency tuning word
	spi_send_byte(0x04);	
	
    //Calculate and transfer the 4 bytes of the tuning word to DDS
    //Start with msb
    for(t1 = 0; t1 < 4; t1++)
    {
        resultbyte = (fword & comparebyte) >> shiftbyte;
        comparebyte >>= 8;
        shiftbyte -= 8;       
        spi_send_byte(resultbyte);	
    }    
	
	//End transfer sequence
    DDS_GPIO->ODR |= (1 << DDS_IO_UD); //DDS_IO_UD hi 
}	

void set_clock_multiplier(void)
{
    //Start transfer to DDS
    DDS_GPIO->ODR &= ~(1 << DDS_IO_UD); //DDS_IO_UD lo
    
	//Send CFR2
	spi_send_byte(0x01);
	
	//Multiply by 4
	spi_send_byte(0x00);
	spi_send_byte(0x00);
	spi_send_byte(0x24); //0x04 << 3
		
	//End transfer sequence
    DDS_GPIO->ODR |= (1 << DDS_IO_UD); //DDS_IO_UD hi 
}	

void set_band_relay(int b)
{
	int t1;
	for(t1 = 3; t1 >= 0; t1--)
	{
		if(b & (1 << t1))
		{
			GPIOB->ODR |= (1 << (t1 + 2));
		}
		else	
		{
	    	GPIOB->ODR &= ~(1 << (t1 + 2));
	    }
	     
	 }   	     
}	

  ////////////////////////////////////
 //   EEPROM & Mem functions       //
////////////////////////////////////
//Store any other frequency by start byte address
void store_frequency(long f, int bnd, int vfo)
{
    long hiword, loword;
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr = bnd * 96 + vfo * 4;

    hiword = f >> 16;
    loword = f - (hiword << 16);
    hmsb = hiword >> 8;
    hlsb = hiword - (hmsb << 8);
    lmsb = loword >> 8;
    llsb = loword - (lmsb << 8);

    eeprom_write(start_adr, hmsb);

    eeprom_write(start_adr + 1, hlsb);

    eeprom_write(start_adr + 2, lmsb);

    eeprom_write(start_adr + 3, llsb);
	
}

//Load any other frequency by start byte address
long load_frequency(int bnd, int vfo)
{
    unsigned char hmsb, lmsb, hlsb, llsb;
    int start_adr = bnd * 96 + vfo * 4;

    hmsb = eeprom_read(start_adr);
    hlsb = eeprom_read(start_adr + 1);
    lmsb = eeprom_read(start_adr + 2);
    llsb = eeprom_read(start_adr + 3);
	
    return (long) 16777216 * hmsb + (long) 65536 * hlsb + (unsigned int) 256 * lmsb + llsb;
}

//Store last BAND used
void store_last_band(int bandnum)
{
    eeprom_write(1023, bandnum);
}

//Load last band stored
int load_last_band(void)
{
    int bandnum;

    bandnum = eeprom_read(1023);
	
	if(bandnum >= 0 && bandnum < MAXBANDS)
	{
		return bandnum;
	}
	else
	{
		return -1;
	}	
}

//Store last VFO used
void store_last_vfo(int vfonum)
{
    eeprom_write(1024, vfonum);
}

//Load last VFO stored
int load_last_vfo(void)
{
    int vfonum;
    vfonum = eeprom_read(1024);
	
	if(vfonum >= 0 && vfonum < MAXVFO)
	{
		return vfonum;
	}
	else
	{
		return -1;
	}	
}

//Check if freq is in respective band
int is_freq_ok(long f, int cband)
{
	
	if((f >= band_f0[cband]) && (f <= band_f1[cband]))
	{
	    return 1;
	}	
	return 0;		
}

void eeprom_write(int mem_address, int value)
{
   uint8_t data[3]; 
   data[0] = mem_address >> 8;   //Address of byte in 24C65 MSB
   data[1] = mem_address & 0xFF; //                         LSB
   data[2] = value;
   i2c_write_byte2(data, 3, EEPROM_ADR);
   delay(5);
}	

void eeprom_erase(void)
{
	int t0;
	show_msg((char*)"Deleting...   ");
	
	for(t0 = 0; t0 < 1025; t0++)
	{
		eeprom_write(t0, 0);
	}	 		
	show_msg((char*)"MEM erased.  ");
}	

int eeprom_read(int mem_address)
{
   uint8_t r = i2c_read2(mem_address, EEPROM_ADR);
   delay(5);
   
   return r;
}

  //////////////////////
 //   I2C commands   //
//////////////////////
void i2c_start(void) 
{
    I2C1->CR1 |= I2C_CR1_START;
    while(!(I2C1->SR1 & I2C_SR1_SB));
}

void i2c_stop(void) 
{
    I2C1->CR1 |= I2C_CR1_STOP;
    while(!(I2C1->SR2 & I2C_SR2_BUSY));
}

void i2c_write_byte1(uint8_t regaddr, uint8_t data, int i2c_adr) 
{
    //Start signal
    i2c_start();

    //Send chipaddr to be targeted
    I2C1->DR = i2c_adr;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)); //Wait until transfer done
    //Perform one dummy read to clear register flags etc.
    (void)I2C1->SR2; //Clear addr reg

    //Send operation type/register 
    I2C1->DR = regaddr;
    while(!(I2C1->SR1 & I2C_SR1_BTF)); //Wait until transfer done

    //Send data
    I2C1->DR = data;
    while(!(I2C1->SR1 & I2C_SR1_BTF));  //Wait

    //Send stop signal
    i2c_stop();
}

//Write multiple number of bytes (>1)
void i2c_write_byte2(uint8_t *data, uint8_t n, int i2c_adr) 
{
	int t1 = 0;
	    
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = i2c_adr; 
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Perform one dummy read to clear flags
    (void)I2C1->SR2;

    for(t1 = 0; t1 < n; t1++)
    {
		//Send data
        I2C1->DR = data[t1];
        while (!(I2C1->SR1 & I2C_SR1_BTF));
    }    
    
    //Send stop signal
    i2c_stop();
}

int16_t i2c_read1(uint8_t regaddr, int i2c_adr) 
{
    int16_t reg;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = i2c_adr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register 
    I2C1->DR = regaddr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = i2c_adr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}

int16_t i2c_read2(uint16_t regaddr, int i2c_adr) 
{
    int16_t reg;
    int16_t r_msb, r_lsb;
    
    r_msb = (regaddr & 0xFF00) >> 8;
    r_lsb = regaddr & 0x00FF;

    //Start communication
    i2c_start();

    //Send device address
    I2C1->DR = i2c_adr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //Dummy read to clear flags
    (void)I2C1->SR2; //Clear addr register

    //Send operation type/register MSB
    I2C1->DR = r_msb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send operation type/register LSB
    I2C1->DR = r_lsb;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Restart by sending stop & start sig
    i2c_stop();
    i2c_start();

    //Repeat
    I2C1->DR = i2c_adr | 0x01; // read
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;

    //Wait until data arrived in receive buffer
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    //Read value
    reg = (uint8_t)I2C1->DR;

    //Send stop signal
    i2c_stop();

    return reg;
}


//Set MCP4724 DAC with 3 parameters according to datasheet
void mcp4725(uint8_t data0, uint8_t data1, uint8_t data2) 
{
    //Send start signal
    i2c_start();

    //Send device address
    I2C1->DR = MCP4725_ADR;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    //One dummy read to clear flags
    (void)I2C1->SR2;
    
    //Send data byte 0
    I2C1->DR = data0;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
    //Send data byte 1
    I2C1->DR = data1;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send data byte 2
    I2C1->DR = data2;
    while (!(I2C1->SR1 & I2C_SR1_BTF));

    //Send stop signal
    i2c_stop();
}


int main(void)
{
	int t0, t1;
	int tr_old = 0;
	int key;
	int sideband_old;
	int tunspeed = 2; //Tuning speed: 0=norm; 1=hi
	int volts, volts_old = 0;
        	
	//Measurement interval
	long runsecs_meas = 0;
	long runsecs_msg = 0;
	
	/////////////////////////////////////////////
    // Set SystemClock to 50 MHz with 25 MHz HSE
    //////////////////////////////////////////////
    FLASH->ACR |= (1 << 1);                     //2 wait states for 96+ MHz
    RCC->CR |= (1 << 16);                       //Activate external clock (HSE: 25 MHz)
    while ((RCC->CR & (1 << 17)) == 0);         //Wait until HSE is ready
    
    RCC->PLLCFGR |= (1 << 22);                  //PLL source is HSE
    
    RCC->PLLCFGR &= ~0b11111;                   //Reset bits 4..0, then set PLL-M: VCO input frequency = PLL input clock frequency / PLLM with 2 ≤ PLLM ≤ 63
    RCC->PLLCFGR |= 20;                         // -> f.VCO.in = 25MHz / 20 = 1.25MHz
                                                
    RCC->PLLCFGR &= ~(32704 << 6);
    RCC->PLLCFGR |= 160 << 6;                   //PLL-N: f.VCO.out = f.VCO.in * 160 = 250MHz
    
    RCC->PLLCFGR &= ~(0b11 << 16);              //Reset - PLL-P: Main PLL (PLL) division factor for main system clock
    RCC->PLLCFGR &= ~(0b01 << 16);              //f.PLL.output.clock = f.VCO.out / 4 = 50MHz
                                                
    RCC->PLLCFGR &= ~(0b111 << 24);             //Main PLL (PLL) division factor for USB OTG FS, SDIO and 
                                                //random number generator clocks (f<=48MHz for RNG!, 48MHz for USB)
    RCC->PLLCFGR |= (4 << 24);                  //PLL-Q: f.VCO.out / 4 = 25MHz
        
    RCC->CR |= (1 << 24);                       //Activate PLL (Output: 100 MHz)
    while ((RCC->CR & (1 << 25)) == 0);         //Wait until PLL is ready
    
    //Division by 2 of clk signal       
    RCC->CFGR |= (0b1000 << 4)                  //AHB divider:  /2
              | (0b100 << 10)                   //APB1 divider: /2
              | (0b100 << 13);                  //APB2 divider: /2
               
    RCC->CFGR |= (1 << 1);                      //Switching to PLL clock source
    
    //GPIOx power on sequence
    RCC->AHB1ENR |= (1 << 0);  //GPIOA
    RCC->AHB1ENR |= (1 << 1);  //GPIOB
    RCC->AHB1ENR |= (1 << 2);  //GPIOC
    
    //Pull up PA7 for sideband indicator
    GPIOB->PUPDR |= (1 << (7 << 1));
            
     /////////////////////////
    //Relay driver section  //
    /////////////////////////
    GPIOB->MODER = 0; //Reset MODER to override JTAG functions on PB3 and PB4
    //PB2..PB5 to general purpose output mode
    for(t0 = 2; t0 < 6; t0++)
    {
		GPIOB->MODER |= (1 << (t0 << 1));
	}	
	
	delay(50);
	    
    /////////////////////////
    //LCD Setup            //
    /////////////////////////
    //Put pin 0..4 in general purpose output mode
    LCD_GPIO->MODER |= (1 << (SCK << 1));	
    LCD_GPIO->MODER |= (1 << (SDA << 1));	
    LCD_GPIO->MODER |= (1 << (DC << 1));	
    LCD_GPIO->MODER |= (1 << (RES << 1));	
    LCD_GPIO->ODR = 0;
    
    LCD_GPIO->OSPEEDR |= (1 << (SCK << 1));	
    LCD_GPIO->OSPEEDR  |= (1 << (SDA << 1));	
    LCD_GPIO->OSPEEDR  |= (1 << (DC << 1));	
    LCD_GPIO->OSPEEDR  |= (1 << (RES << 1));	
                
    //Reset LCD	
    LCD_GPIO->ODR &= ~(1 << RES);  //0
    delay(5);
    LCD_GPIO->ODR |= 1 << RES;     //1
    delay(5);
                
    lcd_init();        
    lcd_cls(bgcolor);    
    
	show_msg((char*)" DK7IH 2023");
		
    /////////////////////////
    //Rotary Encoder Setup //
    /////////////////////////
    //Set PB0, PB1 as input pins
    RCC->AHB1ENR |= (1 << 1);                           //GPIOB power up
    GPIOB->MODER &= ~((3 << (0 << 1))|(3 << (1 << 1))); //PB0 und PB1 for Input
    GPIOB->PUPDR |= (1 << (0 << 1))|(1 << (1 << 1));    //Pullup PB0 und PB1
    
    RCC->APB2ENR |= (1 << 14); //Enable SYSCFG clock (APB2ENR: bit 14)
    
    SYSCFG->EXTICR[0] |= 0x0001;  //Write 0b01 to map PB0 to EXTI0
    EXTI->RTSR |= 0x01;           //Enable rising edge trigger on EXTI0
    EXTI->IMR |= 0x01;            //Mask EXTI0

    //Initialize interrupt controller
    NVIC_SetPriorityGrouping(3);
    NVIC_SetPriority(EXTI0_IRQn, 1); //Set Priority for each interrupt request Priority level 1
    NVIC_EnableIRQ(EXTI0_IRQn);      //Enable EXT0 IRQ from NVIC

    /////////////////////////
    //TIMER2 Setup         //
    /////////////////////////
    //Enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
    
    //Timer calculation
    //Timer update frequency = TIM_CLK/(TIM_PSC+1)/(TIM_ARR + 1) 
    TIM2->PSC = 10000-1;   //Divide system clock (f=100MHz) by 10000 -> update frequency = 10000/s
    TIM2->ARR = 3500;      //Define overrun

    //Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    TIM2->CR1 |= (1 << 0);           //Enable Timer 2 module (CEN, bit0)
    
    NVIC_SetPriority(TIM2_IRQn, 2); //Priority level 2
    NVIC_EnableIRQ(TIM2_IRQn);      //Enable TIM2 IRQ from NVIC
    
    /////////////////////////
    //ADC1 Init            //
    /////////////////////////
    //Port config
    for(t0 = 4; t0 < 7; t0++) //PA4:PA7 to analog mode
    {
        GPIOA->MODER |= (3 << (t0 << 1));           //Set pins to analog mode
    }
        
    //ADC config sequence
    RCC->APB2ENR |= (1 << 8);	                    //Enable ADC1 clock (Bit8) 
    ADC1->CR1 &= ~(1 << 8);			                //SCAN mode disabled (Bit8)
	ADC1->CR1 &= ~(3 << 24);				        //12bit resolution (Bit24,25 0b00)
	ADC1->SQR1 &= ~(0x0F << 20);                    //Set number of conversions projected (L[3:0] 0b0000)
	ADC1->SQR3 &= ~(0x3FFFFFFF);	                //Clears whole 1st 30bits in register
	ADC1->SQR3 |= (4 << 0);			                //First conversion in regular sequence: PB4 as ADC1_4 (-> select ADC channel 4 as first (and only) conversion)
    ADC1->CR2 &= ~(1 << 1);			                //Single conversion ADON=1
	ADC1->CR2 &= ~(1 << 11);			            //Right alignment of data bits  bit12....bit0
	ADC1->SMPR2 |= (7 << 0);	 		            //Sampling rate 480 cycles. 16MHz bus clock for ADC. 1/16MHz = 62.5ns. 480*62.5ns=30us
    ADC1->CR2 |= (1 << 0);                          //Switch on ADC1
    //ADC ready for use
    
    /////////////////
	//I2C Setup    //
	/////////////////
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; //Enable I2C clock
    RCC->AHB1ENR |= (1 << 1);          //GPIOB power up
    GPIOB->MODER &= ~(3 << (6 << 1)); //PB6 as SCK
    GPIOB->MODER |=  (2 << (6 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 6);        //open-drain
    GPIOB->MODER &= ~(3 << (9 << 1)); //PB9 as SDA
    GPIOB->MODER |=  (2 << (9 << 1)); //Alternate function
    GPIOB->OTYPER |= (1 << 9);        //open-drain

    //Choose AF4 option for I2C1 in Alternate Function registers
    GPIOB->AFR[0] |= (4 << (6 << 2));     // for PB6
    GPIOB->AFR[1] |= (4 << ((9 - 8) << 2)); // for PB9

    //Reset and clear control register
    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    //Enable error interrupt
    I2C1->CR2 |= (I2C_CR2_ITERREN); 

    //Set I2C clock
    I2C1->CR2 |= (10 << 0); //10Mhz peripheral clock
    I2C1->CCR |= (50 << 0);
    //Maximum rise time set
    I2C1->TRISE |= (11 << 0); //TRISE=11ns for 100khz
    
    //Enable I2C
    I2C1->CR1 |= I2C_CR1_PE; 
    
    /////////////////////////
    //DDS Setup            //
    /////////////////////////
    
    //Put pin B15:B12 to general purpose output mode
    DDS_GPIO->MODER  |=  (1 << (DDS_IO_UD << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_SCLK << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_SDIO << 1));	
    DDS_GPIO->MODER  |=  (1 << (DDS_RESET << 1));	
    
    //Reset DDS (AD9951)
	DDS_GPIO->ODR |= (1 << DDS_RESET);  
	delay(10);
	DDS_GPIO->ODR &= ~(1 << DDS_RESET);  
    delay(10);
	DDS_GPIO->ODR |= (1 << DDS_RESET);  
	
	set_clock_multiplier();
     
    /////////////////////////
    //    Load TRX data    //
    /////////////////////////
    show_msg((char*)"Loading... ");
    
    cur_band = load_last_band();
    if(cur_band < 0)
    {
		cur_band = 4;
	}	
	set_band_relay(cur_band);
	
	cur_vfo = load_last_vfo();
    if(cur_vfo < 0)
    {
		cur_vfo = 0;
	}	
	
	f_vfo[cur_band][cur_vfo] = load_frequency(cur_band, cur_vfo);
    if(!is_freq_ok(f_vfo[cur_band][cur_vfo], cur_band))
    {
		f_vfo[cur_band][cur_vfo] = f_cntr[cur_band];
	}	
	
    sideband = pref_sideband[cur_band];
     
  	set_frequency(f_vfo[cur_band][cur_vfo]);
	set_frequency(f_vfo[cur_band][cur_vfo]);

    show_frequency(f_vfo[cur_band][cur_vfo]);
    show_sideband(sideband);
    sideband_old = sideband;
    show_band(cur_band); 
    
    show_vfo(cur_vfo);
    show_pa_temp(get_pa_temp());
    show_vdd(get_vdd());
    show_txrx(0);
    show_tunspeed(tunspeed);
    lcd_drawhline(0, 160, 19, LIGHTBLUE);
    lcd_drawhline(0, 160, 75, LIGHTBLUE);
    show_msg((char*)"VFOs...     ");
    
    //Load rest of VFOs
    for(t0 = 0; t0 < MAXBANDS; t0++)
	{
	    for(t1 = 0; t1 < MAXVFO; t1++)
		{	
			f_vfo[t0][t1] = load_frequency(t0, t1);
			if(!is_freq_ok(f_vfo[t0][t1], t0))
            {
		        f_vfo[t0][t1] = f_cntr[t0];
	        }	
		}
	}		
    
    show_msg((char*)"V 23-OCT-11");
    
	while(1)
    {   
		if(tuning)
		{
			switch(tunspeed)
	        {
	            case 0: f_vfo[cur_band][cur_vfo] += pulses  * tuning * -1; //Slow
	                    break;
	            case 1: f_vfo[cur_band][cur_vfo] += pulses  * tuning * -1 * (pulses >> 2); //Fast
	                    break;
	            case 2: f_vfo[cur_band][cur_vfo] += pulses  * tuning * -1 * pulses; //Superfast
	                    break;        
	            case 3: f_vfo[cur_band][cur_vfo] += pulses  * tuning * -1 * (pulses << 1); //Ultrafast
	                    break;                
	        }
	         
			set_frequency(f_vfo[cur_band][cur_vfo]);		
			show_frequency(f_vfo[cur_band][cur_vfo]);
			tuning = 0;
		}	
								
		key = get_keys();
	      
        switch(key)
        {
			case 0: if(cur_band < (MAXBANDS - 1))
			        {
						cur_band++;
					}
					else
					{
						cur_band = 0;
						show_frequency(0);
					}	
					
					
					if(!is_freq_ok(f_vfo[cur_band][cur_vfo], cur_band))
					{
						f_vfo[cur_band][cur_vfo] = f_cntr[cur_band];
					}	
							
					set_frequency(f_vfo[cur_band][cur_vfo]);
					show_frequency(f_vfo[cur_band][cur_vfo]);
					show_band(cur_band);
					set_band_relay(cur_band);
	                   show_sideband(pref_sideband[cur_band]);
					//mcp4725_setvalue(load_txdrvive(cur_band));
					store_last_band(cur_band);
					break;
                    
            case 1: if(cur_band > 0)
			        {
						if(cur_band == 4)
						{
							show_frequency(0);
						}	
							
						cur_band--;
					}
					else
					{
						cur_band = MAXBANDS - 1;
					}	
					
					
					if(!is_freq_ok(f_vfo[cur_band][cur_vfo], cur_band))
					{
						f_vfo[cur_band][cur_vfo] = f_cntr[cur_band];
					}	
							
					set_frequency(f_vfo[cur_band][cur_vfo]);
					show_frequency(f_vfo[cur_band][cur_vfo]);
					show_band(cur_band);
					set_band_relay(cur_band);
	                show_sideband(pref_sideband[cur_band]);
					//mcp4725_setvalue(load_txdrvive(cur_band));
					store_last_band(cur_band);
					break;	
					
			case 2: if(cur_vfo < MAXVFO) //VFO select
		            {
						cur_vfo++;
					}
					else	
					{
						cur_vfo = 0;
					}
					show_vfo(cur_vfo);
					
					if(is_freq_ok(load_frequency(cur_band, cur_vfo), cur_band))
					{
						f_vfo[cur_band][cur_vfo] = load_frequency(cur_band, cur_vfo);
					}
					else	
					{
						f_vfo[cur_band][cur_vfo] = f_cntr[cur_band];
					}
					set_frequency(f_vfo[cur_band][cur_vfo]);	
					show_frequency(f_vfo[cur_band][cur_vfo]);
					break;
						
			case 3: if(tunspeed < 4)
			        {
						tunspeed++;
					}
					else
					{
						tunspeed = 0;
					}	
					show_tunspeed(tunspeed);
					break;
					
			case 4: show_msg((char*)"Storing...  ");
			        store_frequency(f_vfo[cur_band][cur_vfo], cur_band, cur_vfo);
			        store_last_vfo(cur_vfo);
			        store_last_band(cur_band);
			        for(t0 = 0; t0 < MAXBANDS; t0++)
			        {
					    for(t1 = 0; t1 < MAXVFO; t1++)
			            {	
							store_frequency(f_vfo[t0][t1], t0, t1);
						}
					}		
					show_msg((char*)"Stored.    ");
					delay(100);
					runsecs_msg = runsecs;
					break;			
		}					
		
		//Show VDD, PATMP every 10 secs
		if(runsecs > runsecs_meas + 3)
		{
			show_pa_temp(get_pa_temp());
			volts = get_vdd();
			if(volts != volts_old)
			{
		        show_vdd(get_vdd());	
		        volts_old = volts;
		    }    
			runsecs_meas = runsecs;
		}	

		if(GPIOA->IDR & (1 << 7))
		{
			if(!tr_old)
			{
			    show_txrx(1);
			    tr_old = 1;
			}    
		}
		else	
		{
			if(tr_old)
			{
			    show_txrx(0);
			    tr_old = 0;
			}    
		}
				
		if((runsecs_msg  + 5) < runsecs)
		{
			runsecs_msg = (1 << 31);
            show_msg((char*)"*DK7IH/QRP*");
		}	
		
		if(GPIOB->IDR & (1 << 7))
		{
			sideband = 1;
		}
		else
		{
			sideband = 0;
		}
		
		if(sideband_old != sideband)
		{
			show_sideband(sideband);
			sideband_old = sideband;
		}	
	}		
	return 0;
}
