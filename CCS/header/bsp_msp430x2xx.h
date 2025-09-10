#ifndef _bsp_H_
#define _bsp_H_

#include  <msp430g2553.h>          // MSP430x2xx


//------Exporting configuration functions and variables to higher hierarchies-----------
extern void GPIOconfig(void);
extern void Timers_Init(void);
extern void ADC_config(void);
extern void UART_init(void);
extern void FlashConfig(void);
extern int g_script_delay_period_ms;
extern int g_timer2_ccr0_value; 

#define  debounceVal  250

// Flash Memory Locations for Calibration Data
#define LDR1_CALIB_START_ADDR   0x1040  // Start of Segment C
#define LDR2_CALIB_START_ADDR   0x1080  // Start of Segment B


// --- Delay Macros 
// Assumes a 1MHz clock, where 1 cycle = 1 microsecond.
#define DelayUs(us)   __delay_cycles(us)
#define DelayMs(ms)   __delay_cycles((long)ms * 1000)



//-------------------------------------------------
// Timers abstraction
//-------------------------------------------------

// Timer1
#define Timer_1_CLR         TACLR
#define Timer_1_SSEL2       TASSEL_2
#define Timer_1_CCR1_IFG    TA1IV_TACCR1
#define Timer_1_CCR2_IFG    TA1IV_TACCR2
#define Timer_1_IFG         TAIFG
#define Timer_1_CTL         TA1CTL
#define Timer_1_CCR0        TA1CCR0
#define Timer_1_CCR1        TA1CCR1
#define Timer_1_CCR2        TA1CCR2
#define Timer_1_CCTL1       TA1CCTL1
#define Timer_1_CCTL2       TA1CCTL2
#define Timer_1_IE          TAIE

// Timer2
#define Timer_2_SSEL2       TASSEL_2 //SMCLK 2^20
#define Timer_2_CLR         TACLR
#define Timer_2_CTL         TA0CTL
#define Timer_2_CCR0        TA0CCR0
#define Timer_2_CCR1        TA0CCR1
#define Timer_2_CCTL0       TA0CCTL0
#define Timer_2_CCTL1       TA0CCTL1



//-------------------------------------------------
// ADC abstraction
//-------------------------------------------------
#define Adc_Mem             ADC10MEM
#define Adc_CTL0            ADC10CTL0
#define Adc_CTL1            ADC10CTL1
#define Adc_AE0             ADC10AE0
#define Adc_ON              ADC10ON
#define Adc_IE              ADC10IE
#define Adc_SHT_3           ADC10SHT_3
#define Adc_SSEL_3          ADC10SSEL_3
#define ADC_SC              ADC10SC


//-------------------------------------------------
// UART abstraction
//-------------------------------------------------
#define TXD                 BIT2
#define RXD                 BIT1


//-------------------------------------------------
// PushButtons abstraction
//-------------------------------------------------
//#define PBsArrPort	        P2IN
#define PBsArrIntPend	    P2IFG
#define PBsArrIntEn	        P2IE
#define PBsArrIntEdgeSel    P2IES
#define PBsArrPortSel       P2SEL
#define PBsArrPortDir       P2DIR
#define PB0                 0x01  //P2.0
#define PB1                 0x80  //P2.7



//-------------------------------------------------
// LDRs abstraction
//-------------------------------------------------
//#define LDRPortOut          P1OUT
#define LDRPortDir          P1DIR
#define LDRPortSel          P1SEL
#define LDR1leg             0x01  //p1.0
#define LDR2leg             0x08  //p1.3
#define LDR1_CHANNEL           INCH_0  // A0 (P1.0)
#define LDR2_CHANNEL           INCH_3  // A3 (P1.3)

//-------------------------------------------------
// Servo Motor abstraction
//-------------------------------------------------
#define ServoPortDir        P2DIR
#define ServoPortSel        P2SEL
//#define ServoPortSel2       P2SEL2
#define ServoPwmLeg         0x10       //p2.4



//-------------------------------------------------
// ULTRASONIC abstraction
//-------------------------------------------------
#define ULTRASONIC_PORT_DIR         P2DIR
#define ULTRASONIC_PORT_SEL         P2SEL
#define ULTRASONIC_PORT_SEL2        P2SEL2
#define ULTRASONIC_ECHO_PIN         0x04       //p2.2
#define ULTRASONIC_TRIG_PIN        0x40       //p2.6
#define ULTRASONIC_PORT_OUT         P2OUT


//-------------------------------------------------
// LCD abstraction
//-------------------------------------------------
#ifdef CHECKBUSY
  #define	LCD_WAIT lcd_check_busy()
#else
  #define LCD_WAIT __delay_cycles(5000)
#endif


/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)	    (!a ? (P2OUT&=~0X02) : (P2OUT|=0X02)) // P2.1 is lcd enable pin // INSTEAD OF 2.5 (0X20)
#define LCD_EN_DIR(a)	(!a ? (P2DIR&=~0X02) : (P2DIR|=0X02)) // P2.1 pin direction     // INSTEAD OF 2.5 (0X20)

#define LCD_RS(a)	    (!a ? (P2OUT&=~0X08) : (P2OUT|=0X08)) // P2.3 is lcd RS pin     // INSTEAD OF 2.6 (0X40)
#define LCD_RS_DIR(a)	(!a ? (P2DIR&=~0X08) : (P2DIR|=0X08)) // P2.3 pin direction     // INSTEAD OF 2.6 (0X40)
  
#define LCD_RW(a)	    (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd RW pin     // INSTEAD OF 2.7 (0X80)
#define LCD_RW_DIR(a)	(!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction     // INSTEAD OF 2.7 (0X80)

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset
   
#define LCD_DATA_WRITE	P1OUT
#define LCD_DATA_DIR	P1DIR
#define LCD_DATA_READ	P1IN


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE	0x0
#define EIGHTBIT_MODE	0x1
#define LCD_MODE        FOURBIT_MODE
   
#define OUTPUT_PIN      1	
#define INPUT_PIN       0	
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00	

#define LCD_STROBE_READ(value)	LCD_EN(1), \
				asm("nop"), asm("nop"), \
				value=LCD_DATA_READ, \
				LCD_EN(0) 

#define	lcd_cursor(x)		    lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()		        lcd_cmd(0x01)
#define lcd_putchar(x)		    lcd_data(x)
#define lcd_goto(x)		        lcd_cmd(0x80+(x))
#define lcd_cursor_right()	    lcd_cmd(0x14)
#define lcd_cursor_left()	    lcd_cmd(0x10)
#define lcd_display_shift()	    lcd_cmd(0x1C)
#define lcd_home()		        lcd_cmd(0x02)
#define cursor_off              lcd_cmd(0x0C)
#define cursor_on               lcd_cmd(0x0F) 
#define lcd_function_set        lcd_cmd(0x3C) // 8bit,two lines,5x10 dots 
#define lcd_new_line()          lcd_cmd(0xC0)                                  

#endif
