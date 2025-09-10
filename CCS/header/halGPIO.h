#ifndef _halGPIO_H_
#define _halGPIO_H_

#include "..\header\bsp_msp430x2xx.h"
#include "..\header\app.h"          // private library - APP layer
#include "..\header\api.h"      


//==============================================================================
//                       HAL "PUBLIC" FUNCTION PROTOTYPES
//==============================================================================
// These are the functions that higher layers (like api.c) are intended to use.

//------------------------------------------------------------------------------
//                             System Configuration
//------------------------------------------------------------------------------
extern void sysConfig(void);

//------------------------------------------------------------------------------
//                             General Purpose
//------------------------------------------------------------------------------
extern void enable_interrupts(void);
extern void disable_interrupts(void);
extern void enterLPM(unsigned char LPM_level);

//------------------------------------------------------------------------------
//                             LCD Public API
//------------------------------------------------------------------------------
extern void lcd_init(void); // Should be called once from sysConfig
extern void lcd_puts(const char *s);
//extern void lcd_clear(); // Note: This is a macro in bsp.h


//------------------------------------------------------------------------------
//                             Servo PWM Driver
//------------------------------------------------------------------------------
extern void pwmOutServoConfig(int pulse_width);

//------------------------------------------------------------------------------
//                             Timer Drivers
//------------------------------------------------------------------------------
extern void Timer_Start_Script_Delay_ms(int delay_ms);
extern void Timer_Start_General_Delay_ms(int delay_ms);
extern void Timer_Stop_Delay_ms(void);
extern void Ultrasonic_Start_Measurement(void);
extern void Ultrasonic_Stop_Measurement(void);
void Timer_Start_Ultrasonic_Timeout(void);
extern void Timer_Start_General_Delay_sec(unsigned int delay_sec);

extern void Set_Angle_PWM(int pulse_width);

extern void start_ultrasonic_timer();
extern void stop_ultrasonic_timer();


//------------------------------------------------------------------------------
//                             ADC Driver
//------------------------------------------------------------------------------
extern void ADC_Enable(void);
extern void ADC_Disable(void);
extern unsigned int ADC_sample(void);
extern void ADC_Configure_LDR1(void);
extern void ADC_Configure_LDR2(void);

//------------------------------------------------------------------------------
//                             UART TX Drivers
//------------------------------------------------------------------------------
extern void UART_send_char(char a);
extern void UART_send_tele_distance(void);
extern void UART_send_tele_angle(void);
extern void UART_send_string_blocking(const char *s);

// ... (add other UART send function prototypes if you have them)

//------------------------------------------------------------------------------
//                             Flash Memory Driver
//------------------------------------------------------------------------------
extern void init_flash_write(int addr);
extern void disable_flash_write(void);
extern void write_flash_char(unsigned char value);



//------------------------------------------------------------------------------
// UART Circular Buffer
//------------------------------------------------------------------------------


extern void UartBuffer_Init(void);
extern int UartBuffer_Put(char c);
extern int UartBuffer_Get(unsigned char *c);


extern char *Flash_ptr;

extern volatile int echo_pulse_time_us;


#endif /* _halGPIO_H_ */

