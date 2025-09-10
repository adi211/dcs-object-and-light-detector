// private library - BSP layer

#include  "..\header\bsp_msp430x2xx.h"


//-----------Defining Global variables, used in higher hierarchies--------------

int g_script_delay_period_ms = 500;     // Timer delay value, may be changed by user in Script mode
int g_timer2_ccr0_value;        // Timer delay value, affected by 'g_script_delay_period_ms'


//------------------------------------------------------------------------------  
//                              GPIO configuration
//------------------------------------------------------------------------------
void GPIOconfig(void){
    WDTCTL = WDTHOLD | WDTPW;		  // Stop WDT
	__delay_cycles(50000); 
   
// PushButtons Setup
  
    PBsArrPortSel         &= ~(PB0 | PB1);       //  I/O mode
    PBsArrPortDir         &= ~(PB0 | PB1);       // Input mode
    PBsArrIntEdgeSel      |= PB0 | PB1;          // Interrupt on high-to-low edge
    PBsArrIntEn           |=  PB0 | PB1;         // enable interrupt
    PBsArrIntPend         &= ~(PB0| PB1);        // clear pending interrupt
	
	

// LDR setup        LDR1, P1.0 (A0)  ,  LDR2, P1.3 (A3)
    LDRPortSel        &= ~LDR1leg; // LDR1 Input
    LDRPortSel        &= ~LDR2leg; // LDR2 Input
    LDRPortDir        &= ~LDR1leg; // LDR1 Input capture
    LDRPortDir        &= ~LDR2leg; // LDR2 Input capture


// Servo motor setup 
    ServoPortSel        |=   ServoPwmLeg;   // Timer Mode
    ServoPortDir        |=   ServoPwmLeg;   //Output mode
    

// Telemeter setup 

    // Configure P2.6 (and P2.7) to PWM out
    // Note: P1.2, P1.6 and P2.6 can all three be used as TA0.CCR1 PWM output. However, only P1.2 can be used as TA0.CCR1.CCI1A capture input.

    ULTRASONIC_PORT_DIR        |=     ULTRASONIC_TRIG_PIN;
    ULTRASONIC_PORT_SEL        &=    ~ULTRASONIC_TRIG_PIN;
    //ULTRASONIC_PORT_SEL        &=   ~ TeleTrigLeg2;
    ULTRASONIC_PORT_SEL2       &=   ~(ULTRASONIC_TRIG_PIN | PB1); //to validate no noise on pb1


    // Configure P2.2 to timer mode, Input capture
    ULTRASONIC_PORT_SEL        |=    ULTRASONIC_ECHO_PIN;
    ULTRASONIC_PORT_DIR        &=   ~ULTRASONIC_ECHO_PIN;


    _BIS_SR(GIE);                           // enable interrupts globally

}                             
//------------------------------------------------------------------------------
//                             Timers configuration
//------------------------------------------------------------------------------

void Timers_Init(void) {
    // --- Timer_A0 (was Timer_2) Default Configuration ---
    // Set clock source to SMCLK and divider to /8, but keep the timer stopped.
    // MC_0 is the Stop mode, which is a safe default state.
    Timer_2_CTL = TASSEL_2 + ID_3 + MC_0; // SMCLK, /8, Timer is Off

    // --- Timer_A1 (was Timer_1) Default Configuration ---
    // Set clock source to SMCLK, but keep the timer stopped.
    Timer_1_CTL = TASSEL_2 + MC_0; // SMCLK, Timer is Off
}

//------------------------------------------------------------------------------
//                              ADC configuration (Using timer A1)
//------------------------------------------------------------------------------
void ADC_config(void){



  Adc_CTL0 = Adc_ON + Adc_IE+ Adc_SHT_3+SREF_0;   // ADC10 On/Enable           +
                                                  // Interrupt enable          +
                                                  // use 64 x ADC10CLK cycles  +
                                                  // Set ref to Vcc and Gnd

  Adc_CTL1 = LDR1_CHANNEL+Adc_SSEL_3;             // Input channel A0 (p1.0) + SMCLK
  ADC10AE0 |= (BIT0 | BIT3);

}


//------------------------------------------------------------------------------
//                             UART configuration
//------------------------------------------------------------------------------

void UART_init(void){
    
    // Clock setup - ensure 1MHz for accurate timing
    if (CALBC1_1MHZ==0xFF)                  // Check factory calibration exists
      {
        while(1);                           // Trap CPU if calibration missing
      }
    DCOCTL = 0;                             // Reset oscillator settings
    BCSCTL1 = CALBC1_1MHZ;                  // Load 1MHz coarse calibration
    DCOCTL = CALDCO_1MHZ;                   // Load 1MHz fine calibration

    // Pin configuration
    P1SEL |= BIT1 + BIT2;                   // P1.1 = RXD, P1.2 = TXD
    P1SEL2 |= BIT1 + BIT2;                  // Enable UART function


    // UART registers
    UCA0CTL1 |= UCSSEL_2;                   // Use SMCLK (1MHz)

    UCA0BR0 = 104;                          // Baud rate: 1MHz/104 = 9615 ≈ 9600
    UCA0BR1 = 0x00;                         // Upper byte = 0
    
    UCA0MCTL = UCBRS0;                      // Modulation for accuracy
    UCA0CTL1 &= ~UCSWRST;                   // Enable UART (exit reset)
	IFG2 &= ~UCA0RXIFG;
	IE2 |= UCA0RXIE;                        // Enable USCI_A0 RX interrupt
}
 



