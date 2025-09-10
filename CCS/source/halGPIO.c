#include  "..\header\halGPIO.h"     // private library - HAL layer
#include "..\header\bsp_msp430x2xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <msp430.h>

//==============================================================================
//                      INTERNAL (STATIC) GLOBAL VARIABLES
//==============================================================================

// Static variables to manage the UART TX state machine
static volatile int g_tx_index = 0;
static volatile int g_tx_length = 0;

// --- Ultrasonic Measurement State ---
static volatile int echo_arr[2];
static volatile int capture_index = 0;
volatile int echo_pulse_time_us;

// --- Step Delay Timer State ---
static volatile int timer_counter = 0;





static volatile unsigned char g_rx_buffer[RX_BUFFER_SIZE];
static volatile int g_rx_buffer_head = 0;
static volatile int g_rx_buffer_tail = 0;

static enum {
    TIMER_MODE_SCRIPT_DELAY,
    TIMER_MODE_ULTRASONIC_TIMEOUT,
    TIMER_MODE_GENERAL_DELAY 
} g_timer0_mode;


char *Flash_ptr;


//==============================================================================
//                       SYSTEM CONFIGURATION
//==============================================================================
void sysConfig(void){ 
    GPIOconfig();
    UART_init();
    Timers_Init();
    lcd_init();
}

//==============================================================================
//                      1. LOW-LEVEL HARDWARE DRIVERS
//==============================================================================

//------------------------------------------------------------------------------
//                             LCD Driver
//------------------------------------------------------------------------------

//       lcd strobe functions
void lcd_strobe(){
  LCD_EN(1);
  __delay_cycles(2);
  LCD_EN(0);
}

//    send a command to the LCD
void lcd_cmd(unsigned char c){
  
	LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

	if (LCD_MODE == FOURBIT_MODE)
	{
		LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
		lcd_strobe();
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
    		LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
		lcd_strobe();
	}
	else
	{
		LCD_DATA_WRITE = c;
		lcd_strobe();
	}
}

//      send data to the LCD
void lcd_data(unsigned char c){
        
	LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

	LCD_DATA_WRITE &= ~OUTPUT_DATA;       
	LCD_RS(1);
	if (LCD_MODE == FOURBIT_MODE)
	{
    		LCD_DATA_WRITE &= ~OUTPUT_DATA;
                LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;  
		lcd_strobe();		
                LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
                LCD_DATA_WRITE &= ~OUTPUT_DATA;
		LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET; 
		lcd_strobe();
	}
	else
	{
		LCD_DATA_WRITE = c;
		lcd_strobe();
	}
          
	LCD_RS(0);   
}

//      initialize the LCD
void lcd_init(){
  
	char init_value;
    
        P2SEL &= 0xD5; // 11010101 clear bits p2.5, p2.3, p2.1
        
	if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
        else init_value = 0x3F;
	
	LCD_RS_DIR(OUTPUT_PIN);
	LCD_EN_DIR(OUTPUT_PIN);
	LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
	LCD_EN(0);
	LCD_RW(0);
        
	DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
	LCD_DATA_WRITE |= init_value;
	lcd_strobe();
	
	if (LCD_MODE == FOURBIT_MODE){
		LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
		LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
		lcd_strobe();
		lcd_cmd(0x28); // Function Set
	}
        else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots 
	
	lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
	lcd_cmd(0x1); //Display Clear
	lcd_cmd(0x6); //Entry Mode
	lcd_cmd(0x80); //Initialize DDRAM address to zero
}

// write a string of chars to the LCD
void lcd_puts(const char * s){
    while(*s)
        lcd_data(*s++);
}



//------------------------------------------------------------------------------
//                             Flash Memory Driver
//------------------------------------------------------------------------------
void Flash_SetTiming(){

     FCTL2 = FWKEY + FSSEL0 + FN1;             // MCLK/3 for Flash Timing Generator
	 
}

void init_flash_write(int addr){
    Flash_ptr = (char *) addr;                // Initialize Flash pointer  // 0x1000,0x1040,0x1080
    FCTL1 = FWKEY + ERASE;                    // Set Erase bit
    FCTL3 = FWKEY;                            // Clear Lock bit
    *Flash_ptr = 0;                           // Dummy write to erase Flash segment
    FCTL1 = FWKEY + WRT;                      // Set WRT bit for write operation
}

void disable_flash_write(){
    FCTL1 = FWKEY;                            // Clear WRT bit
    FCTL3 = FWKEY + LOCK;                     // Set LOCK bit
}


void write_flash_char(unsigned char value){
    *Flash_ptr++ = value;               // Write value to flash
}





/**
 * @brief Writes a block of data from RAM to a specified Flash memory address.
 * @note THIS FUNCTION ERASES THE ENTIRE SEGMENT before writing.
 * Ensure that target_flash_addr is the start of a segment you intend to overwrite.
 * @param source_ram_ptr Pointer to the source data in RAM.
 * @param target_flash_addr The destination address in Flash.
 * @param num_bytes The number of bytes to write.
 */
void Flash_WriteBlock(void* source_ram_ptr, unsigned int target_flash_addr, unsigned int num_bytes) {
    // Cast the generic void pointer to a char pointer for byte-wise access
    char* data_ptr = (char*)source_ram_ptr;
    int i;

    // --- Critical Section: Disable interrupts during Flash operations ---
    disable_interrupts();

    // 1. Configure Flash controller timing
    Flash_SetTiming();

    // 2. Erase the target segment and prepare for writing
    init_flash_write(target_flash_addr);

    // 3. Write the data block byte by byte
    for (i = 0; i < num_bytes; i++) {
        write_flash_char(data_ptr[i]);
    }

    // 4. Finalize and lock the Flash controller
    disable_flash_write();

    // --- End of Critical Section: Re-enable interrupts ---
    enable_interrupts();
}









// Configures the Flash controller timing. Must be called before any erase/write.
void FlashConfig(void){
     FCTL2 = FWKEY + FSSEL0 + FN1; // MCLK/3 for Flash Timing
}

// Erases a single segment containing the specified address.
void Flash_EraseSegment(unsigned int addr) {
    Flash_ptr = (char*)addr;
    FCTL1 = FWKEY + ERASE;
    FCTL3 = FWKEY;
    *Flash_ptr = 0; // Dummy write to trigger erase
    FCTL1 = FWKEY; // Clear ERASE bit
}

// Prepares the Flash controller for a sequence of write operations.
void Flash_StartWrite(unsigned int addr) {
    Flash_ptr = (char*)addr;
    FCTL1 = FWKEY + WRT;
    FCTL3 = FWKEY; // Ensure lock is clear
}

// Writes a single byte and advances the internal pointer.
void Flash_WriteByte(unsigned char value) {
    *Flash_ptr++ = value;
}

// Locks the Flash controller, ending the write sequence.
void Flash_Close(void) {
    FCTL1 = FWKEY; // Clear WRT bit
    FCTL3 = FWKEY + LOCK; // Set LOCK bit
}






// In halGPIO.c

/**
 * @brief Writes a block of data to Flash WITHOUT erasing first.
 * @warning Use ONLY on memory that is already erased (0xFF) or if you
 * intend to perform a logical AND with existing data.
 */
void Flash_WriteData_NoErase(void* source_ram_ptr, unsigned int target_flash_addr, unsigned int num_bytes) {
    char* data_ptr = (char*)source_ram_ptr;
    int i;

    disable_interrupts();
    FlashConfig();
    Flash_StartWrite(target_flash_addr); // Prepares for writing, does NOT erase

    for (i = 0; i < num_bytes; i++) {
        Flash_WriteByte(data_ptr[i]);
    }

    Flash_Close();
    enable_interrupts();
}





//==============================================================================
//                      2. HARDWARE ABSTRACTION LAYER (HAL)
//==============================================================================

//------------------------------------------------------------------------------
//                             General Purpose
//------------------------------------------------------------------------------
void enable_interrupts(){
  _BIS_SR(GIE);
}

void disable_interrupts(){
  _BIC_SR(GIE);
}

void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits + GIE);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits + GIE);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits + GIE);     /* Enter Low Power Mode 2 */
        else if(LPM_level == 0x03)
	  _BIS_SR(LPM3_bits + GIE);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits + GIE);     /* Enter Low Power Mode 4 */
}

//------------------------------------------------------------------------------
//                             Servo PWM Driver
//------------------------------------------------------------------------------
void pwmOutServoConfig(int pulse_width){
	disable_interrupts();
    Timer_1_CCR0 = (int) 21600;
    Timer_1_CCR2 = (int) pulse_width;
    Timer_1_CCTL2 = OUTMOD_7;
    Timer_1_CTL = TASSEL_2 + MC_1 + TACLR; // SMCLK, Up mode
	 enable_interrupts(); 
}

void Set_Angle_PWM(int pulse_width){
    Timer_1_CCR2 = (int) pulse_width;
}

void stopServoPwm(void){

    Timer_1_CTL = MC_0; 
    Timer_1_CCTL2 = 0;
}

//------------------------------------------------------------------------------
//                             Timer Drivers
//------------------------------------------------------------------------------


void start_ultrasonic_timer(){
	Timer_1_CTL = TASSEL_2 + MC_2 + TACLR;  

}

void stop_ultrasonic_timer(){
	Timer_1_CTL = MC_0; 
}


void Timer_Start_Script_Delay_ms(int delay_ms) {
	
	g_timer0_mode = TIMER_MODE_SCRIPT_DELAY;
	
    /*
     * Converts a desired delay in milliseconds into the required value for the TACCR0 register.
     *
     * Parametric Formula:
     * TACCR0_Value = (Delay_ms * F_smclk) / (1000 * Divider * Mode_Factor * ISR_Count)
     *
     * Where:
     * - Delay_ms:     Desired delay in milliseconds.
     * - F_smclk:      System clock frequency in Hz.
     * - Divider:      Timer's input clock prescaler.
     * - Mode_Factor:  2 for Up/Down mode, 1 for Up mode.
     * - ISR_Count:    Number of interrupts before the CPU wakes up.
     *
     * Numerical Substitution for this project:
     * - F_smclk:      1,000,000 Hz
     * - Divider:      8
     * - Mode_Factor:  2 (Up/Down mode)
     * - ISR_Count:    16
     *
     * TACCR0_Value = (delay_ms * 1,000,000) / (1000 * 8 * 2 * 16)
     * = (delay_ms * 1,000,000) / 256,000
     * = delay_ms * 3.90625
     *
     * This value is approximated to 4 for simplicity and integer math.
     */
    g_timer2_ccr0_value = 4 * delay_ms;
    Timer_2_CCR0 = (unsigned int)g_timer2_ccr0_value;
    Timer_2_CTL = MC_3 + TASSEL_2 + ID_3 + TACLR;
    Timer_2_CCTL0 |= CCIE;
}

void Timer_Start_General_Delay_ms(int delay_ms) {
    g_timer0_mode = TIMER_MODE_GENERAL_DELAY;
    // We start the timer in simple "Up" mode.
    // SMCLK is 1MHz, divider is /8, so timer clock is 125,000 Hz.
    // 125 ticks = 1 ms.
    Timer_2_CCR0 = delay_ms * 125;
    Timer_2_CTL = TASSEL_2 + ID_3 + MC_1 + TACLR; // SMCLK, /8, Up mode
    Timer_2_CCTL0 |= CCIE;
}

void Timer_Start_General_Delay_sec(unsigned int sec) {
	unsigned long total_ms = (unsigned long)sec * 1000;

    // בצע לולאה עד שכל זמן ההשהיה חלף
    while (total_ms > 0)
    {
        // קבע את גודל מקטע ההשהיה הנוכחי.
        // נשתמש ב-500ms כמקטע בטוח, שנמצא בתוך מגבלות הטיימר.
        unsigned int current_delay_chunk = (total_ms > 500) ? 500 : (unsigned int)total_ms;

        // הפעל את הטיימר החומרתי עבור המקטע הנוכחי
        Timer_Start_General_Delay_ms(current_delay_chunk);
        
        // כנס למצב שינה (LPM) כדי לחסוך בחשמל בזמן ההמתנה
        enterLPM(lpm_mode);
        // פסיקת הטיימר תעיר את המעבד ממצב השינה

        // הפחת את הזמן שחלף מהזמן הכולל שנותר
        total_ms -= current_delay_chunk;
    }
	
	Timer_Stop_Delay_ms();
}



void Timer_Start_Ultrasonic_Timeout(void) {
    g_timer0_mode = TIMER_MODE_ULTRASONIC_TIMEOUT; // Set the mode for timeout
    Timer_2_CCR0 = 50000; // ~50ms timeout duration
    Timer_2_CCTL0 = CCIE; // Enable CCR0 interrupt
    Timer_2_CTL = TASSEL_2 + MC_1 + TACLR; // Start Timer_A0 in Up mode
}



void Timer_Stop_Delay_ms(void) {
    Timer_2_CTL &= ~MC_3; 
    Timer_2_CCTL0 &= ~CCIE;
	TA0CCTL0 &= ~CCIFG; 
}

void Ultrasonic_Start_Measurement(void) {
    capture_index = 0;

    // STEP 1: Start the timeout timer using our new clean function.
    Timer_Start_Ultrasonic_Timeout();

    // STEP 2: SEND THE TRIGGER PULSE
    ULTRASONIC_PORT_OUT |= ULTRASONIC_TRIG_PIN;
    __delay_cycles(15);
    ULTRASONIC_PORT_OUT &= ~ULTRASONIC_TRIG_PIN;
	
	// STEP 3: PREPARE THE ECHO CAPTURE TIMER (Timer_A1)
    
    Timer_1_CCTL1 = CAP + CCIE + CCIS_1 + CM_3 + SCS;
}

void Ultrasonic_Stop_Measurement(void) {
    Timer_2_CTL = MC_0; // Stop Timer_A0 (the timeout timer)
    Timer_2_CCTL0 &= ~CCIE; // Disable its interrupt
	Timer_2_CCTL0 &= ~CCIFG; 
    //Timer_1_CTL = MC_0; // Stop Timer_A1 (the capture timer)
    Timer_1_CCTL1 = 0;
}

//------------------------------------------------------------------------------
//                             ADC Driver
//------------------------------------------------------------------------------
void ADC_Enable(void){
    Adc_CTL0 |= Adc_ON + ENC + ADC_SC + Adc_IE;
}

void ADC_Disable(void){
    Adc_CTL0 &= ~(Adc_ON + ENC + ADC_SC + ADC10IE);
}

unsigned int ADC_sample(void){
    return (unsigned int)Adc_Mem;
}

void ADC_Configure_LDR1(void){
    Adc_CTL1 = LDR1_CHANNEL + Adc_SSEL_3; // Input channel A0 (p1.0) + SMCLK
}

void ADC_Configure_LDR2(void){
    Adc_CTL1 = LDR2_CHANNEL + Adc_SSEL_3; // Input channel A3 (p1.3) + SMCLK
}

//------------------------------------------------------------------------------
//                             UART TX Drivers
//------------------------------------------------------------------------------

void UART_send_string_blocking(const char *s) {
    while (*s) {
        // Poll the flag: wait until the TX buffer is ready.
        while (!(IFG2 & UCA0TXIFG));
        // Send one character and move to the next.
        UCA0TXBUF = *s++;
    }
}


void UART_send_char(char a) {
    while (!(IFG2 & UCA0TXIFG)); // Wait until the TX buffer is ready
    UCA0TXBUF = a;              // Send the character
}


//==============================================================================
//                      3. INTERRUPT SERVICE ROUTINES (ISRs)
//==============================================================================

#pragma vector=PORT2_VECTOR
  __interrupt void PBs_handler(void){
      DelayMs(150);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------
	if (PBsArrIntPend & PB0){
		g_button_pressed = PB0;
        PBsArrIntPend &= ~PB0;
    }
	if (PBsArrIntPend & PB1){
		g_button_pressed = PB1;
        PBsArrIntPend &= ~PB1;
    }
//---------------------------------------------------------------------
//            Exit from a given LPM 
//---------------------------------------------------------------------	
	 switch(lpm_mode){
		case mode0: LPM0_EXIT; break;
		case mode1: LPM1_EXIT; break;
		case mode2: LPM2_EXIT; break;
		case mode3: LPM3_EXIT; break;
		case mode4: LPM4_EXIT; break;
	 }     
}

#pragma vector=TIMER1_A1_VECTOR
 __interrupt void Timer_1_ISR(void){
							  // ECHO input capture
	echo_arr[capture_index] = (int)Timer_1_CCR1;
	capture_index += 1;
	Timer_1_CCTL1 &= ~CCIFG;
	Timer_1_CTL   &= ~Timer_1_IFG;          //turn off flag
	if(capture_index==2){
		 echo_pulse_time_us = echo_arr[1]-echo_arr[0];
		 capture_index=0;
		 LPM0_EXIT;  //out from sleep
	}
    
 }



#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR_Handler(void) {
   
	TA0CCTL0 &= ~CCIFG;

   switch (g_timer0_mode) {
		
        case TIMER_MODE_SCRIPT_DELAY:
            // This is the logic from the original ISR for script delays
            if (timer_counter == 16) {
                timer_counter = 0;
                LPM0_EXIT; // Wake from sleep
            }
            timer_counter++;
            break;

        case TIMER_MODE_ULTRASONIC_TIMEOUT:
			TA0CCTL0 &= ~CCIE;
            // This is the new logic for the ultrasonic measurement timeout
            echo_pulse_time_us = 0; // Set distance to 0 to indicate a failed measurement
            LPM0_EXIT; // Wake the CPU to prevent a deadlock
            break;
			
		case TIMER_MODE_GENERAL_DELAY:
            LPM0_EXIT; 
            break;
    }
	 
}

#pragma vector=ADC10_VECTOR
__interrupt void ADC_ISR(void){
    LPM0_EXIT;
}


//            UART Interrupt Service Rotine

//   Transmit (TX)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0TX_VECTOR))) USCI0TX_ISR (void)
#else
#error Compiler not supported!
#endif
{

}

//  Receive (RX)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
	UartBuffer_Put(UCA0RXBUF);
    LPM0_EXIT;
	
}

void UartBuffer_Init(void){
    g_rx_buffer_head = 0;
    g_rx_buffer_tail = 0;
}

// Function to be called from ISR to add a char to the buffer
int UartBuffer_Put( char c){
    int next_head = (g_rx_buffer_head + 1) % RX_BUFFER_SIZE;
    if (next_head == g_rx_buffer_tail) {
        return -1; // Buffer is full
    }
    g_rx_buffer[g_rx_buffer_head] = c;
    g_rx_buffer_head = next_head;
    return 0; // Success
}

// Function to be called from the main loop to get a char
int UartBuffer_Get(unsigned char *c){
    if (g_rx_buffer_head == g_rx_buffer_tail) {
        return -1; // Buffer is empty
    }
    *c = g_rx_buffer[g_rx_buffer_tail];
    g_rx_buffer_tail = (g_rx_buffer_tail + 1) % RX_BUFFER_SIZE;
    return 0; // Success
}


