Final Project, DCS Course, Light source & object proximity detector system
==============================================================================
Hardware
==============================================================================
Layering: BSP → HAL → API/APP → main  (FSM, interrupt-driven)
I/O: UART 9600 bps 
Display: LCD 16×2
Sensors: HC-SR04 + two LDRs  
Motor: SG90 servo
==============================================================================
Pin out
==============================================================================
P1.0 - Analog input signal (channel A0) - LDR1
P1.1 - UART
P1.2 - UART
P1.3 - Analog input signal (channel A3) - LDR2
P1.4 - LCD Data D4
P1.5 - LCD Data D5
P1.6 - LCD Data D6
P1.7 - LCD Data D7
P2.0 - PB0 (for calibrating the LDR's)
P2.1 - LCD control signal E
P2.2 - Ultrasonic sensor output signal (ECHO) - Timer A1 CCR1
P2.3 - LCD control signal RS
P2.4 - Timer A1 CCR2 - PWM out (for servo motor)
P2.5 - LCD control signal RW
P2.6 - Ultrasonic sensor input signal  (Trig) - Timer A0 CCR1
P2.7 - PB1
==============================================================================
Source index
==============================================================================
MCU (C):
• headers: app.h, api.h, halGPIO.h, bsp_msp430x2xx.h
• sources: bsp.c, halGPIO.c, api.c, main.c

PC (Python):
• main.py
==============================================================================
[MCU / Headers]
==============================================================================
1) app.h
   • Constants & memory map: FILE_METADATA_ADDR, FILE_CONTENT_START_ADDR/END_ADDR,
     LDR1_CALIB_START_ADDR, LDR2_CALIB_START_ADDR, CHUNK_SIZE, FILE_ENTRY_SIZE, MAX_FILES, …
   • Enums: top-level FSM states and sub-FSMs (Objects / Telemeter / Lights / Files),
     ScanMode, FileType, UploadState, ViewLcdState.
   • Types: FileEntry (name, type, size, start address).
   • extern: shared state for filesystem counters, upload buffers/indices, FSM flags.

2) api.h
   • APP/API prototypes: object_detector, telemeter, light_detector, light_object_detector,
     servo_scan, measure_distance_averaged,
     measure_single_ldr, inc_lcd / dec_lcd / rra_lcd / clear_lcd,
     set_delay, mcu_sleep, FileSystem_Init, Handle_UploadFile_Start, Handle_Upload_Byte,
     Handle_ListFiles_Command, Display_File_List / Display_File_Content,
     run_calibration_sequence, write_all_calib_to_flash, send_calib_arr,
     parse_and_execute_line, get_next_line_from_flash.

3) halGPIO.h
   • HAL prototypes for LCD / UART / ADC / Timers / PWM / Ultrasonic / Flash, circular RX buffer
     (UartBuffer_*), pin macros and ISR vector macros, LPM helpers.

4) bsp_msp430x2xx.h
   • BSP for MSP430G2553: pin mapping, general macros, Timer/ADC/UART defines, ISR vectors.

==============================================================================
[MCU / C Sources] — Purpose + per-function summary
==============================================================================

A) bsp.c — Board Support Package
   Purpose: low-level board bring-up (GPIO/Timers/ADC/UART/LCD) and system helpers.
   Functions:
   - GPIOconfig — Configure pins for LCD, Servo/PWM, Ultrasonic TRIG/ECHO, UART, buttons.
   - Timers_Init — Set up TimerA0/TimerA1 for PWM/Delay/Capture per macros.
   - ADC_config — Initialize ADC10 with default settings.
   - UART_init — Configure UART (9600 bps) and enable RX interrupt.
   - enterLPM / enable_interrupts / disable_interrupts — LPM and global IRQ utilities.

B) halGPIO.c — HAL (device drivers + ISRs)
   Purpose: direct hardware layer—LCD, Flash, Timers, PWM/Servo, Ultrasonic, ADC, UART, Buttons,
            plus the circular RX buffer and all relevant ISRs.
   Functions:

   LCD:
   - lcd_init — Initialize LCD in 4-bit mode.
   - lcd_cmd — Send LCD command nibble(s).
   - lcd_data — Send LCD data/character.
   - lcd_puts — Print a C-string to the LCD.
   - lcd_clear / lcd_strobe — Clear display / EN strobe pulse.

   UART + RX buffer:
   - UART_send_char / UART_send_string_blocking — Transmit a byte / string (polling).
   - UartBuffer_Init / UartBuffer_Put / UartBuffer_Get — Circular RX buffer (filled by RX ISR).

   Timers / delays / ultrasonic:
   - Timer_Start_General_Delay_ms / _sec / Timer_Stop_Delay_ms — Blocking delays with LPM.
   - Timer_Start_Script_Delay_ms — Script-driven delay tick.
   - Ultrasonic_Start_Measurement — Fire TRIG and arm ECHO capture.
   - Ultrasonic_Stop_Measurement — Stop capture, clear flags.
   - Timer_Start_Ultrasonic_Timeout — Safety timeout for a measurement.

   PWM / Servo:
   - pwmOutServoConfig — Compute CCR/Ton for a requested angle.
   - Set_Angle_PWM — Map angle to PWM pulse width.
   - stopServoPwm — Disable PWM output to the servo.

   ADC (LDR):
   - ADC_Enable / ADC_Disable — Power the ADC on/off.
   - ADC_sample — Blocking single-shot ADC read.
   - ADC_Configure_LDR1 / ADC_Configure_LDR2 — Select channel and config per LDR.

   Flash:
   - FlashConfig / Flash_SetTiming — Flash timing vs. clock.
   - Flash_EraseSegment — Erase a Flash segment.
   - Flash_WriteByte / Flash_WriteBlock / Flash_WriteData_NoErase — Program data (content/meta).
   - write_flash_char / init_flash_write / disable_flash_write / Flash_StartWrite / Flash_Close — write helpers.

   ISRs:
   - PBs_handler — Button ISR (debounce, set button flag).
   - Timer_1_ISR — Timer1 capture ISR for ECHO (HC-SR04) + exit LPM.
   - Timer0_A0_ISR_Handler — Timer0 CCR0 ISR for delays/script tick.
   - ADC_ISR — End-of-conversion ISR.

C) api.c — APP/API (FSM logic, filesystem, calibration)
   Purpose: high-level logic, runtime FSMs, Flash filesystem & scripts, LDR calibration, scanning.
   Functions:

   Scans & sensing:
   - servo_scan(start, end, mode, return_to_center)
     Perform a 0°–180° sweep in 1° steps; per step, measure and transmit according to ScanMode.
     Send FF FF FF terminator at the end; optionally return to 90°.
   - object_detector — Objects FSM: move servo, measure HC-SR04, send (2B distance + 1B angle).
   - telemeter — Telemeter FSM: after ‘T’, wait for angle byte, move, take one measurement,
     send (2B distance + 1B angle).
   - light_detector — Light FSM: scan and send (LDR1, LDR2, angle) per step.
   - light_object_detector — Combined FSM: send (distance, LDR1, LDR2, angle) per step.
   - measure_single_ldr — Read a single LDR via ADC.

   LDR calibration:
   - run_calibration_sequence — Collect 20 samples (10 per LDR) guided by LCD/buttons,
     stream live to PC, then compress for storage.
   - write_all_calib_to_flash — Store two 10-point arrays (compressed >>3) into two segments.
   - send_calib_arr — Read back and send 20 bytes from Flash to the PC.

   Filesystem / scripts + LCD:
   - FileSystem_Init — Initialize Flash areas (metadata/content) and internal indices.
   - Handle_UploadFile_Start — On ‘U’: check space and arm upload → reply ‘A’ (ready for metadata).
   - Handle_Upload_Byte — Receive metadata (name/type/size + checksum) → reply ‘B’ (ready for data).
     Then receive file content in 64-byte chunks with checksum; reply ‘K’ per chunk and ‘S’ on final.
     Update metadata and file count.
   - Handle_ListFiles_Command — On ‘L’: send file names (one per line), end with EOT=0x04.
   - Display_File_List / Display_File_Content — Browse names/content on LCD using PB0/PB1.
   - parse_and_execute_line — Script line interpreter (inc_lcd / dec_lcd / rra_lcd / set_delay /
     servo_deg / servo_scan / clear_lcd / sleep, …) and executor.
   - get_next_line_from_flash — Fetch next line from Flash content.
   - mcu_sleep — Enter LPM for waits/delays.

D) main.c — Entry point and command loop
   Purpose: initialize layers, run RX loop, decode UART commands and dispatch FSMs.
   Functions:
   - main — Bring-up BSP/HAL/API/APP; main RX/command loop; state transitions.
   - process_command — Decode commands:
     ‘P’→‘A’ (handshake), ‘L’ (list), ‘U’ (start upload), ‘R’ (run script by name),
     ‘V’ (LCD view mode), ‘D’ (delete / re-init FS), ‘S’ (objects scan),
     ‘K’ (lights scan), ‘X’ (combined scan), ‘J’ (calibration), ‘Z’ (send calibration),
     ‘T’+ angle (interactive telemeter).

==============================================================================
[PC / Python] — main.py (GUI + protocol + processing)
==============================================================================
Purpose: PySimpleGUI desktop app to control all modes, run calibration, manage files,
process data and plot polar maps.

Key functions:
- init_uart / send_command / receive_data / send_data — Open port; I/O helpers.
- calculate_checksum — 8-bit checksum (mod-256) for metadata and 64-byte chunks.
- objects_detector — Send ‘S’; read (dist_lo, dist_hi, angle); segment objects and plot polar map.
- telemeter — Send ‘T’, then angle byte; display live single-point measurements.
- lights_detector — Send ‘K’; read (LDR1, LDR2, angle); smooth, find peaks/valleys; estimate distance.
- light_objects_detector — Combined processing and plotting for lights + objects.
- process_light_scan_data / find_beam_center — smoothing, valley splitting, “wide-blob” merge,
  beam-center estimation.
- expand_calibration_array / get_distance_from_voltage — build continuous calibration map(s),
  convert LDR reading to distance.
- draw_scanner_map / draw_scanner_map_lights / draw_combined_map — polar plotting.
- refresh_file_list — ‘L’: receive names until EOT (0x04).
- upload_file_to_mcu — Full upload protocol:
  1) send ‘U’, wait for ‘A’ (ready for metadata),
  2) send metadata + checksum, wait for ‘B’ (ready for data),
  3) send file in 64-byte chunks (each with checksum); receive ‘K’ per chunk and ‘S’ on final.
- determine_file_type / validate_script_syntax — TEXT vs. SCRIPT detection and basic checks.
- file_mode — GUI for list / view / upload / delete / run / calibrate.
- main — App main loop and mode switching.

Table of opcodes:
inc_lcd=0x01, dec_lcd=0x02, rra_lcd=0x03, set_delay=0x04, clear_lcd=0x05, servo_deg=0x06, servo_scan=0x07, sleep=0x08
==============================================================================
UART protocol — quick reference
==============================================================================
• Control Markers: Start-of-Script = 0xCC, Telemetry Header = 0xFE, End/Sleep = 0x7F, End-of-Scan = FF FF FF
• Handshake: ‘P’ → MCU replies ‘A’.
• Objects:  ‘S’ → stream of (2B distance + 1B angle) per step; end marker FF FF FF.
• Lights:   ‘K’ → stream of (LDR1_lo, LDR1_hi, LDR2_lo, LDR2_hi, angle).
• Combined: ‘X’ → stream of (2B distance, 2B LDR1, 2B LDR2, 1B angle); end FF FF FF.
• Telemeter: ‘T’ then angle byte → MCU returns (2B distance + 1B angle);
• Files:
  - ‘L’ — list files, one name per line; end with EOT=0x04.
  - ‘U’ — upload: ‘A’ (ready for metadata) → send metadata+checksum → ‘B’ (ready for data)
           → send 64-byte chunks with checksum; MCU replies ‘K’ (chunk OK) and ‘S’ (final).
  - ‘R’ + name — run script from Flash.   - ‘V’ — LCD list/view mode.   - ‘D’ — delete (re-init FS).
• Calibration: ‘J’ — guided LDR capture (20 samples); ‘Z’ — read back compressed 20-byte array.
• Endianness - Little-Endian, first Low-Byte, second High-byte.

Authors:
Arkady Gerasimuk
Adi Shlomo

