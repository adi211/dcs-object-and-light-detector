#include  "..\header\api.h"   
//#include  "..\header\app.h"           // private library - APP layer
#include  "..\header\halGPIO.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>


//==============================================================================
//                      GLOBAL VARIABLE DEFINITIONS
//==============================================================================

// --- System State & FSMs ---
enum FSMstate state;
enum SYSmode lpm_mode;
enum FSM_object_detector state_object_detector;
enum FSM_telemeter state_telemeter;
enum FSM_light_sources_detector FSM_light_sources_detector;
enum FSM_light_sources_and_objects_detector FSM_light_sources_and_objects_detector;
enum FSM_file state_file;

// --- Data Buffers & Application Logic Variables ---

int calibrate_index = 0;

// --- State Flags & Indices for Communication ---
volatile int waiting_for_telemeter_angle = 0;



char tele_angle[5] = {'0','0','0','\n','\0'};
unsigned int tele_angle_int = 0; 


// for file mode
UploadState g_upload_state = UPLOAD_IDLE;
FileEntry g_incoming_file_meta;
unsigned int g_upload_byte_counter = 0;
unsigned int g_current_flash_write_addr = 0; 



unsigned char g_upload_buffer[CHUNK_SIZE+1];
unsigned int g_upload_buffer_index = 0;


volatile int g_button_pressed = 0;
int g_browse_index = 0;
int g_selected_file_index = -1;
unsigned int g_file_content_offset = 0;


static ViewLcdState g_view_lcd_state;



unsigned int g_num_files = 0;
unsigned int g_next_free_content_addr = 0;


char g_script_name_buffer[SCRIPT_NAME_BUFFER_SIZE];
volatile int g_script_name_idx = 0;



static char* g_script_content_ptr = NULL; 
static unsigned int g_script_bytes_left = 0; 

volatile int g_stop_script_request = 0;


static int g_text_file_indices[MAX_FILES]; // Stores the *physical* indices of text files
static int g_num_text_files = 0; 


static volatile int g_nack_counter = 0;

// States Functions 

//------------------------------------------------------------------------------
// 1. Object Detector function
//------------------------------------------------------------------------------

void object_detector(){
    
    switch(state_object_detector){
        case object_detector_sleep:
            lcd_clear();
            lcd_puts("Object Detector");
            lcd_new_line();
            lcd_puts("mode");
            enterLPM(lpm_mode); 
            break;

        case object_detector_scan:
            lcd_clear();
            lcd_puts("Scanning Mode");
            servo_scan(0, 180, SCAN_OBJECTS, 0); 
            state_object_detector = object_detector_sleep;
            break;
    }
}


//------------------------------------------------------------------------------
// 2. Telemeter function
//------------------------------------------------------------------------------

void telemeter() {
	char lcd_buffer[20];
    
    switch (state_telemeter) {
        
        case tele_sleep:
            // Idle state. Wait in LPM until the ISR receives a 'T' command.
            lcd_clear();
            lcd_puts("Telemeter");
            enterLPM(lpm_mode);
            break;

        case tele_action:
            // The servo has already been moved by process_command.
            // time between sampels
            Timer_Start_General_Delay_ms(500);
            enterLPM(lpm_mode);
			Timer_Stop_Delay_ms();
            
            stopServoPwm();
			
			start_ultrasonic_timer();
			// Measure distance
            int distance = measure_distance_averaged();
			sprintf(lcd_buffer, "Dist: %d cm", distance);
			stop_ultrasonic_timer();
			
			lcd_clear();
			lcd_puts(lcd_buffer);
			
			
			//Send binary data
            UART_send_char((char)(distance & 0xFF));
            UART_send_char((char)(distance >> 8));
            UART_send_char((char)tele_angle_int);
			
            break;
    }
}

//------------------------------------------------------------------------------
// 3. Light sources detection
//------------------------------------------------------------------------------




void light_detector() {
    
    switch (FSM_light_sources_detector) {
        
        case light_sleep:
            lcd_clear();
            lcd_puts("Light Detector");
            enterLPM(lpm_mode);
            break;
            
        case light_scan:
            lcd_clear();
            lcd_puts("Scanning Mode");
            
            
            servo_scan(0, 180, SCAN_LIGHT, 0);

            // After the scan is complete, automatically return to sleep state
            FSM_light_sources_detector = light_sleep;
            break;
    }
}


//------------------------------------------------------------------------------
// 4. Light sources and Objects detection function (bonus)
//------------------------------------------------------------------------------

void light_object_detector(){

	switch(FSM_light_sources_and_objects_detector){
	case light_object_sleep:
		lcd_init();
		lcd_puts("Lights and Objects");
		lcd_new_line();
		lcd_puts("Detector");
		enterLPM(lpm_mode);
		break;

	case light_object_scan:
		enable_interrupts();
		lcd_clear();
		lcd_puts("Scanning");
		lcd_new_line();
		lcd_puts("Environment");
		servo_scan(0, 180, SCAN_OBJECTS_AND_LIGHT, 0);
		FSM_light_sources_and_objects_detector = light_object_sleep;
		break;
	}
    
}

//------------------------------------------------------------------------------
// 5. file menu FSM function
//------------------------------------------------------------------------------




void file_fsm(){

    switch(state_file){
    case file_sleep:
		lcd_clear();
		lcd_puts("File Mode");
        enterLPM(lpm_mode);
        break;
		
	case calibrate_sensors: 
		run_calibration_sequence();	
        break;
	
	case send_file_list:
		Handle_ListFiles_Command();
		state_file = file_sleep;
        break;
	
	case upload_file:
		enterLPM(lpm_mode);
        break;
		
	case script_idle:
		
		lcd_clear();
		
		
		state_file = script_wait_for_name;
		break; 

	case script_wait_for_name:
		
		enterLPM(lpm_mode);
		break;
		
		
    case script_start: {
		int i;
		int file_index = -1; 

		FileEntry* files_in_flash = (FileEntry*)FILE_METADATA_ADDR;

		// find file index
		for (i = 0; i < g_num_files; i++) {
			if (strcmp(g_script_name_buffer, files_in_flash[i].name) == 0) {
				file_index = i; 
				break; 
			}
		}

		
		if (file_index != -1) { 
			if (files_in_flash[file_index].type == SCRIPT) {// cheak if it script
				lcd_clear();
				lcd_puts("Starting Script...");
				Timer_Start_General_Delay_sec(2);
				
				g_script_content_ptr = (char*)files_in_flash[file_index].address;
				g_script_bytes_left = files_in_flash[file_index].size;
				UART_send_char(0xCC);
				
				state_file = script_running; 
				
			} else {
				lcd_clear();
				lcd_puts("Not a script!");
				Timer_Start_General_Delay_sec(2);
				state_file = file_sleep;
			}
		} else { 
			lcd_clear();
			lcd_puts("File Not Found!");
			Timer_Start_General_Delay_sec(2);
			state_file = file_sleep;
		}
		break;
	}


    case script_running: {

		// Check for stop request (from UART ISR)
		if (g_stop_script_request) {
			g_stop_script_request = 0; 
			g_script_bytes_left = 0; // This will terminate the loop below

			UartBuffer_Init(); // Clear any pending UART data
			Timers_Init();     // Reset timers to a known state
			lcd_clear();
			lcd_puts("Script Aborted!");
			Timer_Start_General_Delay_sec(2);
			state_file = file_sleep;
			state = state0; // Return to main menu
			break;
		}

		// If there are still bytes left in the script to execute
		if (g_script_bytes_left > 0) {
			unsigned char opcode;
			unsigned char arg1, arg2;

			// 1. Read the Opcode
			opcode = *g_script_content_ptr;
			g_script_content_ptr++; // Move pointer to the next byte (argument or next opcode)
			g_script_bytes_left--;

			// 2. Decode and Execute using a switch statement
			switch (opcode) {
				case 0x01: // inc_lcd
					if (g_script_bytes_left > 0) {
						arg1 = *g_script_content_ptr++;
						g_script_bytes_left--;
						inc_lcd((int)arg1);
					}
					break;

				case 0x02: // dec_lcd
					if (g_script_bytes_left > 0) {
						arg1 = *g_script_content_ptr++;
						g_script_bytes_left--;
						dec_lcd((int)arg1);
					}
					break;

				case 0x03: // rra_lcd
					if (g_script_bytes_left > 0) {
						arg1 = *g_script_content_ptr++;
						g_script_bytes_left--;
						rra_lcd((int)arg1);
					}
					break;

				case 0x04: // set_delay
					if (g_script_bytes_left > 0) {
						arg1 = *g_script_content_ptr++;
						g_script_bytes_left--;
						set_delay((int)arg1);
					}
					break;

				case 0x05: // clear_lcd
					clear_lcd();
					break;

				case 0x06: // servo_deg
					if (g_script_bytes_left > 0) {
						arg1 = *g_script_content_ptr++;
						g_script_bytes_left--;
						servo_deg((int)arg1);
					}
					break;

				case 0x07: // servo_scan
					if (g_script_bytes_left > 1) { // Needs 2 arguments
						arg1 = *g_script_content_ptr++;
						arg2 = *g_script_content_ptr++;
						g_script_bytes_left -= 2;
						script_servo_scan((int)arg1, (int)arg2);
					}
					break;

				case 0x08: // sleep
					mcu_sleep();
					// mcu_sleep will change state, so we just break
					break;

				default:
					// Unknown opcode, stop script to be safe
					g_script_bytes_left = 0;
					lcd_clear();
					lcd_puts("Unknown Opcode!");
					Timer_Start_General_Delay_sec(2);
					break;
			}
			
		} else {
			// Script has finished
			lcd_clear();
			lcd_puts("Script finished.");
			Timer_Start_General_Delay_sec(2);
			UART_send_char(0x7F); // Send script finished signal
			state_file = file_sleep; 
		}

		break; 
	}
	
	
    case view_text_lcd: {
		int i;
		FileEntry* files_in_flash = (FileEntry*)FILE_METADATA_ADDR;

		
		g_view_lcd_state = VIEW_STATE_BROWSING_FILES;
		g_browse_index = 0;      // Reset the browsing index for the filtered list
		g_num_text_files = 0;    // Reset the text file counter

		// Iterate through all stored files to find the text files
		for (i = 0; i < g_num_files; i++) {
			
			if (files_in_flash[i].type == 1 /* TEXT */) {
				// If it is a text file, and we have space in our index array...
				if (g_num_text_files < MAX_FILES) {
					// ...store the PHYSICAL index of this file in our map.
					g_text_file_indices[g_num_text_files] = i;
					g_num_text_files++;
				}
			}
		}

		Display_File_List(); // Show the first file from our new filtered list
		state_file = view_text_lcd_active; // Immediately transition to the active state
		break;
	}



    case view_text_lcd_active:
		enterLPM(lpm_mode);

		if (state_file != view_text_lcd_active) {
			break;
		}

		if (g_view_lcd_state == VIEW_STATE_BROWSING_FILES) {
			if (g_button_pressed == PB0) { // Scroll through filtered list
				g_browse_index++;
				Display_File_List(); 
			} 
			else if (g_button_pressed == PB1) { // Select a file
				// Only proceed if there are text files to select
				if (g_num_text_files > 0) {
					
					// Set the selected index to the PHYSICAL index from our map
					// to ensure Display_File_Content reads the correct file from flash.
					g_selected_file_index = g_text_file_indices[g_browse_index]; 
					
					g_file_content_offset = 0;
					g_view_lcd_state = VIEW_STATE_VIEWING_CONTENT;
					Display_File_Content();
				}
			}
		} 
		else if (g_view_lcd_state == VIEW_STATE_VIEWING_CONTENT) {
			if (g_button_pressed == PB0) { // Scroll logic
				unsigned int file_size = ((FileEntry*)FILE_METADATA_ADDR)[g_selected_file_index].size;
				g_file_content_offset += 32;
				if (g_file_content_offset >= file_size) {
					g_file_content_offset = 0;
				}
				Display_File_Content();

			} else if (g_button_pressed == PB1) { // Go back to browsing
				g_view_lcd_state = VIEW_STATE_BROWSING_FILES;
				Display_File_List();
			}
		}
		// Clear the button press flag
		g_button_pressed = 0;
		break;
        
	
	case delete_file:
		FileSystem_Init();
		state_file = file_sleep;
		

        break;
	}

 }

//------------------------------------------------------------------------------
// calibrate_sensors
//------------------------------------------------------------------------------

// This new function contains the entire calibration logic
void run_calibration_sequence(void){
    char lcd_buffer[20];
    char tx_buffer[6]; // Buffer for sending data to PC
    int distance_cm;
    uint16_t current_sample;
	static uint16_t ldr1_samples[10];
    static uint16_t ldr2_samples[10];
	
	
    // 1. Determine which LDR and distance we are currently calibrating
    if (calibrate_index < 10) {
        // Calibrating LDR1
        distance_cm = (calibrate_index + 1) * 5;
        sprintf(lcd_buffer, "LDR1 at %d cm", distance_cm);
    } else {
        // Calibrating LDR2
        distance_cm = ((calibrate_index % 10) + 1) * 5;
        sprintf(lcd_buffer, "LDR2 at %d cm", distance_cm);
    }

    // 2. Display clear instructions on the LCD
    lcd_clear();
    lcd_puts(lcd_buffer);
    lcd_new_line();
    lcd_puts("Then press PB0");

    // Wait for a button press (ISR will wake the system)
    enterLPM(lpm_mode);

    // 3. Measure the appropriate LDR and store the sample in RAM
    if (calibrate_index < 10) {
        current_sample = measure_single_ldr(1);
        ldr1_samples[calibrate_index] = current_sample;
    } else {
        current_sample = measure_single_ldr(2);
        // Store in LDR2's array at index 0-9
        ldr2_samples[calibrate_index - 10] = current_sample;
    }

    // 4. Send the measured value to the PC for real-time feedback
    sprintf(tx_buffer, "%d\n", current_sample);
    UART_send_string_blocking(tx_buffer);

    // 5. Increment index and check if calibration is complete
    calibrate_index++;

    if (calibrate_index >= 20) {
        // All 20 samples have been collected
        lcd_clear();
        lcd_puts("Saving Data...");
        
        // Call the function to write both arrays to Flash (to be created in the next step)
        write_all_calib_to_flash(ldr1_samples, ldr2_samples);

        lcd_clear();
        lcd_puts("Calibration Done!");
        DelayMs(1000);

        // Reset index for the next calibration run
        calibrate_index = 0;
		state_file = file_sleep;
        
    }
}



/*
 * Writes the collected LDR calibration samples to their designated segments in Flash memory.
 * This function should only be called after all 20 samples have been collected into RAM.
 */

void write_all_calib_to_flash(uint16_t* ldr1_data, uint16_t* ldr2_data) {
    // 1. Prepare compressed data in a RAM buffer
    char compressed_ldr1[10];
    char compressed_ldr2[10];
    int i;

    for (i = 0; i < 10; i++) {
        compressed_ldr1[i] = (char)(ldr1_data[i] >> 3);
        compressed_ldr2[i] = (char)(ldr2_data[i] >> 3);
    }

    // 2. Write the entire block to Flash in one clean operation
    Flash_WriteBlock(compressed_ldr1, LDR1_CALIB_START_ADDR, 10);
    
    // 3. Write the second block
    Flash_WriteBlock(compressed_ldr2, LDR2_CALIB_START_ADDR, 10);
}



/*
 * Reads the calibration data for BOTH LDRs from their respective Flash segments
 * and sends all 20 bytes sequentially over UART to the PC.
 */
void send_calib_arr(void){
    int i;

    // --- Send LDR1 Calibration Data (from Segment C) ---
    char *read_ptr = (char *)LDR1_CALIB_START_ADDR;
    for (i = 0; i < 10; i++) {
        UART_send_char(*read_ptr++);
        DelayMs(5); // Small delay for stable transmission
    }

    // --- Send LDR2 Calibration Data (from Segment B) ---
    read_ptr = (char *)LDR2_CALIB_START_ADDR;
    for (i = 0; i < 10; i++) {
        UART_send_char(*read_ptr++);
        DelayMs(5); // Small delay for stable transmission
    }
}

//------------------------------------------------------------------------------
// Init_file_system
//------------------------------------------------------------------------------



void FileSystem_Init(void) {
    int i;

    // This function performs a "clean slate" erase on the entire file system area.
    disable_interrupts();
    FlashConfig(); // Configure flash timing before any operation
    
    // 1. Erase the single metadata segment
    Flash_EraseSegment(FILE_METADATA_ADDR);

    // 2. Erase the four content segments in a loop
    for (i = 0; i < 4; i++) {
        Flash_EraseSegment(FILE_CONTENT_START_ADDR + (i * 512));
    }
    
    enable_interrupts();
	
	g_num_files = 0;
    g_next_free_content_addr = FILE_CONTENT_START_ADDR;
}






// This function acts as a "gatekeeper" before the upload starts.
void Handle_UploadFile_Start(void) {
    int i;
    int files_count = 0;
    
    // Check if the file slots are full
    if (g_num_files >= MAX_FILES) {
        UART_send_char('N'); // NACK: File limit reached
        return;
    }

    // If we have a free slot, proceed
    g_upload_state = UPLOAD_WAITING_FOR_META;
    g_upload_byte_counter = 0;
    UART_send_char('A'); // ACK: Ready for metadata
	g_nack_counter = 0;
}




void Handle_Upload_Byte(unsigned char byte) {

    switch (g_upload_state) {
        case UPLOAD_WAITING_FOR_META:
            g_upload_buffer[g_upload_buffer_index++] = byte;
            if (g_upload_buffer_index >= 21) {
                if (is_checksum_valid(g_upload_buffer, 21)) {
                    // Checksum OK, process metadata
                    memcpy(g_incoming_file_meta.name, g_upload_buffer, 17);
                    g_incoming_file_meta.type = (FileType)g_upload_buffer[17];
                    g_incoming_file_meta.size = (g_upload_buffer[18] & 0xFF) | ((unsigned int)(g_upload_buffer[19] & 0xFF) << 8);

                    if (((unsigned long)g_next_free_content_addr + g_incoming_file_meta.size) > FILE_CONTENT_END_ADDR) {
                        UART_send_char('F');
                        g_upload_state = UPLOAD_IDLE;
						state_file = file_sleep;
                    } else {
                        UART_send_char('B');
                        g_current_flash_write_addr = g_next_free_content_addr;
                        g_upload_state = UPLOAD_WAITING_FOR_CONTENT;
                        g_upload_buffer_index = 0;
                        g_upload_byte_counter = 0; // Reset total counter for the new file
						g_nack_counter = 0;
                    }
                } else {
                    UART_send_char('N');
                    g_upload_buffer_index = 0;
					g_upload_state = UPLOAD_IDLE;
					state_file = file_sleep;
                }
            }
            break;

        case UPLOAD_WAITING_FOR_CONTENT:
            g_upload_buffer[g_upload_buffer_index++] = byte;

            unsigned int current_chunk_data_size;
            if ((g_incoming_file_meta.size - g_upload_byte_counter) < CHUNK_SIZE) {
                current_chunk_data_size = g_incoming_file_meta.size - g_upload_byte_counter;
            } else {
                current_chunk_data_size = CHUNK_SIZE;
            }

            if (g_upload_buffer_index >= (current_chunk_data_size + 1)) {
                if (is_checksum_valid(g_upload_buffer, current_chunk_data_size + 1)) {
                    Flash_WriteData_NoErase(g_upload_buffer, g_current_flash_write_addr, current_chunk_data_size);
                    g_current_flash_write_addr += current_chunk_data_size;
                    g_nack_counter = 0;
                    
                    // Update the total byte counter AFTER a successful write
                    g_upload_byte_counter += current_chunk_data_size;
                    
                    g_upload_buffer_index = 0;

                    if (g_upload_byte_counter >= g_incoming_file_meta.size) {
                        // Finalization logic
                        int empty_slot_index = g_num_files;
                        if (empty_slot_index < MAX_FILES) {
                            FileEntry new_entry;
                            memset(&new_entry, 0, sizeof(FileEntry));
                            new_entry.type = g_incoming_file_meta.type;
                            new_entry.size = g_incoming_file_meta.size;
                            new_entry.address = g_next_free_content_addr;
                            strncpy(new_entry.name, g_incoming_file_meta.name, 16);
                            new_entry.name[16] = '\0';
                            unsigned int slot_address = FILE_METADATA_ADDR + (empty_slot_index * FILE_ENTRY_SIZE);
                            Flash_WriteData_NoErase(&new_entry, slot_address, FILE_ENTRY_SIZE);
                            g_num_files++;
                            g_next_free_content_addr += g_incoming_file_meta.size;
                        }
                        UART_send_char('S');
                        g_upload_state = UPLOAD_IDLE;
                    } else {
                        UART_send_char('K');
                    }
                } else {
                    // Checksum failed, request retry
                     g_nack_counter++;
					 if (g_nack_counter >= 3) {
						// אם הגענו ל-3 כישלונות רצופים, בטל את ההעלאה
						g_upload_state = UPLOAD_IDLE;
						state_file = file_sleep; // חזור למצב המתנה
						g_nack_counter = 0;
						
					} else {
						// אם עוד לא הגענו ל-3, בקש ניסיון חוזר
						UART_send_char('N');
						g_upload_buffer_index = 0;
					}
				}
            }
            break;

        default:
            g_upload_state = UPLOAD_IDLE;
            break;
    }
}


//------------------------------------------------------------------------------
// upload_file
//------------------------------------------------------------------------------


// In api.c

void Handle_ListFiles_Command(void) {
    int i;
    FileEntry* files_in_flash = (FileEntry*)FILE_METADATA_ADDR;

    // --- FIX: Loop only up to the number of valid files ---
    for (i = 0; i < g_num_files; i++) {
        UART_send_string_blocking(files_in_flash[i].name);
        UART_send_char('\n');
    }
    UART_send_char(0x04);
}


//------------------------------------------------------------------------------
// view_text_lcd
//------------------------------------------------------------------------------





void Display_File_List(void) {
    FileEntry* files_in_flash = (FileEntry*)FILE_METADATA_ADDR;
    char lcd_buffer[20];

    lcd_clear();
    // Check our counter for TEXT files, not the global file counter
    if (g_num_text_files == 0) {
        lcd_puts("No Text Files");
        return;
    }

    // Cyclic logic for the BROWSE index, now based on the number of TEXT files found
    if (g_browse_index >= g_num_text_files) {
        g_browse_index = 0;
    }
    
    // Get the PHYSICAL file index from our temporary index array
    int physical_index = g_text_file_indices[g_browse_index];

    // Print the file name on the LCD using the correct physical index
    sprintf(lcd_buffer, "> %s", files_in_flash[physical_index].name);
    lcd_puts(lcd_buffer);
}


void Display_File_Content(void) {
    FileEntry* files_in_flash = (FileEntry*)FILE_METADATA_ADDR;

    unsigned char content_buffer[33];
    unsigned char* read_ptr;
    unsigned int file_size;
    unsigned int bytes_to_read;
    int i, line_char_count = 0;

    if (g_selected_file_index < 0) return;

    
    file_size = files_in_flash[g_selected_file_index].size;
    memset(content_buffer, 0, 33); 

    if (g_file_content_offset < file_size) {
        unsigned int remaining_bytes = file_size - g_file_content_offset;
        bytes_to_read = (remaining_bytes < 32) ? remaining_bytes : 32;
    } else {
        bytes_to_read = 0;
    }

    if (bytes_to_read > 0) {
        read_ptr = (unsigned char*)(files_in_flash[g_selected_file_index].address + g_file_content_offset);
        for (i = 0; i < bytes_to_read; i++) {
            content_buffer[i] = read_ptr[i];
        }
    }
    
    
    lcd_clear();
    line_char_count = 0; 

    for (i = 0; i < 32; i++) {
        char current_char = content_buffer[i];
        if (current_char == '\0') break;

        // replace \n with space
        if (current_char == '\n' || current_char == '\r') {
            current_char = ' '; 
        }

        // end of line
        if (line_char_count == 16) {
            lcd_new_line();
            line_char_count = 0;
        }

        
        if (current_char >= 32) {
            lcd_data(current_char);
        } else {
            // non readable character replace with space
            lcd_data(' ');
        }
        line_char_count++; 
    }
}


//------------------------------------------------------------------------------
//                              Scripts
//------------------------------------------------------------------------------


void parse_and_execute_line(char* line) {
    char* command;
    char* arg1_str;
    char* arg2_str;
    int arg1_val, arg2_val;


    command = strtok(line, " ,");
    if (command == NULL) return; 

    
    arg1_str = strtok(NULL, " ,");
    arg2_str = strtok(NULL, " ,");

    
    if (strcmp(command, "inc_lcd") == 0) {
        if (arg1_str) { 
            arg1_val = atoi(arg1_str);
            inc_lcd(arg1_val);
        }
    } else if (strcmp(command, "dec_lcd") == 0) {
        if (arg1_str) {
            arg1_val = atoi(arg1_str);
            dec_lcd(arg1_val);
        }
    } else if (strcmp(command, "rra_lcd") == 0) {
        if (arg1_str) {
            arg1_val = atoi(arg1_str);
            rra_lcd(arg1_val);
        }
    } else if (strcmp(command, "set_delay") == 0) {
        if (arg1_str) {
            arg1_val = atoi(arg1_str);
            set_delay(arg1_val);
        }
    } else if (strcmp(command, "clear_lcd") == 0) {
        clear_lcd(); 
    } else if (strcmp(command, "servo_deg") == 0) {
        if (arg1_str) {
            arg1_val = atoi(arg1_str);
            servo_deg(arg1_val);
        }
    } else if (strcmp(command, "servo_scan") == 0) {
        if (arg1_str && arg2_str) { 
            arg1_val = atoi(arg1_str);
            arg2_val = atoi(arg2_str);
            script_servo_scan(arg1_val, arg2_val);
        }
    } else if (strcmp(command, "sleep") == 0) {
		mcu_sleep();
        
    }
}

// Helper function to read the next line from flash memory into a buffer
int get_next_line_from_flash(char* buffer, unsigned int max_len, char** flash_ptr, unsigned int* bytes_remaining) {
    unsigned int count = 0;
    while (*bytes_remaining > 0 && count < (max_len - 1)) {
        char c = **flash_ptr;
        (*flash_ptr)++;
        (*bytes_remaining)--;

        if (c == '\n' || c == '\r') {
            break; // End of line
        }
        buffer[count++] = c;
    }
    buffer[count] = '\0'; // Null-terminate the string
    return count; // Return number of characters read
}




void inc_lcd(int x) {
    int upcounter = 0;
    char upcounter_str[10];

    

    while (upcounter <= x) {
		lcd_clear();
        sprintf(upcounter_str, "%d", upcounter);
        lcd_home();
        lcd_puts(upcounter_str);
        upcounter++;
		Timer_Start_Script_Delay_ms(g_script_delay_period_ms);
		enterLPM(lpm_mode);
		Timer_Stop_Delay_ms();
    }
    
    Timer_Start_General_Delay_sec(2);
	lcd_clear();
}



void dec_lcd(int x) {
    int downcounter = x;
    char downcounter_str[10];

    lcd_clear();

    while (downcounter >= 0) {
		lcd_clear();
        
        sprintf(downcounter_str, "%d", downcounter);
        lcd_home();
        lcd_puts(downcounter_str);
        downcounter--;
		Timer_Start_Script_Delay_ms(g_script_delay_period_ms);
		enterLPM(lpm_mode);
		Timer_Stop_Delay_ms();
	
    }

    Timer_Start_General_Delay_sec(2);
	lcd_clear();
}


//------------------------------------------------------------------------------
// Rotates a character across both lines of the LCD.
//------------------------------------------------------------------------------

void rra_lcd(int x) {
    char char_to_rotate = (char)x; 
    int i;

    lcd_clear();    
    
	
    for (i = 0; i < 32; i++) {
        
        
        if (i > 0) {
            int prev_pos = i - 1;
            if (prev_pos < 16) {
                
                lcd_goto(prev_pos); 
            } else {
                
                lcd_cmd(0xC0 + (prev_pos - 16));
            }
            lcd_putchar(' '); 
        }

        
        if (i < 16) {
            
            lcd_goto(i);
        } else {
            
            lcd_cmd(0xC0 + (i - 16));
        }
        lcd_putchar(char_to_rotate); 

        
        Timer_Start_Script_Delay_ms(g_script_delay_period_ms);
        enterLPM(lpm_mode);
        Timer_Stop_Delay_ms(); 
    }

    
    Timer_Start_General_Delay_sec(2);

    lcd_clear();

}


//------------------------------------------------------------------------------
// Point the ultrasonic sensor to a specific angle, measure distance,
// and send the results back to the PC. (Called from scripts)
//------------------------------------------------------------------------------
void servo_deg(int angle) {
    char lcd_buffer[20];
    int distance;

    // --- 1. Move Servo ---
    // Move the servo to the requested angle.
    pwmOutServoConfig(ANGLE_TO_PWM(angle));

    // --- 2. Wait for Movement ---
    // Give the servo time to physically reach the position.
    Timer_Start_General_Delay_ms(200); // A short delay for the servo to settle
    enterLPM(lpm_mode);
    Timer_Stop_Delay_ms();
    stopServoPwm(); // Stop the PWM to prevent servo jitter

    // --- 3. Measure Distance ---
    // Perform an averaged distance measurement.
	start_ultrasonic_timer();
    distance = measure_distance_averaged();
	stop_ultrasonic_timer();

	// --- 4. Send Data to PC (Binary Protocol with Header) ---
	UART_send_char(0xFE);                        
	UART_send_char((char)(distance & 0xFF));     // Send distance low byte
	UART_send_char((char)(distance >> 8));      // Send distance high byte
	UART_send_char((char)angle);                // Send the angle

    // --- 5. Update LCD ---
    // Provide feedback on the device's screen.
    lcd_clear();
    sprintf(lcd_buffer, "Ang:%d Dist:%d", angle, distance);
    lcd_puts(lcd_buffer);
	
	Timer_Start_General_Delay_sec(2);
	lcd_clear();
}


void script_servo_scan(int left_angle, int right_angle) {
    servo_scan(left_angle, right_angle, SCAN_OBJECTS, 1);
	Timer_Start_General_Delay_sec(2);
}

void clear_lcd(void) {
    lcd_clear();
}

void set_delay(int delay) {
    g_script_delay_period_ms = delay * 10;
}

void mcu_sleep(void) {
    lcd_clear();
    lcd_puts("MCU sleep");
    Timer_Start_General_Delay_sec(2);
	UART_send_char(0x7F);
	state_file = file_sleep;
}


//------------------------------------------------------------------------------
// Helper Functions for states 1 to 4
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Scans the environment with the servo from a start angle to an end angle.
// Measures distance and/or light based on the scan_mode.
//------------------------------------------------------------------------------

void servo_scan(int angle1, int angle2, enum ScanMode mode, int script_mode) {
    
    const int SCAN_STEP_ANGLE = 1;
    int current_angle;
	
    
	pwmOutServoConfig(ANGLE_TO_PWM(angle1));
	Timer_Start_General_Delay_ms(250); // Give the servo time to reach the first angle
    enterLPM(lpm_mode);
    Timer_Stop_Delay_ms();
	
	
	for (current_angle = angle1; current_angle <= angle2; current_angle += SCAN_STEP_ANGLE) {
		
		
        // Calculate PWM for the rellevant angle.
        Set_Angle_PWM(ANGLE_TO_PWM(current_angle));
        // Wait for the servo to physically reach the position.
		
		
        Timer_Start_General_Delay_ms(50); 
		enterLPM(lpm_mode);
		Timer_Stop_Delay_ms();
		

		
        // MEASURE by state
        
        switch (mode) {
            case SCAN_OBJECTS: {
                int distance = measure_distance_averaged();
                if (script_mode) {
					UART_send_char(0xFE);
				}
				UART_send_char((char)(distance & 0xFF));      
                UART_send_char((char)(distance >> 8));     
                UART_send_char((char)current_angle);
                break;
            }

            case SCAN_LIGHT: {
				
				Timer_Start_General_Delay_ms(50); 
				enterLPM(lpm_mode);
				Timer_Stop_Delay_ms();
				
				int ldr1_val = measure_single_ldr(1);
				Timer_Start_General_Delay_ms(30);
				enterLPM(lpm_mode);
				Timer_Stop_Delay_ms();
				int ldr2_val = measure_single_ldr(2);


				UART_send_char((char)(ldr1_val & 0xFF));     
				UART_send_char((char)(ldr1_val >> 8));
				
				// LDR2 value (2 bytes, little-endian)
				UART_send_char((char)(ldr2_val & 0xFF));
				UART_send_char((char)(ldr2_val >> 8));
				// Angle value (1 byte)
				UART_send_char((char)current_angle);
                break;
            }

            case SCAN_OBJECTS_AND_LIGHT: {

                int distance = measure_distance_averaged();
				int ldr1_val = measure_single_ldr(1);
				Timer_Start_General_Delay_ms(30);
				enterLPM(lpm_mode);
				Timer_Stop_Delay_ms();
				int ldr2_val = measure_single_ldr(2);
				
				// Distance value (2 bytes, little-endian)
				UART_send_char((char)(distance & 0xFF));
				UART_send_char((char)(distance >> 8));
				// LDR1 value (2 bytes, little-endian)
				UART_send_char((char)(ldr1_val & 0xFF));
				UART_send_char((char)(ldr1_val >> 8));
				// LDR2 value (2 bytes, little-endian)
				UART_send_char((char)(ldr2_val & 0xFF));
				UART_send_char((char)(ldr2_val >> 8));
				// Angle value (1 byte)
				UART_send_char((char)current_angle);
                break;
            }
        }
    }

    // --- End of Scan Procedure ---
    UART_send_char(0xFF);
    UART_send_char(0xFF);
    UART_send_char(0xFF); // Dummy angle byte
	

    // Return to center and stop the motor completely.
    if (!script_mode) {
        pwmOutServoConfig(ANGLE_TO_PWM(90));
        Timer_Start_General_Delay_ms(250); // Using the longer, safer delay.
		enterLPM(lpm_mode);
		Timer_Stop_Delay_ms();
        stopServoPwm(); 
    }
	else {
		stopServoPwm();
	}
}

//------------------------------------------------------------------------------
// Measure distance [0-500 cm] using ultrasonic sensor
//------------------------------------------------------------------------------
int measure_distance_averaged(void) {
    long distance_sum = 0;
    int samples_to_average = 3;
	int successful_samples = 0;
    int i;
    
    for (i = 0; i < samples_to_average; i++) {
        
        Timer_Start_General_Delay_ms(30); 
		enterLPM(lpm_mode);
		Timer_Stop_Delay_ms();
		
		Ultrasonic_Start_Measurement();
        enterLPM(lpm_mode); 
        Ultrasonic_Stop_Measurement();
        
		 /*******************************************************************************
		 * @brief Optimized fixed-point calculation to convert pulse time to distance.
		 *
		 * @details This function converts the ultrasonic echo pulse duration (in microseconds)
		 * to distance (in centimeters) using fast integer arithmetic. This avoids
		 * slow floating-point division (e.g., `time_us / 58.0`), which is very
		 * inefficient on an MCU without a dedicated Floating-Point Unit (FPU).
		 *
		 * --- 1. THEORETICAL DERIVATION (from Speed of Sound) ---
		 *
		 * - The basic physics formula is: Distance = Speed × Time.
		 * - The measured time (`time_us`) is for a round trip (sensor -> object -> sensor),
		 * so the actual distance is covered in half this time.
		 * => Distance = Speed × (time_us / 2)
		 *
		 * - [cite_start]The speed of sound at room temperature (25°C) is ~34,645 cm/s[cite: 191].
		 * - Our time is in microseconds (µs), so we convert the speed's units:
		 * Speed = 34,645 cm/s / 1,000,000 µs/s = 0.034645 cm/µs.
		 *
		 * - Combining these gives the floating-point formula:
		 * Distance_cm = (time_us / 2) * 0.034645
		 * Distance_cm = time_us * 0.0173225
		 *
		 * - To express this using division:
		 * Distance_cm = time_us / (1 / 0.0173225)
		 * Distance_cm ≈ time_us / 57.7
		 *
		 * This is the origin of divide by 58 approximation.
		 *
		 * --- 2. IMPLEMENTATION (Q16 Fixed-Point Math) ---
		 *
		 * - To implement the formula `distance = time_us / 58` without using floats,
		 * we use Q16 fixed-point arithmetic. We represent the factor (1/58)
		 * as a 16-bit integer fraction: FACTOR / 65536.
		 *
		 * - Calculate the integer FACTOR:
		 * FACTOR = (1 / 58.0) × 65536 = 1129.93... ≈ 1130.
		 *
		 * - The final operation becomes `(time_us * 1130) / 65536`. Division by 65536
		 * is computationally cheap, as it's just a 16-bit right shift (`>> 16`).
		 *
		 * - The cast to `(unsigned long)` is critical to ensure the intermediate
		 * multiplication `(time_us * 1130)` does not overflow a 16-bit integer,
		 * which would produce a completely wrong result.
		 *
		 ******************************************************************************/
	 
        if (echo_pulse_time_us > 0 && echo_pulse_time_us < 30000) {
            
            distance_sum += ((unsigned long)echo_pulse_time_us * 1130) >> 16;
            successful_samples++;
        }
        
    }
    
     // Calculate the average only based on successful measurements
    if (successful_samples > 0) {
        return distance_sum / successful_samples;
    } else {
        return 0; // Return 0 if all measurements failed
    }
}



int measure_single_ldr(int ldr_index) {
     

    if (ldr_index == 1) {
        ADC_Configure_LDR1();
    } else { 
        ADC_Configure_LDR2();
    }
	__delay_cycles(5000);

    ADC_Enable();
	enterLPM(lpm_mode); 
    ADC_Disable();

    return ADC_sample();
}



unsigned int ANGLE_TO_PWM(int angle) {
    

    long pwm_val; //למניעת גלישה בחישובים

    if (angle <= 90) {
        // נוסחה לינארית מס' 1: עעבור הטווח 0-90 מעלות
        pwm_val = PWM_FOR_CMD_0 + ((long)angle * (PWM_FOR_CMD_90 - PWM_FOR_CMD_0)) / 90;
    } else {
        // נוסחה לינארית מס' 2: עובור הטווח 90-180 מעלות
        pwm_val = PWM_FOR_CMD_90 + ((long)(angle - 90) * (PWM_FOR_CMD_180 - PWM_FOR_CMD_90)) / 90;
    }

    return (unsigned int)pwm_val;

}



//------------------------------------------------------------------------------
// Checksum Function for upload file system
//------------------------------------------------------------------------------

bool is_checksum_valid(const unsigned char* buffer, int length) {
    unsigned int sum = 0;
    int i;
    for (i = 0; i < length; i++) {
        sum += buffer[i];
    }
    return (sum & 0xFF) == 0;
}



