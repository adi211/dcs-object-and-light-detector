
#include "..\header\api.h"


void main(void){
  state = state0;  
  lpm_mode = mode0;     
  sysConfig();
  UartBuffer_Init();
  FileSystem_Init();
  
  
  while(1){
	unsigned char command_char;
	
      while (UartBuffer_Get(&command_char) == 0) {
          process_command(command_char);
      }
    
	
	
	switch(state){
	
	case state0:
		lcd_clear();
	    lcd_puts("Main Menu");
        enterLPM(lpm_mode);
    break;

	case state1: 
        object_detector();
	break;
		 
	case state2:    
	    telemeter();
	break;
                
    case state3:    // Light sources detection
        light_detector();
    break;

    case state4:   
        light_object_detector();
    break;

    case state5:    // Script
        file_fsm();
    break;

	}
  }
}


void process_command(unsigned char cmd) {
	
	if (state_file == script_running) {
        if (cmd == 'Q') {
            g_stop_script_request = 1;
        }
        return; 
    }
	
	if (g_upload_state != UPLOAD_IDLE) {
        Handle_Upload_Byte(cmd);
        return; 
    }
	
    if (state_file == script_wait_for_name) {
        
        
        // If the character is a newline, the name is complete.
        if (cmd == '\n' || cmd == '\r') {
            g_script_name_buffer[g_script_name_idx] = '\0'; // Finalize the string
			g_script_name_idx = 0;
            
            // Now that we have the full name, move to the start state to process it.
            state_file = script_start; 

        } 
        // Otherwise, if there is space, add the character to our name buffer.
        else if (g_script_name_idx < (SCRIPT_NAME_BUFFER_SIZE - 1)) {
            g_script_name_buffer[g_script_name_idx++] = cmd;
        }
        
        return; // Done processing this character.
    }

	
    if (waiting_for_telemeter_angle) {
        tele_angle_int = (int)cmd; // The byte we received IS the angle
        
        // Move the servo immediately upon receiving the angle
        pwmOutServoConfig(ANGLE_TO_PWM(tele_angle_int));
        
        waiting_for_telemeter_angle = 0; // Reset the flag
        return; // We are done 
    }


    
    // --------- For receiving single-character commands ---------
    switch(cmd){ 
        case 'Q':
            state = state0; 
            break;
        
        case 'P':
            UART_send_char('A');
            break;

        case '0':
            state = state0;
            break;

        case '1':
            state = state1;
            break;

        case '2':
            state = state2;
            break;

        case '3':
            state = state3;
            break;

        case '4':
            state = state4;
            break;

        case '5':
            state = state5;
            break;

		case 'L': // List Files
            state_file = send_file_list;
            break;
            
        case 'U': // Upload File
			state_file = upload_file;
			Handle_UploadFile_Start();
            break;

        case 'R': // Play Script
            // The next byte received will be the file name
            state_file = script_idle;
            break;
			
		case 'V': // Play Script
            
            state_file = view_text_lcd;
            break;
            
        case 'D': // Delete All Files
            
            state_file = delete_file;
            break;

        // FSM state change commands
        case 'T':
            state_telemeter = tele_action;
            waiting_for_telemeter_angle = 1; 
            break;
        case 'M':
            state_telemeter = tele_sleep;
            break;
        case 'S':
            state_object_detector = object_detector_scan;
            break;
        case 'J':
            state_file = calibrate_sensors;
            break;
        case 'K':
            FSM_light_sources_detector = light_scan;
            break;
        case 'X':
            FSM_light_sources_and_objects_detector = light_object_scan;
            break;
        case 'Z':
            send_calib_arr();
            break;


        default:
            
            break;
    }
}
  
  
  
  
  
  
