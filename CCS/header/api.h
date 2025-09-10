

// api.h
#ifndef _api_H_
#define _api_H_

#include  "..\header\halGPIO.h"      // private library - HAL layer
#include  "..\header\app.h"          // private library - APP layer

// --- API Layer Function Prototypes ---
void object_detector(void);
void telemeter(void);
void light_detector(void);
void light_object_detector(void);
void servo_scan(int angle1, int angle2, enum ScanMode mode, int script_mode);
void servo_deg(int);

int measure_distance_averaged(void);
void inc_lcd(int);
void dec_lcd(int);
void rra_lcd(int);
void set_delay(int);
void send_calib_arr(void);
void run_calibration_sequence(void);




// file mode
void file_fsm(void);
extern unsigned int g_file_content_offset;

// --- Global variables declared in api.c
extern enum FSM_object_detector state_object_detector;
extern enum FSM_telemeter state_telemeter;
extern enum FSM_light_sources_detector FSM_light_sources_detector;
extern enum FSM_light_sources_and_objects_detector FSM_light_sources_and_objects_detector;
extern enum FSM_file state_file;

extern volatile int g_button_pressed;


extern unsigned int g_next_free_content_addr; 



extern volatile int g_button_pressed;
extern int g_browse_index;
extern int g_selected_file_index;
extern unsigned int g_file_content_offset;





extern volatile int g_script_name_idx ;
extern char g_script_name_buffer[SCRIPT_NAME_BUFFER_SIZE];
extern volatile int g_stop_script_request;




#endif