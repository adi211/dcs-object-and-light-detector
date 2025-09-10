#ifndef APP_H_
#define APP_H_

#include <stdint.h> 

//==============================================================================
//                      1. CORE DEFINITIONS & CONSTANTS
//==============================================================================

#define MAX_FILES 10
#define CHUNK_SIZE 64
#define FILE_ENTRY_SIZE 32
#define RX_BUFFER_SIZE 65


// --- Memory Map ---
#define FILE_METADATA_ADDR          0xF400
#define FILE_CONTENT_START_ADDR     0xF600
#define FILE_CONTENT_END_ADDR       0xFDFF
#define LDR1_CALIB_START_ADDR       0x1040
#define LDR2_CALIB_START_ADDR       0x1080

// --- Servo PWM Calculation Constants ---
#define PWM_FOR_CMD_0   490
#define PWM_FOR_CMD_90  1360
#define PWM_FOR_CMD_180 2370
//#define ANGLE_TO_PWM(angle) (SERVO_PWM_MIN + (((long)angle * (SERVO_PWM_MAX - SERVO_PWM_MIN)) / 180))


#define LINE_BUFFER_SIZE 32 
#define SCRIPT_NAME_BUFFER_SIZE 17

//==============================================================================
//                        FSM STATES ENUMERATIONS
//==============================================================================

enum FSMstate {state0, state1, state2, state3, state4, state5};
enum SYSmode {mode0, mode1, mode2, mode3, mode4};

enum FSM_object_detector {object_detector_sleep, object_detector_scan};
enum FSM_telemeter {tele_sleep, tele_action};
enum FSM_light_sources_detector {light_sleep, light_scan};
enum FSM_light_sources_and_objects_detector {light_object_sleep, light_object_scan};
enum FSM_file {file_sleep, send_file_list, upload_file, view_text_lcd,view_text_lcd_active, delete_file,
     script_idle,script_wait_for_name, script_start, script_running, calibrate_sensors};

enum ScanMode {SCAN_OBJECTS, SCAN_LIGHT, SCAN_OBJECTS_AND_LIGHT};


// --- System State ---
extern enum FSMstate state;
extern enum SYSmode lpm_mode;

// --- Sub-FSM States (defined in api.c) ---
extern enum FSM_object_detector state_object_detector;
extern enum FSM_telemeter state_telemeter;
extern enum FSM_light_sources_detector FSM_light_sources_detector;
extern enum FSM_light_sources_and_objects_detector FSM_light_sources_and_objects_detector;
extern enum FSM_file state_file;


//==============================================================================
//                      File STRUCTURES
//==============================================================================


typedef enum {
    TEXT = 1,
    SCRIPT = 2
} FileType;
typedef enum {UPLOAD_IDLE, UPLOAD_WAITING_FOR_META, UPLOAD_WAITING_FOR_CONTENT} UploadState;
typedef enum {VIEW_STATE_BROWSING_FILES, VIEW_STATE_VIEWING_CONTENT} ViewLcdState;




typedef struct {
    char name[17];
    FileType type;
    unsigned int size;
    unsigned int address;
    char padding[8];
} FileEntry;

//==============================================================================
//                      GLOBAL VARIABLE DECLARATIONS 
//==============================================================================

//for file mode
extern unsigned int g_num_files;
extern unsigned int g_next_free_content_addr;
extern UploadState g_upload_state;
extern FileEntry g_incoming_file_meta;
extern unsigned int g_upload_byte_counter;
extern unsigned  char g_upload_buffer[CHUNK_SIZE+1];
extern unsigned int g_upload_buffer_index;
extern unsigned int g_current_flash_write_addr;
extern volatile int g_button_pressed;


extern volatile int g_button_pressed;
extern int g_browse_index;
extern int g_selected_file_index;
extern unsigned int g_file_content_offset;



//for telemeter mode
extern unsigned int tele_angle_int ;
extern volatile int waiting_for_telemeter_angle;

#endif /* APP_H_ */