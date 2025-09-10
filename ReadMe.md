# Light Source & Object Proximity Detector System
**Final Project - DCS Course**

## 👥 Authors
- Arkady Gerasimuk
- Adi Shlomo

## 📋 Project Overview
An embedded system for detecting light sources and measuring object proximity using MSP430G2553 microcontroller with multiple sensors and actuators.

## 🔧 Hardware Components

### Main Components
- **Microcontroller**: MSP430G2553
- **Display**: LCD 16×2
- **Sensors**: 
  - HC-SR04 Ultrasonic sensor
  - 2× LDR (Light Dependent Resistors)
- **Motor**: SG90 Servo
- **Communication**: UART @ 9600 bps

### System Architecture
```
BSP → HAL → API/APP → Main (FSM, interrupt-driven)
```

## 📌 Pin Configuration

| Pin  | Function | Description |
|------|----------|-------------|
| **P1.0** | Analog Input (A0) | LDR1 |
| **P1.1** | UART | Communication |
| **P1.2** | UART | Communication |
| **P1.3** | Analog Input (A3) | LDR2 |
| **P1.4** | LCD Data D4 | Display data |
| **P1.5** | LCD Data D5 | Display data |
| **P1.6** | LCD Data D6 | Display data |
| **P1.7** | LCD Data D7 | Display data |
| **P2.0** | PB0 | LDR calibration button |
| **P2.1** | LCD Control E | Enable signal |
| **P2.2** | Ultrasonic ECHO | Timer A1 CCR1 |
| **P2.3** | LCD Control RS | Register select |
| **P2.4** | PWM Output | Timer A1 CCR2 - Servo motor |
| **P2.5** | LCD Control RW | Read/Write |
| **P2.6** | Ultrasonic TRIG | Timer A0 CCR1 |
| **P2.7** | PB1 | Button input |

## 📁 Source Code Structure

### MCU Code (C)

#### Headers
- **`app.h`** - Application layer definitions
  - Memory map constants and addresses
  - FSM state enumerations
  - File system structures
  - Shared state declarations

- **`api.h`** - API function prototypes
  - Sensor control functions
  - Display management
  - File system operations
  - Calibration routines

- **`halGPIO.h`** - Hardware abstraction layer
  - GPIO operations
  - UART circular buffer
  - Timer/PWM control
  - Flash memory access

- **`bsp_msp430x2xx.h`** - Board support package
  - MSP430-specific definitions
  - Register mappings
  - Interrupt vectors

#### Source Files

##### `bsp.c` - Board Support Package
Low-level hardware initialization and configuration
- `GPIOconfig()` - Configure all GPIO pins
- `Timers_Init()` - Setup TimerA0/A1 for PWM/delays
- `ADC_config()` - Initialize ADC10
- `UART_init()` - Configure UART @ 9600 bps
- `enterLPM()` - Low power mode control

##### `halGPIO.c` - Hardware Abstraction Layer
Device drivers and interrupt service routines

**LCD Functions:**
- `lcd_init()` - Initialize in 4-bit mode
- `lcd_cmd()` - Send commands
- `lcd_data()` - Send data
- `lcd_puts()` - Display strings

**UART & Buffer:**
- `UART_send_char/string()` - Transmit functions
- `UartBuffer_*()` - Circular RX buffer management

**Timers & Measurements:**
- `Timer_Start_*_Delay()` - Blocking delays
- `Ultrasonic_*_Measurement()` - HC-SR04 control
- `Set_Angle_PWM()` - Servo positioning

**ADC Operations:**
- `ADC_sample()` - Single-shot reading
- `ADC_Configure_LDR1/2()` - Channel selection

**Flash Memory:**
- `Flash_EraseSegment()` - Erase operations
- `Flash_Write*()` - Program data
- Flash helper functions

##### `api.c` - Application Layer
High-level logic and FSM implementations

**Scanning Modes:**
- `servo_scan()` - 0°-180° sweep with measurements
- `object_detector()` - Object detection FSM
- `telemeter()` - Single-point measurement
- `light_detector()` - Light scanning FSM
- `light_object_detector()` - Combined mode

**Calibration:**
- `run_calibration_sequence()` - 20-sample collection
- `write_all_calib_to_flash()` - Store calibration data
- `send_calib_arr()` - Transmit calibration to PC

**File System:**
- `FileSystem_Init()` - Initialize Flash areas
- `Handle_Upload*()` - File upload protocol
- `Handle_ListFiles_Command()` - Directory listing
- `Display_File_*()` - LCD file browser

**Script Engine:**
- `parse_and_execute_line()` - Script interpreter
- `get_next_line_from_flash()` - Script reader

##### `main.c` - Main Program
Entry point and command dispatcher
- `main()` - System initialization and main loop
- `process_command()` - UART command decoder

### PC Software (Python)

#### `main.py` - GUI Application
PySimpleGUI desktop application for system control

**Core Functions:**
- **Communication**: `init_uart()`, `send/receive_data()`
- **Scanning**: `objects_detector()`, `lights_detector()`, `telemeter()`
- **Data Processing**: `process_light_scan_data()`, `find_beam_center()`
- **Visualization**: `draw_scanner_map*()` - Polar plotting
- **File Management**: `upload_file_to_mcu()`, `refresh_file_list()`
- **Calibration**: `expand_calibration_array()`, `get_distance_from_voltage()`

## 🔌 UART Protocol Reference

### Command Set

| Command | Description | Response |
|---------|-------------|----------|
| **`P`** | Handshake | `A` (ACK) |
| **`S`** | Start object scan | Stream: (2B distance + 1B angle), ends with `FF FF FF` |
| **`K`** | Start light scan | Stream: (2B LDR1 + 2B LDR2 + 1B angle) |
| **`X`** | Combined scan | Stream: (2B dist + 2B LDR1 + 2B LDR2 + 1B angle) |
| **`T`** + angle | Telemeter mode | Single measurement: (2B distance + 1B angle) |
| **`L`** | List files | File names, one per line, ends with `0x04` (EOT) |
| **`U`** | Upload file | See upload protocol below |
| **`R`** + name | Run script | Executes script from Flash |
| **`V`** | LCD view mode | Enter file browser on LCD |
| **`D`** | Delete all files | Re-initialize file system |
| **`J`** | Calibration mode | Guided LDR calibration sequence |
| **`Z`** | Get calibration | Returns 20-byte calibration array |

### File Upload Protocol
1. Send `U` → Receive `A` (ready for metadata)
2. Send metadata + checksum → Receive `B` (ready for data)
3. Send file in 64-byte chunks with checksum
   - MCU replies `K` for each chunk
   - MCU replies `S` on final chunk

### Control Markers
- **`0xCC`** - Start of script
- **`0xFE`** - Telemetry header
- **`0x7F`** - End/Sleep marker
- **`FF FF FF`** - End of scan marker
- **`0x04`** - End of transmission (EOT)

### Script Opcodes

| Opcode | Command | Description |
|--------|---------|-------------|
| `0x01` | `inc_lcd` | Increment LCD value |
| `0x02` | `dec_lcd` | Decrement LCD value |
| `0x03` | `rra_lcd` | Rotate LCD display |
| `0x04` | `set_delay` | Set delay time |
| `0x05` | `clear_lcd` | Clear LCD display |
| `0x06` | `servo_deg` | Set servo angle |
| `0x07` | `servo_scan` | Start servo scan |
| `0x08` | `sleep` | Enter sleep mode |

## 📊 Data Format
- **Endianness**: Little-Endian (Low byte first, High byte second)
- **Distance**: 16-bit value (2 bytes)
- **LDR Values**: 16-bit values (2 bytes each)
- **Angles**: 8-bit value (0-180 degrees)
- **Checksum**: 8-bit modulo-256 sum

## 🎯 Operating Modes

### 1. Object Detection Mode
- Performs 180° sweep with ultrasonic sensor
- Measures distance at each degree
- Generates polar map of surroundings

### 2. Light Detection Mode
- Scans with dual LDRs
- Identifies light source positions
- Estimates distance based on intensity

### 3. Combined Mode
- Simultaneous object and light detection
- Comprehensive environmental mapping

### 4. Telemeter Mode
- Interactive single-point measurements
- User-specified angles
- Real-time distance feedback

### 5. File Management
- Upload scripts and text files
- Browse files on LCD
- Execute stored scripts

### 6. Calibration Mode
- Guided LDR calibration sequence
- Stores calibration curves in Flash
- Improves distance estimation accuracy

## 💾 Memory Organization

### Flash Memory Map
- **Metadata Area**: File entries (name, type, size, address)
- **Content Area**: Actual file data
- **Calibration Data**: LDR calibration arrays (2 segments)
- **Maximum Files**: System-defined limit

### File Entry Structure
```c
typedef struct {
    char name[12];      // File name
    uint8_t type;       // TEXT or SCRIPT
    uint16_t size;      // File size in bytes
    uint16_t startAddr; // Start address in Flash
} FileEntry;
```

## 🚀 Getting Started

### Hardware Setup
1. Connect MSP430G2553 to PC via UART
2. Wire LCD, sensors, and servo according to pin configuration
3. Power the system

### Software Setup
1. Flash the MCU with compiled C code
2. Install Python dependencies for GUI
3. Configure COM port in Python application

### Basic Usage
1. Run Python GUI application
2. Click "Connect" to establish UART communication
3. Select desired operating mode
4. View results in real-time plots or LCD display

## 📝 Notes
- System uses interrupt-driven architecture for efficiency
- Low Power Mode (LPM) utilized during idle periods
- Circular buffer prevents UART data loss
- Flash memory provides non-volatile storage for scripts and calibration
