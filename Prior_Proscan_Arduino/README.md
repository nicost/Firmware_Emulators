# Prior ProScan Controller Emulator

This Arduino sketch emulates the serial communication interface of a Prior ProScan microscope stage controller. It faithfully implements the command set as documented in the ProScan II Command Set specification.

## Features

- Full RS-232 command set emulation
- Support for both Standard and Compatibility modes
- XYZ stage position tracking and movement commands
- Realistic motion timing based on distance and speed settings
- Independent XY and Z axis motion with different speeds
- Configurable speed, acceleration, and S-curve parameters
- Joystick control emulation
- Flexible command delimiter support (COMMA, SPACE, TAB, EQUALS, SEMICOLON, COLON)
- Baud rate switching (9600, 19200, 38400)

## Hardware Requirements

- Arduino board (Uno, Mega, or compatible)
- USB or RS-232 connection to host computer

## Installation

1. Open the `Prior_Proscan_Arduino.ino` file in the Arduino IDE
2. Select your Arduino board type from Tools > Board
3. Select the appropriate COM port from Tools > Port
4. Upload the sketch to your Arduino

## Serial Configuration

- **Default Baud Rate**: 9600
- **Data Bits**: 8
- **Parity**: None
- **Stop Bits**: 1
- **Flow Control**: None
- **Command Termination**: Carriage Return (CR, '\r')

## Usage

### Connecting

1. Connect your Arduino to your computer via USB
2. Open a serial terminal (Arduino Serial Monitor, PuTTY, TeraTerm, etc.)
3. Configure the terminal to 9600 baud, 8-N-1, CR line ending
4. The emulator is ready to receive commands

### Command Format

Commands are sent as ASCII text terminated with a Carriage Return (CR). Multiple delimiters are supported:

```
G,100,200<CR>      (comma-separated)
G 100 200<CR>      (space-separated)
G=100,200<CR>      (mixed delimiters)
P<CR>              (position query)
```

## Supported Commands

### General Commands

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `?` | None | Text | Reports controller and peripheral information |
| `DATE` | None | Text | Reports version and compile date |
| `VERSION` | None | Number | Reports software version (100 = v1.00) |
| `SERIAL` | None | Number | Reports serial number |
| `COMP` | None/0/1 | 0/1 | Get/Set compatibility mode (0=Standard, 1=Compat) |
| `ERROR` | None/0/1 | 0/1 | Get/Set error reporting mode (0=codes, 1=human) |
| `$` | None/axis | Number | Reports motion status |
| `=` | None | Hex | Reports and clears limit switch hit status |
| `LMT` | None | Hex | Reports current limit switch status |
| `I` | None | R | Controlled stop (empties queue) |
| `K` | None | R | Emergency stop |
| `BAUD` | 96/19/38 | 0 | Sets baud rate (96=9600, 19=19200, 38=38400) |

### Position Commands

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `P` | None | x,y,z | Reports absolute position |
| `P` | x,y,z | 0 | Sets absolute position |
| `PS` | None | x,y | Reports stage (XY) position |
| `PS` | x,y | 0 | Sets stage position |
| `PX` | None | x | Reports X position |
| `PX` | x | 0 | Sets X position |
| `PY` | None | y | Reports Y position |
| `PY` | y | 0 | Sets Y position |
| `PZ` | None | z | Reports Z position |
| `PZ` | z | 0 | Sets Z position |

### Movement Commands (Absolute)

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `G` | x,y[,z] | R | Go to absolute position x,y,z |
| `GR` | x,y[,z] | R | Go relative (move by offset) |
| `GX` | x | R | Move to absolute X position |
| `GY` | y | R | Move to absolute Y position |
| `GZ` | z | R | Move to absolute Z position |
| `V` | z | R | Move to absolute Z position |
| `M` | None | R | Move to home (0,0,0) |
| `Z` | None | 0 | Set position to zero (0,0,0) |

### Movement Commands (Relative)

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `F` | None/y | R | Move forward by step size or y steps |
| `B` | None/y | R | Move back by step size or y steps |
| `L` | None/x | R | Move left by step size or x steps |
| `R` | None/x | R | Move right by step size or x steps |
| `U` | None/z | R | Move up by step size or z steps |
| `D` | None/z | R | Move down by step size or z steps |

### Step Size Configuration

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `X` | None | u,v | Reports XY step sizes |
| `X` | u,v | 0 | Sets XY step sizes |
| `C` | None | w | Reports Z step size |
| `C` | w | 0 | Sets Z step size |

### Speed and Acceleration

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `SMS` | None/m | m/0 | Get/Set stage maximum speed (1-100%) |
| `SAS` | None/a | a/0 | Get/Set stage acceleration (1-100%) |
| `SCS` | None/c | c/0 | Get/Set stage S-curve (1-100%) |
| `SMZ` | None/m | m/0 | Get/Set Z maximum speed (1-100%) |
| `SAZ` | None/a | a/0 | Get/Set Z acceleration (1-100%) |
| `SCZ` | None/c | c/0 | Get/Set Z S-curve (1-100%) |

### Joystick Control

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `J` | None | 0 | Enable joystick |
| `H` | None | 0 | Disable joystick |
| `O` | None/s | s/0 | Get/Set joystick speed (1-100%) |
| `OF` | None/s | s/0 | Get/Set focus (Z) joystick speed (1-100%) |
| `JXD` | None/d | d/0 | Get/Set X joystick direction (1/-1) |
| `JYD` | None/d | d/0 | Get/Set Y joystick direction (1/-1) |
| `JZD` | None/d | d/0 | Get/Set Z joystick direction (1/-1) |

### Direction Configuration

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `XD` | 1/-1 | 0 | Sets X axis direction |
| `YD` | 1/-1 | 0 | Sets Y axis direction |

### Information Commands

| Command | Arguments | Response | Description |
|---------|-----------|----------|-------------|
| `STAGE` | None | Text | Reports stage information |
| `FOCUS` | None | Text | Reports focus unit information |
| `RES` | axis | value | Reports resolution for axis |
| `RES` | axis,r | 0 | Sets resolution for axis (microns) |
| `UPR` | axis | value | Reports microns per revolution |
| `UPR` | axis,n | 0 | Sets microns per revolution |

## Example Session

```
> ?<CR>
PROSCAN INFORMATION
DSP_1 IS 4-AXIS STEPPER VERSION 2.7
DSP_2 IS 2-AXIS STEPPER VERSION 2.7
DRIVE CHIPS 010111 (F2 F1 A Z Y X) 0 = Not Fitted
JOYSTICK ACTIVE
STAGE = H101/2
FOCUS = NORMAL
FILTER_1 = NONE
FILTER_2 = NONE
SHUTTERS = 000 (S3 S2 S1) 0 = Not Fitted
AUTOFOCUS = NONE
VIDEO = NONE
END

> P<CR>
0,0,0

> G,1000,2000,500<CR>
R

> P<CR>
1000,2000,500

> PX<CR>
1000

> GR,100,0,0<CR>
R

> P<CR>
1100,2000,500
```

## Operating Modes

### Standard Mode (Default)
- Commands can be queued (up to 100 commands)
- Non-blocking operation
- Supports MACRO and SOAK commands
- All commands require CR termination

### Compatibility Mode
- Compatible with older H127/H128 controllers
- Some commands (I, K, #) work without CR
- Less flexible, no MACRO/SOAK support
- Set with `COMP,1`

## State Management

The emulator maintains the following state:

- **Position**: X, Y, Z coordinates
- **Step Sizes**: Default step sizes for relative moves
- **Speed Settings**: Maximum speeds, acceleration, S-curve for XY and Z
- **Joystick**: Enable/disable state and speed settings
- **Directions**: Axis direction multipliers
- **Mode Flags**: Compatibility mode, error reporting mode
- **Configuration**: Resolution, microns per revolution

## Motion Timing

The emulator now includes realistic motion timing that simulates actual stage behavior:

### Base Speeds
- **XY Stage**: 10,000 microns/sec (10 mm/sec) at 100% speed
- **Z Focus**: 1,000 microns/sec (1 mm/sec) at 100% speed

### How It Works
- Movement commands (G, GR, GX, GY, GZ, V, F, B, L, R, U, D, M) trigger motion that takes time
- Motion duration is calculated based on distance and current speed settings (SMS for XY, SMZ for Z)
- XY and Z axes move independently and may complete at different times
- Position queries (P, PS, PX, PY, PZ) return interpolated positions during motion
- Motion status ($) correctly reports which axes are moving:
  - Bit 0 (0x01): X axis moving
  - Bit 1 (0x02): Y axis moving
  - Bit 2 (0x04): Z axis moving
  - Example: status=3 means X and Y moving, status=7 means all three axes moving
- Stop commands (I, K) immediately halt motion at the current interpolated position
- Serial communication continues to work during motion (non-blocking implementation)

### Motion Algorithm
- Uses constant velocity (linear interpolation)
- Acceleration and S-curve parameters are accepted but not yet simulated
- Motion timing: `duration = distance / (baseSpeed * speedPercent / 100)`

## Notes

- All positions are maintained as long integers
- Speed, acceleration, and S-curve values are constrained to 1-100%
- Motion timing uses millis() for non-blocking operation
- Limit switches are not physically implemented (always report inactive)

## Arduino Auto-Reset Behavior

### The Issue

When opening a serial connection to the Arduino, the board automatically resets due to the DTR/RTS signal. This causes:

1. Arduino resets (~100-500ms)
2. Bootloader runs for approximately 2 seconds waiting for code upload
3. The emulator sketch finally starts running

**Any commands sent during the first ~2-3 seconds after opening the port will be lost.**

### Solutions

#### For Host Software (like Micro-Manager)

**Option 1: Add Initialization Delay (Recommended)**
- Configure your software to wait 2-3 seconds after opening the serial port before sending the first command
- This is the simplest and most reliable solution
- Example: The test script (`test_emulator.py`) includes a 2-second delay on line 35

**Option 2: Wait for Ready Signal**
- Some versions of this emulator may send controller information on startup
- Wait for "END\r" marker before sending commands

#### For Hardware (Production Use)

If you need instant communication without delays:
- Add a 10µF capacitor between RESET and GND pins on the Arduino
- This prevents auto-reset when the serial port opens
- **Note**: This makes uploading new code more difficult (requires manual reset)

### Delimiter Support

The emulator correctly implements all ProScan delimiters:
- `,` (COMMA)
- ` ` (SPACE)
- `\t` (TAB)
- `=` (EQUALS)
- `;` (SEMICOLON)
- `:` (COLON)

All commands work with any delimiter or combination of delimiters.

## Testing

### Automated Test Suite

A comprehensive Python test suite is included (`test_emulator.py`):

**Basic Command Tests** (default):
```bash
python test_emulator.py COM3
```

**Motion Timing Tests** (verifies realistic motion behavior):
```bash
python test_emulator.py COM3 --timing
```

**All Tests** (basic + motion timing):
```bash
python test_emulator.py COM3 --all
```

The motion timing tests verify:
- Motion takes realistic time based on distance and speed
- Position interpolation during motion
- Motion status reporting ($ command)
- Independent XY and Z axis motion at different speeds
- Speed settings (SMS/SMZ) affect motion duration
- Stop commands (I/K) halt motion at current position

### Manual Testing

You can test the emulator using any serial terminal:

1. Arduino IDE Serial Monitor (set line ending to "Carriage return")
2. PuTTY (Serial connection)
3. TeraTerm
4. screen/minicom on Linux
5. Python with pyserial

Example Python test:
```python
import serial
import time

ser = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Wait for Arduino reset

# Query initial position
ser.write(b'P\r')
print(ser.readline().decode())  # Should print "0,0,0"

# Move to position
ser.write(b'G,10000,0,0\r')
print(ser.readline().decode())  # Should print "R"

# Query position during motion (after 0.5 seconds)
time.sleep(0.5)
ser.write(b'PX\r')
print(ser.readline().decode())  # Should print intermediate value (~5000)

# Wait for motion to complete
time.sleep(0.6)
ser.write(b'P\r')
print(ser.readline().decode())  # Should print "10000,0,0"

# Check motion status
ser.write(b'$\r')
print(ser.readline().decode())  # Should print "0" (not moving)

ser.close()
```

## Limitations

This is an emulator and does not control physical hardware:

- No actual motor control
- No physical limit switches
- Motion uses constant velocity (no acceleration or S-curve simulation yet)
- Backlash commands are accepted but not simulated
- Filter wheel and shutter commands not implemented

## References

- Prior ProScan II Command Set documentation
- Original command set: ~/tmp/ProScan_Command_set.txt

## License

This is an educational emulator for testing and development purposes.

## Version History

- v1.0 - Initial implementation with core command set
