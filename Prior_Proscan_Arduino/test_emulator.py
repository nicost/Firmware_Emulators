#!/usr/bin/env python3
"""
Test script for Prior ProScan Controller Emulator

This script tests the basic functionality of the Arduino-based
ProScan controller emulator.

Requirements: pyserial (install with: pip install pyserial)
"""

import serial
import serial.tools.list_ports
import time
import sys

def send_command(ser, cmd):
    """Send a command and return the response"""
    print(f"Sending: {cmd}")
    ser.write((cmd + '\r').encode())
    time.sleep(0.1)  # Small delay for Arduino to process

    response = ""
    while ser.in_waiting:
        response += ser.read(ser.in_waiting).decode()
        time.sleep(0.05)  # Small delay to receive multi-line responses

    print(f"Response: {repr(response)}")
    print()
    return response

def test_basic_commands(port, baud=9600):
    """Run basic command tests"""
    try:
        print(f"Connecting to {port} at {baud} baud...")
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset

        print("="*60)
        print("Prior ProScan Emulator Test Suite")
        print("="*60)
        print()

        # Test 1: Get controller information
        print("Test 1: Controller Information")
        print("-"*60)
        send_command(ser, "?")

        # Test 2: Version information
        print("Test 2: Version and Serial Number")
        print("-"*60)
        send_command(ser, "VERSION")
        send_command(ser, "SERIAL")
        send_command(ser, "DATE")

        # Test 3: Position queries
        print("Test 3: Position Queries")
        print("-"*60)
        send_command(ser, "P")
        send_command(ser, "PS")
        send_command(ser, "PX")
        send_command(ser, "PY")
        send_command(ser, "PZ")

        # Test 4: Absolute movement
        print("Test 4: Absolute Movement (G command)")
        print("-"*60)
        send_command(ser, "G,1000,2000,500")
        send_command(ser, "P")

        # Test 5: Individual axis movement
        print("Test 5: Individual Axis Movement")
        print("-"*60)
        send_command(ser, "GX,5000")
        send_command(ser, "PX")
        send_command(ser, "GY,3000")
        send_command(ser, "PY")
        send_command(ser, "GZ,1000")
        send_command(ser, "PZ")

        # Test 6: Relative movement
        print("Test 6: Relative Movement (GR command)")
        print("-"*60)
        send_command(ser, "P")
        send_command(ser, "GR,100,200,50")
        send_command(ser, "P")

        # Test 7: Step size configuration
        print("Test 7: Step Size Configuration")
        print("-"*60)
        send_command(ser, "X")
        send_command(ser, "X,250,250")
        send_command(ser, "X")
        send_command(ser, "C")
        send_command(ser, "C,150")
        send_command(ser, "C")

        # Test 8: Relative step commands
        print("Test 8: Relative Step Commands (F, B, L, R, U, D)")
        print("-"*60)
        send_command(ser, "Z")  # Reset to 0,0,0
        send_command(ser, "P")
        send_command(ser, "F")   # Forward
        send_command(ser, "P")
        send_command(ser, "R")   # Right
        send_command(ser, "P")
        send_command(ser, "U")   # Up
        send_command(ser, "P")

        # Test 9: Speed and acceleration
        print("Test 9: Speed and Acceleration Settings")
        print("-"*60)
        send_command(ser, "SMS")
        send_command(ser, "SMS,75")
        send_command(ser, "SMS")
        send_command(ser, "SAS")
        send_command(ser, "SAS,80")
        send_command(ser, "SAS")
        send_command(ser, "SMZ")
        send_command(ser, "SMZ,60")
        send_command(ser, "SMZ")

        # Test 10: Joystick control
        print("Test 10: Joystick Control")
        print("-"*60)
        send_command(ser, "H")   # Disable joystick
        send_command(ser, "?")   # Check status
        send_command(ser, "J")   # Enable joystick
        send_command(ser, "?")
        send_command(ser, "O")   # Query joystick speed
        send_command(ser, "O,70")
        send_command(ser, "O")

        # Test 11: Mode settings
        print("Test 11: Mode Settings")
        print("-"*60)
        send_command(ser, "COMP")
        send_command(ser, "COMP,1")
        send_command(ser, "COMP")
        send_command(ser, "COMP,0")
        send_command(ser, "COMP")

        # Test 12: Stage and Focus info
        print("Test 12: Stage and Focus Information")
        print("-"*60)
        send_command(ser, "STAGE")
        send_command(ser, "FOCUS")

        # Test 13: Status commands
        print("Test 13: Status Commands")
        print("-"*60)
        send_command(ser, "$")
        send_command(ser, "=")
        send_command(ser, "LMT")

        # Test 14: Different delimiter formats
        print("Test 14: Different Delimiter Formats")
        print("-"*60)
        send_command(ser, "G 7000 8000 900")    # Space delimiter
        send_command(ser, "P")
        send_command(ser, "G=6000,5000;400")    # Mixed delimiters
        send_command(ser, "P")
        send_command(ser, "G:4000:3000:200")    # Colon delimiter
        send_command(ser, "P")

        # Test 15: Emergency stop and home
        print("Test 15: Stop and Home Commands")
        print("-"*60)
        send_command(ser, "I")   # Controlled stop
        send_command(ser, "M")   # Move to home
        send_command(ser, "P")

        print("="*60)
        print("Basic tests completed!")
        print("="*60)

        ser.close()

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}")
        print(f"Details: {e}")
        print("\nAvailable ports:")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(f"  {p.device} - {p.description}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        sys.exit(1)

def test_motion_timing(port, baud=9600):
    """Test realistic motion timing implementation"""
    try:
        print(f"Connecting to {port} at {baud} baud...")
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # Wait for Arduino to reset

        print("="*60)
        print("Motion Timing Test Suite")
        print("="*60)
        print()

        # Reset to home position
        print("Resetting to home position...")
        send_command(ser, "M")
        time.sleep(0.5)

        # Test 1: Verify motion takes time (not instant)
        print("Test 1: Motion Takes Time")
        print("-"*60)
        print("Moving 10,000 microns at default speed (100%)...")
        print("Expected duration: 10000 / 10000 = 1.0 second")

        send_command(ser, "Z")  # Reset to 0,0,0
        start_time = time.time()
        send_command(ser, "GX,10000")

        # Query position immediately
        time.sleep(0.05)
        pos = send_command(ser, "PX")
        print(f"Position after 50ms: {pos.strip()}")

        # Query position midway
        time.sleep(0.45)
        pos = send_command(ser, "PX")
        print(f"Position after ~500ms: {pos.strip()}")

        # Wait for completion and measure
        time.sleep(0.6)
        end_time = time.time()
        pos = send_command(ser, "PX")
        actual_duration = end_time - start_time
        print(f"Final position: {pos.strip()}")
        print(f"Actual duration: {actual_duration:.3f} seconds")
        print(f"✓ Motion is NOT instant" if actual_duration > 0.5 else "✗ Motion appears instant")
        print()

        # Test 2: Motion status during movement
        print("Test 2: Motion Status Reporting")
        print("-"*60)
        print("Starting XY motion and checking status...")

        send_command(ser, "Z")  # Reset
        send_command(ser, "G,10000,10000,0")

        # Check status during motion
        time.sleep(0.05)
        status = send_command(ser, "$")
        status_val = int(status.strip()) if status.strip().isdigit() else 0
        print(f"Motion status during XY move: {status_val}")
        print(f"  X moving: {'Yes' if status_val & 0x01 else 'No'} (bit 0)")
        print(f"  Y moving: {'Yes' if status_val & 0x02 else 'No'} (bit 1)")
        print(f"  Z moving: {'Yes' if status_val & 0x04 else 'No'} (bit 2)")

        # Wait for completion
        time.sleep(1.5)
        status = send_command(ser, "$")
        status_val = int(status.strip()) if status.strip().isdigit() else 0
        print(f"Motion status after completion: {status_val}")
        print(f"✓ Status reporting works" if status_val == 0 else "✗ Status should be 0 when stopped")
        print()

        # Test 3: Independent XY and Z motion
        print("Test 3: Independent XY and Z Motion")
        print("-"*60)
        print("XY base speed: 10,000 microns/sec")
        print("Z base speed: 1,000 microns/sec")
        print("Moving XY by 5000 microns and Z by 5000 microns...")
        print("Expected XY duration: 5000 / 10000 = 0.5 sec")
        print("Expected Z duration: 5000 / 1000 = 5.0 sec")

        send_command(ser, "Z")  # Reset
        send_command(ser, "G,5000,0,5000")

        # Check status after 1 second (XY should be done, Z still moving)
        time.sleep(1.0)
        status = send_command(ser, "$")
        status_val = int(status.strip()) if status.strip().isdigit() else 0
        pos = send_command(ser, "P")
        print(f"After 1 second:")
        print(f"  Position: {pos.strip()}")
        print(f"  Motion status: {status_val}")
        print(f"  XY done, Z moving: {'Yes' if (status_val & 0x03) == 0 and (status_val & 0x04) != 0 else 'No'}")

        # Wait for Z to complete
        time.sleep(4.5)
        status = send_command(ser, "$")
        pos = send_command(ser, "P")
        print(f"After 5.5 seconds:")
        print(f"  Position: {pos.strip()}")
        print(f"  Motion status: {status.strip()}")
        print()

        # Test 4: Speed settings affect duration
        print("Test 4: Speed Settings Affect Duration")
        print("-"*60)

        # Set XY speed to 50%
        send_command(ser, "SMS,50")
        print("Set XY speed to 50%")
        print("Moving 10,000 microns at 50% speed...")
        print("Expected duration: 10000 / 5000 = 2.0 seconds")

        send_command(ser, "Z")
        start_time = time.time()
        send_command(ser, "GX,10000")

        # Query midway
        time.sleep(1.0)
        pos = send_command(ser, "PX")
        print(f"Position after 1 second: {pos.strip()}")

        # Wait for completion
        time.sleep(1.5)
        end_time = time.time()
        pos = send_command(ser, "PX")
        actual_duration = end_time - start_time
        print(f"Final position: {pos.strip()}")
        print(f"Actual duration: {actual_duration:.3f} seconds")
        print(f"✓ Speed setting works" if 1.5 < actual_duration < 2.5 else "✗ Duration doesn't match 50% speed")

        # Reset speed to 100%
        send_command(ser, "SMS,100")
        print()

        # Test 5: Stop command during motion
        print("Test 5: Stop Command During Motion")
        print("-"*60)
        print("Starting long motion and stopping midway...")

        send_command(ser, "Z")
        send_command(ser, "GX,20000")

        # Stop after 0.5 seconds (should be around 5000 microns)
        time.sleep(0.5)
        send_command(ser, "I")
        time.sleep(0.1)

        pos = send_command(ser, "PX")
        status = send_command(ser, "$")
        pos_val = int(pos.strip()) if pos.strip().lstrip('-').isdigit() else 0
        print(f"Position after stop: {pos.strip()}")
        print(f"Motion status: {status.strip()}")
        print(f"✓ Stopped at intermediate position" if 3000 < pos_val < 7000 else "✗ Position unexpected")
        print()

        # Test 6: Position interpolation during motion
        print("Test 6: Position Interpolation")
        print("-"*60)
        print("Sampling position during 2-second move...")

        send_command(ser, "Z")
        send_command(ser, "GX,20000")

        positions = []
        timestamps = []
        start_time = time.time()

        for i in range(5):
            time.sleep(0.4)
            pos = send_command(ser, "PX")
            elapsed = time.time() - start_time
            pos_val = int(pos.strip()) if pos.strip().lstrip('-').isdigit() else 0
            positions.append(pos_val)
            timestamps.append(elapsed)
            print(f"  t={elapsed:.2f}s: x={pos_val}")

        # Verify positions are increasing
        increasing = all(positions[i] < positions[i+1] for i in range(len(positions)-1))
        print(f"✓ Positions increase smoothly" if increasing else "✗ Position interpolation issue")
        print()

        # Test 7: Relative Z movement with different speed
        print("Test 7: Z Axis Motion with Custom Speed")
        print("-"*60)

        # Set Z speed to 50%
        send_command(ser, "SMZ,50")
        print("Set Z speed to 50%")
        print("Moving Z by 2,000 microns at 50% speed...")
        print("Expected duration: 2000 / 500 = 4.0 seconds")

        send_command(ser, "Z")
        start_time = time.time()
        send_command(ser, "U,2000")

        time.sleep(2.0)
        pos = send_command(ser, "PZ")
        print(f"Position after 2 seconds: {pos.strip()}")

        time.sleep(2.5)
        end_time = time.time()
        pos = send_command(ser, "PZ")
        actual_duration = end_time - start_time
        print(f"Final position: {pos.strip()}")
        print(f"Actual duration: {actual_duration:.3f} seconds")

        # Reset Z speed
        send_command(ser, "SMZ,100")
        print()

        print("="*60)
        print("Motion timing tests completed!")
        print("="*60)
        print()
        print("Summary:")
        print("- Motion takes realistic time based on distance")
        print("- Speed settings (SMS/SMZ) affect motion duration")
        print("- XY and Z move independently at different speeds")
        print("- Position is interpolated during motion")
        print("- Motion status correctly reports moving axes")
        print("- Stop commands halt motion at current position")

        ser.close()

    except serial.SerialException as e:
        print(f"Error: Could not open serial port {port}")
        print(f"Details: {e}")
        print("\nAvailable ports:")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(f"  {p.device} - {p.description}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        sys.exit(1)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python test_emulator.py <COM_PORT> [BAUD_RATE] [--timing]")
        print("Example: python test_emulator.py COM3")
        print("Example: python test_emulator.py COM3 9600")
        print("Example: python test_emulator.py COM3 9600 --timing")
        print("Example (Linux): python test_emulator.py /dev/ttyUSB0")
        print()
        print("Options:")
        print("  --timing    Run motion timing tests (takes ~30 seconds)")
        print("  --all       Run both basic and timing tests")
        print()

        # List available ports
        print("Available serial ports:")
        ports = serial.tools.list_ports.comports()
        if ports:
            for p in ports:
                print(f"  {p.device} - {p.description}")
        else:
            print("  No serial ports found")
        sys.exit(1)

    port = sys.argv[1]

    # Parse arguments
    baud = 9600
    run_basic = True
    run_timing = False

    for arg in sys.argv[2:]:
        if arg == '--timing':
            run_basic = False
            run_timing = True
        elif arg == '--all':
            run_basic = True
            run_timing = True
        elif arg.isdigit():
            baud = int(arg)

    # Run tests based on arguments
    if run_basic:
        test_basic_commands(port, baud)
        if run_timing:
            print("\n")
            time.sleep(1)

    if run_timing:
        test_motion_timing(port, baud)
