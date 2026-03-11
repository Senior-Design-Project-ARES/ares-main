#!/usr/bin/env python3
import serial
import time

PORT = '/dev/tty.usbserial-D30JZ86A'
BAUD = 9600

def send_at_command(ser, command, wait=0.5):
    """Send AT command and return response"""
    ser.write((command + '\r').encode())
    time.sleep(wait)
    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    return response

def configure_xbee(mode='coordinator'):
    """Configure XBee module"""
    ser = serial.Serial(PORT, BAUD, timeout=1)
    
    print(f"Configuring XBee on {PORT} as {mode}")
    print("=" * 50)
    
    # Enter command mode
    time.sleep(1.1)
    ser.write(b'+++')
    time.sleep(1.1)
    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    
    if 'OK' not in response:
        print("Failed to enter command mode!")
        ser.close()
        return False
    
    print("✓ Entered command mode")
    
    # Read current settings
    print("\nCurrent Configuration:")
    print(f"PAN ID: {send_at_command(ser, 'ATID').strip()}")
    print(f"Channel: {send_at_command(ser, 'ATCH').strip()}")
    print(f"MY Address: {send_at_command(ser, 'ATMY').strip()}")
    print(f"DL Address: {send_at_command(ser, 'ATDL').strip()}")
    print(f"DH Address: {send_at_command(ser, 'ATDH').strip()}")
    
    # Configure common settings
    print("\nConfiguring...")
    
    # Set PAN ID (must be same on both XBees)
    send_at_command(ser, 'ATID1234')  # Use PAN ID 0x1234
    print("✓ Set PAN ID to 1234")
    
    # Set channel (must be same on both XBees)
    send_at_command(ser, 'ATCHC')  # Channel C (0x0C)
    print("✓ Set Channel to C")
    
    if mode == 'coordinator':
        # Coordinator settings
        send_at_command(ser, 'ATMY0')      # My address = 0
        send_at_command(ser, 'ATDL1')      # Destination = 1 (router)
        send_at_command(ser, 'ATDH0')      # Destination high = 0
        print("✓ Configured as Coordinator (MY=0, DL=1)")
    else:
        # Router settings
        send_at_command(ser, 'ATMY1')      # My address = 1
        send_at_command(ser, 'ATDL0')      # Destination = 0 (coordinator)
        send_at_command(ser, 'ATDH0')      # Destination high = 0
        print("✓ Configured as Router (MY=1, DL=0)")
    
    # Write settings to flash
    send_at_command(ser, 'ATWR')
    print("✓ Written to flash")
    
    # Verify settings
    print("\nNew Configuration:")
    print(f"PAN ID: {send_at_command(ser, 'ATID').strip()}")
    print(f"Channel: {send_at_command(ser, 'ATCH').strip()}")
    print(f"MY Address: {send_at_command(ser, 'ATMY').strip()}")
    print(f"DL Address: {send_at_command(ser, 'ATDL').strip()}")
    
    # Exit command mode
    send_at_command(ser, 'ATCN')
    print("\n✓ Configuration complete!")
    
    ser.close()
    return True

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python3 configure_xbee.py [coordinator|router]")
        print("\nRun this on BOTH XBees:")
        print("  Jetson #1: python3 configure_xbee.py coordinator")
        print("  Jetson #2: python3 configure_xbee.py router")
        sys.exit(1)
    
    mode = sys.argv[1].lower()
    if mode not in ['coordinator', 'router']:
        print("Mode must be 'coordinator' or 'router'")
        sys.exit(1)
    
    configure_xbee(mode)