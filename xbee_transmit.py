#!/usr/bin/env python3
import serial
import time

PORT = '/dev/tty.usbserial-D30JZ86A'  # Adjust to your device
BAUD = 9600

def transmit_continuously():
    """Continuously transmit data so receiver can measure RSSI"""
    ser = serial.Serial(PORT, BAUD, timeout=1)
    
    print(f"Transmitting on {PORT} at {BAUD} baud...")
    print("Press Ctrl+C to stop\n")
    
    counter = 0
    
    try:
        while True:
            message = f"Packet {counter:05d} - {time.time():.2f}\n"
            ser.write(message.encode())
            print(f"Sent: {message.strip()}")
            counter += 1
            time.sleep(1)  # Send every 1 second
            
    except KeyboardInterrupt:
        print("\nTransmission stopped")
    finally:
        ser.close()

if __name__ == "__main__":
    transmit_continuously()