#!/usr/bin/env python3
import serial
import time
import csv
from datetime import datetime

PORT = '/dev/tty.usbserial-D30JZ86A'
BAUD = 9600

def log_rssi_over_time(duration_seconds=60, interval=1):
    """Log RSSI measurements to CSV file"""
    ser = serial.Serial(PORT, BAUD, timeout=1)
    
    filename = f"rssi_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    
    print(f"Logging RSSI to {filename} for {duration_seconds} seconds...")
    print("Distance (m), RSSI (dBm)")
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Timestamp', 'Distance_m', 'RSSI_dBm'])
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration_seconds:
                # Enter command mode
                time.sleep(1.1)
                ser.write(b'+++')
                time.sleep(1.1)
                ser.read(ser.in_waiting)
                
                # Read RSSI
                ser.write(b'ATDB\r')
                time.sleep(0.3)
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').strip()
                
                try:
                    rssi_hex = response.replace('OK', '').strip()
                    if rssi_hex:
                        rssi = -int(rssi_hex, 16)
                        
                        # Prompt for distance (or use predetermined)
                        distance = input(f"Enter current distance in meters (RSSI: {rssi} dBm): ")
                        
                        timestamp = datetime.now().isoformat()
                        writer.writerow([timestamp, distance, rssi])
                        print(f"Logged: {distance}m, {rssi} dBm")
                except:
                    pass
                
                # Exit command mode
                ser.write(b'ATCN\r')
                time.sleep(0.5)
                
                time.sleep(interval)
                
        except KeyboardInterrupt:
            print("\nLogging stopped")
        finally:
            ser.close()
    
    print(f"Data saved to {filename}")

if __name__ == "__main__":
    log_rssi_over_time(300, 5)  # 5 minutes, check every 5 seconds