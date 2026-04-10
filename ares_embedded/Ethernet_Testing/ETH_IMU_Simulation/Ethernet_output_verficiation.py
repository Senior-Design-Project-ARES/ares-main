import socket
import struct
import time
import matplotlib.pyplot as plt

# Connect to STM32
s = socket.socket()
s.connect(("10.42.0.187", 7))

buffer = b''
packet_size = 36

last_time = time.time()
count = 0

# Plot setup
history = []
plt.ion()

while True:
    data = s.recv(1024)
    if not data:
        print("Disconnected")
        break

    buffer += data

    while len(buffer) >= packet_size:
        packet = buffer[:packet_size]
        buffer = buffer[packet_size:]

        imu = struct.unpack('9f', packet)
        ax, ay, az, gx, gy, gz, roll, pitch, yaw = imu

        count += 1

        # Print occasionally (not every packet)
        if count % 10 == 0:
            print(f"ax={ax:.2f} ay={ay:.2f} az={az:.2f}")

        # Frequency measurement
        now = time.time()
        if now - last_time >= 1.0:
            rate = count / (now - last_time)
            print(f"\nRate: {rate:.1f} Hz\n")
            count = 0
            last_time = now

        # Update plot
        history.append(ax)
        if len(history) > 100:
            history.pop(0)

        plt.clf()
        plt.plot(history)
        plt.pause(0.01)