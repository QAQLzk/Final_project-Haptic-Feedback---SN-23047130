import serial
import keyboard
from datetime import datetime
import threading
import time

# 设置串口和波特率
serial_port = 'COM8'
baud_rate = 115200

# 创建串口连接
ser = serial.Serial(serial_port, baud_rate, timeout=0.1)


def round_time_to_0_1s(dt):
    # 四舍五入到最近的0.1秒
    rounded_time = round(dt.timestamp() * 10) / 10
    return datetime.fromtimestamp(rounded_time)

def write_data_to_csv(timestamp, x, y, z):
    with open("sensor_data.csv", "a") as file:
        file.write(f"{timestamp},{x},{y},{z}\n")


def read_serial_data():
    global baseline, zero_flag
    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if line and line.startswith('X:'):
                print("Received line:", line)
                line = line.replace(' uT', '').replace('\\', '').strip()
                parts = line.split()
                x = float(parts[1])
                y = float(parts[3])
                z = float(parts[5])

                timestamp = round_time_to_0_1s(datetime.now()).strftime('%Y-%m-%d %H:%M:%S.%f')[:-4]
                write_data_to_csv(timestamp, x, y, z)

        except Exception as e:
            print(f"ERROR: {e}")
        

if __name__ == '__main__':
    read_thread = threading.Thread(target=read_serial_data)
    read_thread.start()
    read_thread.join()
