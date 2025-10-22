import serial
import time
ser = serial.Serial('/dev/ttyUSB1', 115200 , timeout=1)
print("send A")
ser.write(b'A')
#ser.close()
time.sleep(1)
print("send 0")
ser.write(b'0')