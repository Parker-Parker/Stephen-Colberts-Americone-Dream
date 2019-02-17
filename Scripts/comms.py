import time

import serial
ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM12'
ser.open()
ser.write(b'100,100,200,100,330;')
while(1):
    time.sleep(.5)
    ser.write(b'100,100,200,100,330')
    print("send and sleep")
    time.sleep(.5)

    print(ser.read_all().strip(b'\r\n'))