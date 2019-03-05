import time

import serial
# import struct


# def makeBytes(intToSend):
#     char0 = intToSend//256
#     char1 = intToSend % 256
#     struct.pack('>B', char0, char1)
#     return None
#
# print(makeBytes(30))
# print(makeBytes(30000))

print(str(-2).zfill(3))

ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM14'
ser.open()
command = ""
v1 = 5
v2 = 0
v3 = 0
v4 = 0
v5 = 0
s1 = "+"
# ser.write(b'100,100,200,100,330;')
i = 0
stillgood = True
breaktime = time.time()
while(1):
    # #
    # # if(int(v1)>=0):
    # #     s1 = "+"
    # # else:
    # #     s1 = "-"
    # #
    # # s2 = str(v2).zfill(3)
    # # s3 = str(v3).zfill(3)
    # # s4 = str(v4).zfill(3)
    # # s5 = str(v5).zfill(3)
    # #
    # #
    # # v1 = str(abs(int(v1))).zfill(3)
    # # v2 = str(abs(int(v2))).zfill(3)
    # # v3 = str(abs(int(v3))).zfill(3)
    # # v4 = str(abs(int(v4))).zfill(3)
    # # v5 = str(abs(int(v5))).zfill(3)
    # #
    # # printme = s1+v1+"+"+v2+"+"+v3+"+"+v4+"+"+v5
    # # print(printme)
    # # # time.sleep(.5)
    # # # for n in printme.encode("ASCII","ignore"):
    # # ser.write(printme.encode("ASCII","ignore"))
    # # print("send and sleep")
    # time.sleep(2)
    # # print(bytearray())
    # # print(ser.read_all().strip(b'\r\n'))
    # #######################################

    printme = "-024-024+000+000+000"
    ser.write(printme.encode("ASCII","ignore"))
    time.sleep(.2)
    print("negative")
    print(ser.read_all().strip(b'\r\n'))
    # time.sleep(2)
    # print(bytearray())
    # print(ser.read_all().strip(b'\r\n'))
    #######################################


    printme = "+024+024+000+000+000"
    ser.write(printme.encode("ASCII","ignore"))
    time.sleep(.2)
    print("positive")
    xx = ser.read_all().strip(b'\r\n')
    print(xx)

    if (not xx == "+024+024+000+000+000".encode("ASCII")) and stillgood and (i>4):
        stillgood = False
        breaktime = time.time() - breaktime
    i+=1

    if not stillgood:
        print("stopped working at "+ str(breaktime))
    else:
        print("still working at " + str(time.time() - breaktime)+"s at a rate of 0.2s")

    # print(ser.read_all().strip(b'\r\n'))


    # v5 = -5
    # if(i>10):
    #     v5 = 1
    # if(i<-10):
    #     v5 = -1
    # if up:
    #     v5 = (int(v5)+1)
    # else:
    #     v5 = (int(v5)-1)
    # v1 = v5
    # v2 = v5


    # time.sleep(.2)

    # printme = "-001-001+015+015+015"
    # ser.write(printme.encode("ASCII","ignore"))
    # print("send and sleep")
    # print(ser.read_all().strip(b'\r\n'))
