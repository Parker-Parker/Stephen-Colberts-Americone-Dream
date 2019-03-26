import time

import serial
ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM14'
ser.timeout = None
# ser.port = 'COM12'
ser.open()
ser.timeout = None
i = 0
stillgood = True
breaktime = time.time()
while(1):
    last = time.time()
    # time.sleep(.1)
    printme = " 200 200 000 000 000"
    ser.write(printme.encode("ASCII","ignore"))
    # time.sleep(.1)


    # print("positive")
    #
    # while ser.in_waiting<40:
    #     print(ser.in_waiting)

    xx = ser.read(40).strip(b'\r\n')
    print(xx)

    # xx = ser.read_all().strip(b'\r\n')
    # print(xx)
    """
    print(time.time()-last)
    
    #
    # if (not xx == "+024+024+000+000+000".encode("ASCII")) and stillgood and (i>4):
    #     stillgood = False
    #     breaktime = time.time() - breaktime
    # i+=1
    #
    # if not stillgood:
    #     print("stopped working at "+ str(breaktime))
    # else:
    #     print("still working at " + str(time.time() - breaktime)+"s at a rate of 0.2s")

    print("still working at " + str(time.time() - breaktime)+"s at a rate of titties")

    """
