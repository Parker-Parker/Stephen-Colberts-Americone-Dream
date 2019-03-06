import time

import serial
ser = serial.Serial()
ser.baudrate = 9600
ser.port = 'COM14'
ser.open()
i = 0
stillgood = True
breaktime = time.time()
while(1):
    last = time.time()
    # time.sleep(.1)
    printme = "+333+222+000+000+000"
    ser.write(printme.encode("ASCII","ignore"))
    # time.sleep(.1)


    # print("positive")


    xx = ser.read(20).strip(b'\r\n')
    print(xx)
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
