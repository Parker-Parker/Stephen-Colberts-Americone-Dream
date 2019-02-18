import time

import serial

class RobotComs:
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = 'COM14'

    def setPort(self, port):
        self.ser.port = port

    def sendListen(self, inputs):
        self.sendCommand(inputs)
        ret = self.ser.read_all()
        print(ret)
        return ret

    def sendCommand(self, inputs):
        if len(inputs) == 5:
            signed = {("+" if int(n) >= 0 else "-") + str(abs(int(n))).zfill(3) for n in inputs}
            output = "".join(signed)
            try:
                print(output)
                self.ser.write(output.encode("ASCII"))
            except:
                pass

        elif len(inputs) == 20:
            try:
                print(inputs)
                self.ser.write(str(inputs).encode("ASCII"))
            except:
                pass
        else:
            print("something is wrong")

    def startRobotComs(self):
        self.ser = serial.Serial()
        self.ser.baudrate = 9600
        self.ser.port = 'COM14'
        self.ser.open()

    def runDemo(self):
        self.startRobotComs()
        while(1):
            printme = "-022-022+000+000+000"
            self.ser.write(printme.encode("ASCII","ignore"))
            print("send and sleep")
            print(self.ser.read_all().strip(b'\r\n'))
            time.sleep(2)
            # print(bytearray())
            # print(ser.read_all().strip(b'\r\n'))
            #######################################

            printme = "+020+020+000+000+000"
            self.ser.write(printme.encode("ASCII","ignore"))
            print("send and sleep")
            print(self.ser.read_all().strip(b'\r\n'))
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
