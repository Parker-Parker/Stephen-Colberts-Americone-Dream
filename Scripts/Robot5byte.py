import math
from serial.tools.list_ports import comports
import serial
import time
import math
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Comms:

    def __init__(self):
        self.serialPort = serial.Serial()
        self.availableDevices = []
        self.availableDeviceDescriptions = []
        self.availablePorts = []
        for i in comports():
            self.availablePorts.append(i)
            self.availableDevices.append(i.device)
            self.availableDeviceDescriptions.append(i.description)
        # [self.serialPort.setPort(i.device) for i in self.availablePorts if i.description == "1234567876543234567"]
        # [self.serialPort.setPort(i.device) for i in self.availablePorts if i.device == "COM12" or i.device == "COM14"]
        yeet = []
        for i in self.availablePorts:
            if i.device == "COM12" or i.device == "COM13" or i.device == "COM14" or i.device == "COM16":
                # self.serialPort.setPort(i.device)
                print(i)
                self.serialPort = serial.Serial(i.device)
        self.serialPort.baudrate = 115200
        # self.robot = Robot

    def registerRobot(self, robot):
        print("regstart")
        self.robot = robot
        self.robot.comms = self
        # self.serialPort.open()
        # self.serialPort.write(b'123')
        time.sleep(1)
        self.serialPort.flushInput()
        time.sleep(1)
        # self.serialPort.write("-000-000+000+000+007".encode("ASCII", "ignore"))
        # self.serialPort.write("-000-000+000+000+008".encode("ASCII", "ignore"))
        # self.serialPort.write("-000-000+000+000+009".encode("ASCII", "ignore"))
        # self.serialPort.write("-000-000+000+000+009".encode("ASCII", "ignore"))
        # # print("regwait1")
        # # self.serialPort.read(60)
        # # n = 0
        #
        # print("regwait2")
        # while not n == ';':
        #     try:
        #         x = self.serialPort.read(1)
        #         n = x.decode("ASCII")
        #     except:
        #         pass
        # self.serialPort.read(1)
        print("regend")

    def parseLine(self):
        if self.serialPort.is_open:

            self.serialPort.timeout = None
            # self.serialPort.write("000-000+000+000+000".encode("ASCII", "ignore"))



            v1 = str(abs(int(self.robot.jointTargets[0]*100))).zfill(5)
            v2 = str(abs(int(self.robot.jointTargets[1]*100))).zfill(5)
            v3 = str(abs(int(self.robot.jointTargets[2]*100))).zfill(5)
            v4 = str(abs(int(self.robot.jointTargets[3]*100))).zfill(5)
            v5 = str(abs(int(self.robot.jointTargets[4]*100))).zfill(5)

            if int(v2)>17000:
                v2 = "17000"
                print("v2 capped at 170")

            if int(v2)<11500:
                v2 = "11500"
                print("v2 capped at 115")

            if int(v1)<3500:
                v1 = "03500"
                print("v2 capped at 035")
            if int(v1)>10500:
                v1 = "10500"
                print("v2 capped at 105")
            outline = (" " + v1 + " " + v2 + " " + v3 + " " + v4 + " " + v5).encode("ASCII", "ignore")
            self.serialPort.write(outline)
            print(outline)



            # message = self.serialPort.read(5*6+20)
            message = self.serialPort.read(50)
            print(message)

            # print(str(outline)+" "+str(message))
            joints = [0]*5
            for i in range(5):
                joints[i] = int(message[i*6:i*6+6].decode("ASCII"))

            # nub = [0]*5
            # for i in range(5):
            #     nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
            #     if message[i * 4 - 20]==0 :
            #         nub[i] = nub[i]
            #     else:
            #         nub[i] = -nub[i]

            # nub = [0]*5
            # for i in range(5):
            #     nub[i] = message[(i*4-20):(i*4-16)]
            #
            # nub = [int.from_bytes(b, byteorder='big', signed=True) for b in nub]

            nub = [0]*5
            nub[0] = message[30:34]
            nub[1] = message[34:38]
            nub[2] = message[38:42]
            nub[3] = message[42:46]
            nub[4] = message[46:50]
            # print(nub[4])
            nub = [int.from_bytes(b, byteorder='big', signed=True) for b in nub]

            # print(joints)
            # print(nub)

            self.robot.joints = joints
            self.robot.nub = nub

    # def parseLine(self):
    #     if self.serialPort.is_open:
    #         # self.serialPort.write("000-000+000+000+000".encode("ASCII", "ignore"))
    #
    #
    #
    #         v1 = str(abs(int(self.robot.jointTargets[0]))).zfill(3)
    #         v2 = str(abs(int(self.robot.jointTargets[1]))).zfill(3)
    #         v3 = str(abs(int(self.robot.jointTargets[2]))).zfill(3)
    #         v4 = str(abs(int(self.robot.jointTargets[3]))).zfill(3)
    #         v5 = str(abs(int(self.robot.jointTargets[4]))).zfill(3)
    #
    #         if int(v2)>170:
    #             v2 = "170"
    #             print("v2 capped at 170")
    #
    #         if int(v2)<115:
    #             v2 = "115"
    #             print("v2 capped at 115")
    #
    #         if int(v1)<35:
    #             v1 = "035"
    #             print("v2 capped at 035")
    #         if int(v1)>105:
    #             v1 = "105"
    #             print("v2 capped at 105")
    #         outline = (" " + v1 + " " + v2 + " " + v3 + " " + v4 + " " + v5).encode("ASCII", "ignore")
    #         self.serialPort.write(outline)
    #         print(outline)
    #
    #
    #         if self.serialPort.in_waiting < 40:
    #             # print("waiting for message: "+str(self.serialPort.in_waiting))
    #             pass
    #         else:
    #             # jointAngles = self.serialPort.read(20)
    #             # nubValues = self.serialPort.read(20)
    #
    #             message = self.serialPort.read(40)
    #             # print(message)
    #
    #             joints = [0]*5
    #             for i in range(5):
    #                 joints[i] = int(message[i*4:i*4+4].decode("ASCII"))
    #
    #             # nub = [0]*5
    #             # for i in range(5):
    #             #     nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
    #             #     if message[i * 4 - 20]==0 :
    #             #         nub[i] = nub[i]
    #             #     else:
    #             #         nub[i] = -nub[i]
    #
    #             # nub = [0]*5
    #             # for i in range(5):
    #             #     nub[i] = message[(i*4-20):(i*4-16)]
    #             #
    #             # nub = [int.from_bytes(b, byteorder='big', signed=True) for b in nub]
    #
    #             nub = [0]*5
    #             nub[0] = message[20:24]
    #             nub[1] = message[24:28]
    #             nub[2] = message[28:32]
    #             nub[3] = message[32:36]
    #             nub[4] = message[36:40]
    #             print(nub[4])
    #             nub = [int.from_bytes(b, byteorder='big', signed=True) for b in nub]
    #
    #             # print(joints)
    #             # print(nub)
    #
    #             self.robot.joints = joints
    #             self.robot.nub = nub

    # def parseLine(self):
    #     if self.serialPort.is_open:
    #         # self.serialPort.write("000-000+000+000+000".encode("ASCII", "ignore"))
    #
    #
    #
    #         v1 = str(abs(int(self.robot.jointTargets[0]))).zfill(3)
    #         v2 = str(abs(int(self.robot.jointTargets[1]))).zfill(3)
    #         v3 = str(abs(int(self.robot.jointTargets[2]))).zfill(3)
    #         v4 = str(abs(int(self.robot.jointTargets[3]))).zfill(3)
    #         v5 = str(abs(int(self.robot.jointTargets[4]))).zfill(3)
    #
    #         if int(v2)>170:
    #             v2 = "170"
    #             print("v2 capped at 170")
    #
    #         if int(v2)<115:
    #             v2 = "115"
    #             print("v2 capped at 115")
    #
    #         if int(v1)<35:
    #             v1 = "035"
    #             print("v2 capped at 035")
    #         if int(v1)>105:
    #             v1 = "105"
    #             print("v2 capped at 105")
    #         outline = (" " + v1 + " " + v2 + " " + v3 + " " + v4 + " " + v5).encode("ASCII", "ignore")
    #         self.serialPort.write(outline)
    #         print(outline)
    #
    #
    #         if self.serialPort.in_waiting < 40:
    #             # print("waiting for message: "+str(self.serialPort.in_waiting))
    #             pass
    #         else:
    #             # jointAngles = self.serialPort.read(20)
    #             # nubValues = self.serialPort.read(20)
    #
    #             message = self.serialPort.read(40)
    #             # print(message)
    #
    #             joints = [0]*5
    #             for i in range(5):
    #                 joints[i] = int(message[i*4:i*4+4].decode("ASCII"))
    #
    #             nub = [0]*5
    #             for i in range(5):
    #                 nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
    #                 if message[i * 4 - 20]==0 :
    #                     nub[i] = nub[i]
    #                 else:
    #                     nub[i] = -nub[i]
    #
    #             # print(joints)
    #             # print(nub)
    #
    #             self.robot.joints = joints
    #             self.robot.nub = nub

    # def getStatus(self):
    #     if self.serialPort.is_open:
    #         # self.serialPort.write("000-000+000+000+000".encode("ASCII", "ignore"))
    #         v1 = str(abs(int(self.robot.joints[0]))).zfill(3)
    #         v2 = str(abs(int(self.robot.joints[1]))).zfill(3)
    #         v3 = str(abs(int(self.robot.joints[2]))).zfill(3)
    #         v4 = str(abs(int(self.robot.joints[3]))).zfill(3)
    #         v5 = str(abs(int(self.robot.joints[4]))).zfill(3)
    #
    #         self.serialPort.write(("s"+v1+" "+v2+" "+v3+" "+v4+" "+v5).encode("ASCII", "ignore"))
    #         # print((" "+v1+" "+v2+" "+v3+" "+v4+" "+v5).encode("ASCII", "ignore"))
    #
    #
    #         if self.serialPort.in_waiting < 40:
    #             # print("waiting for message: "+str(self.serialPort.in_waiting))
    #             pass
    #         else:
    #             # jointAngles = self.serialPort.read(20)
    #             # nubValues = self.serialPort.read(20)
    #
    #             message = self.serialPort.read(40)
    #             # print(message)
    #
    #             joints = [0]*5
    #             for i in range(5):
    #                 joints[i] = int(message[i*4:i*4+4].decode("ASCII"))
    #
    #             nub = [0]*5
    #             for i in range(5):
    #                 nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
    #                 if message[i * 4 - 20]==0 :
    #                     nub[i] = nub[i]
    #                 else:
    #                     nub[i] = -nub[i]
    #
    #             # print(joints)
    #             # print(nub)
    #
    #             self.robot.joints = joints
    #             self.robot.nub = nub

    # def parseLineTest(self,yeet):
    #     # if self.serialPort.is_open():
    #     #     if self.serialPort.in_waiting < 40:
    #     #         # print("waiting for message: "+str(self.serialPort.in_waiting))
    #     #         pass
    #     #     else:
    #             # jointAngles = self.serialPort.read(20)
    #             # nubValues = self.serialPort.read(20)
    #
    #             # message = self.serialPort.read(40)
    #     message = yeet[0:40]
    #
    #     joints = [0, 0, 0, 0, 0]
    #     for i in range(5):
    #         joints[i] = int("1234")#message[i*4:i*4+4].decode("ASCII"))
    #         # joints[i] = int(message[i*4:i*4+4].decode("ASCII"))
    #
    #     nub = [0, 0, 0, 0, 0]
    #     for i in range(5):
    #                 nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
    #                 if message[i * 4 - 20]==0 :
    #                     nub[i] = nub[i]
    #                 else:
    #                     nub[i] = -nub[i]
    #
    #     print(joints)
    #     print(nub)

    def printPortDebug(self):
        self.serialPort = serial.Serial()
        self.availablePorts = comports()

        for i in self.availablePorts:
            print("~~~~~~~~~~~~~~~~~~~~~~~~~")
            print("Prod: "+str(i.product))
            print("Dev: "+str(i.device))
            print("Desc: "+str(i.description))
            print("Name: "+str(i.name))
            print("Location: "+str(i.location))
            print("Int: "+str(i.interface))
            print("Man: "+str(i.manufacturer))
            print("Prod: "+str(i.product))
        # self.availablePortNames = [i.name for i in self.availablePorts]
        # self.availablePortLocations = [i.location for i in self.availablePorts]
        # self.availablePortDescs = [i.description for i in self.availablePorts]
        # self.availablePortDevs = [i.device for i in self.availablePorts]
        # self.availablePortProds = [i.product for i in self.availablePorts]


class Robot:
    # ####################################
    # ### OG
    # ####################################
    # __TL = 13.
    # __BL = 14.
    # __GL = 2.5
    # __FL = 2.
    # __LL = 4.
    # __T = 21.
    # __HH = 1.
    # __HL = 2.
    # __L0 = 4.
    # __L1 = 4.

    ####################################
    ### 3-30-2019
    ####################################
    # __HL = 0.2   #?
    # __HH = 1.   #?

    __TL = 11.25
    __BL = 12.375
    __GL = 2.48
    __FL = 2.5
    __LL = 3.375
    __T  = 22.  #?
    __HH = 0.0   #?
    __HL = 0.0   #?
    __L0 = 4.   #?
    __L1 = 4.   #?

    def __init__(self):
        self.nub = [0]*5
        self.joints = [50, 150, 000, 000, 000]
        self.jointTargets = [50, 150, 000, 000, 000]
        self.lastTarget = self.joints.copy()

        self.comms = Comms()
        self.comms.registerRobot(self)
        self.calVals = self.nub.copy()
        self.calibrate()
        # self.calibrateR()

        self.lastgood = self.jointTargets.copy()
        time.sleep(1)
        self.calibrate()
        # self.calibrateR()
        self.ts = time.time()

        self.free1 = 0
        self.free2 = 0
        self.free3 = 0

    def fwdKin(self, TH0, TH1, TH2, TH3, TH4):
            #     function[X, Y, Z, THX, THY, THZ] = Forwardplswork(TH0, TH1, TH2, TH3, TH4, self.HH, self.HL)
            #
            #         self.TL = 13;
            #         self.BL = 14;
            #         self.GL = 2;
            #         self.FL = 2;
            #         self.LL = 4;
            #         self.t = 21;
            #
            #         PPY = -GL - self.LL * math.cos(TH4);
            #         PPX = self.LL * math.sin(TH4);
            #
            #         PQY = self.TL * math.sin(TH3);
            #         PQX = self.TL * math.cos(TH3);

        PPY = -self.__GL - self.__LL * math.cos(TH4)
        PPX = self.__LL * math.sin(TH4)

        PQY = self.__TL * math.sin(TH3)
        PQX = self.__TL * math.cos(TH3)


            #
            #         P = sqrt(PPY ** 2 + PPX ** 2);
            #         Q = sqrt((PPY - PQY) ** 2 + (PPX - PQX) ** 2);
            #
            #         ALPHA = acos((Q ** 2 + self.TL ** 2 - P ** 2) / (2 * Q * self.TL));
            #         BETA = acos((Q ** 2 + self.FL ** 2 - self.BL ** 2) / (2 * Q * self.FL));
            #
            #         THK = ALPHA + BETA - pi / 2;
            #

        P = math.sqrt(PPY ** 2 + PPX ** 2)
        Q = math.sqrt((PPY - PQY) ** 2 + (PPX - PQX) ** 2)

        ALPHA = math.acos((Q ** 2 + self.__TL ** 2 - P ** 2) / (2 * Q * self.__TL))
        BETA = math.acos((Q ** 2 + self.__FL ** 2 - self.__BL ** 2) / (2 * Q * self.__FL))

        THK = ALPHA + BETA - math.pi / 2



        # ###################################################################
        # % -THY = TH3 + THK
        # % Z = self.t + 12 * math.sin(TH3) - self.HH * math.cos(TH3 + THK) + self.HL * math.sin(TH3 + THK)
        # ###################################################################

        Z = self.__T + self.__TL * math.sin(TH3) - self.__HH * math.cos(TH3 + THK) + self.__HL * math.sin(TH3 + THK)
        THY = -(TH3 + THK)

        dt = self.__TL * math.cos(TH3) + self.__HH * math.sin(-THY) + self.__HL * math.cos(-THY)

        THZ = TH0 + TH1 + TH2

        X = self.__L0 * math.cos(TH0) + self.__L1 * math.cos(TH0 + TH1) + dt * math.cos(THZ)
        Y = self.__L0 * math.sin(TH0) + self.__L1 * math.sin(TH0 + TH1) + dt * math.sin(THZ)

        THX = 0

        return X, Y, Z, THX, THY, THZ

    def invKin(self,X, Y, Z, THX, THY, THZ):

        TH3 = math.asin((self.__T - Z - self.__HH * math.cos(-THY) + self.__HL * math.sin(-THY)) / (-self.__TL))
        THK = - THY - TH3
        dt = self.__TL * math.cos(TH3) + self.__HH * math.sin(-THY) + self.__HL * math.cos(-THY)
        P2X = X - dt * math.cos(THZ)
        P2Y = Y - dt * math.sin(THZ)
        THR = math.atan2(P2Y, P2X)
        R = math.sqrt(P2X ** 2 + P2Y ** 2)
        GAMMA = math.acos((self.__L0 ** 2 + R ** 2 - self.__L1 ** 2) / (2 * self.__L0 * R))
        # % RIGHT
        TH0 = THR - GAMMA
        TH1 = 2 * GAMMA
        # % % LEFT
        # % TH0 = THR + GAMMA;
        # % TH1 = -2 * GAMMA;
        TH2 = THZ - TH0 - TH1
        PKX = self.__TL * math.cos(TH3) + self.__FL * math.sin(TH3 + THK)
        PKY = self.__TL * math.sin(TH3) - self.__FL * math.cos(TH3 + THK)

        F = math.sqrt(PKX ** 2 + (PKY) ** 2)
        K = math.sqrt(PKX ** 2 + (PKY + self.__GL) ** 2)

        ALPHA = math.acos((self.__GL ** 2 + K ** 2 - F ** 2) / (self.__GL * 2 * K))
        BETA = math.acos((K ** 2 + self.__LL ** 2 - self.__BL ** 2) / (K * self.__LL * 2))
        TH4 = math.pi - ALPHA - BETA

        return TH0, TH1, TH2, TH3, TH4

    def testKin(self):
        TH0, TH1, TH2, TH3, TH4 = 0, math.pi/2, 0, 30*math.pi/180, 40*math.pi/180
        X, Y, Z, THX, THY, THZ = self.fwdKin(TH0, TH1, TH2, TH3, TH4)
        TH02, TH12, TH22, TH32, TH42 = self.invKin( X, Y, Z, THX, THY, THZ)
        X2, Y2, Z2, THX2, THY2, THZ2 = self.fwdKin(TH02, TH12, TH22, TH32, TH42)

        print(TH0, TH1, TH2, TH3, TH4)
        print(TH02, TH12, TH22, TH32, TH42)

        print(X, Y, Z, THX, THY, THZ)
        print(X2, Y2, Z2, THX2, THY2, THZ2)

        # self.drawf(TH0, TH1, TH2, TH3, TH4)

    def fwdKinPTS(self, TH0, TH1, TH2, TH3, TH4):
        PPY = -self.__GL - self.__LL * math.cos(TH4)
        PPX = self.__LL * math.sin(TH4)
        PQY = self.__TL * math.sin(TH3)
        PQX = self.__TL * math.cos(TH3)

        P = math.sqrt(PPY ** 2 + PPX ** 2)
        Q = math.sqrt((PPY - PQY) ** 2 + (PPX - PQX) ** 2)

        ALPHA = math.acos((Q ** 2 + self.__TL ** 2 - P ** 2) / (2 * Q * self.__TL))
        BETA = math.acos((Q ** 2 + self.__FL ** 2 - self.__BL ** 2) / (2 * Q * self.__FL))

        THK = ALPHA + BETA - math.pi / 2

        XPTS = []
        XPTS.append(0)
        XPTS.append(XPTS[0] + self.__L0 * math.cos(TH0))
        XPTS.append(XPTS[1] + self.__L1 * math.cos(TH0 + TH1))
        XPTS.append(XPTS[2] + 0)
        XPTS.append(XPTS[3] + self.__TL * math.cos(TH0 + TH1 + TH2) * math.cos(TH3))
        XPTS.append(XPTS[4] + self.__HH * math.cos(TH0 + TH1 + TH2) * math.sin(TH3 + THK))
        XPTS.append(XPTS[5] + self.__HL * math.cos(TH0 + TH1 + TH2) * math.cos(TH3 + THK))

        YPTS = []
        YPTS.append(0)
        YPTS.append(YPTS[0] + self.__L0 * math.sin(TH0))
        YPTS.append(YPTS[1] + self.__L1 * math.sin(TH0 + TH1))
        YPTS.append(YPTS[2] + 0)
        YPTS.append(YPTS[3] + self.__TL * math.sin(TH0 + TH1 + TH2) * math.cos(TH3))
        YPTS.append(YPTS[4] + self.__HH * math.sin(TH0 + TH1 + TH2) * math.sin(TH3 + THK))
        YPTS.append(YPTS[5] + self.__HL * math.sin(TH0 + TH1 + TH2) * math.cos(TH3 + THK))

        ZPTS = []
        ZPTS.append(0)
        ZPTS.append(ZPTS[0] + 0)
        ZPTS.append(ZPTS[1] + 0)
        ZPTS.append(ZPTS[2] + self.__T)
        ZPTS.append(ZPTS[3] + self.__TL * math.sin(TH3))
        ZPTS.append(ZPTS[4] - self.__HH * math.cos(TH3 + THK))
        ZPTS.append(ZPTS[5] + self.__HL * math.sin(TH3 + THK))
        return [XPTS, YPTS, ZPTS]

    def drawf(self, TH0, TH1, TH2, TH3, TH4):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d', )

        [xs, ys, zs] = self.fwdKinPTS(TH0, TH1, TH2, TH3, TH4)
        self.ax.axis('equal')

        self.ax.plot(xs, ys, zs)
        # print((xs, ys, zs))
        self.fig.show()

    def main(self):
        self.comms.parseLine()
        # print(self.joints)
        print(self.nub)
        time.sleep(.1)

    def calibrate(self):

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLine()
        cal1 = self.nub.copy()
        time.sleep(1)
        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints)+ "    Joints targ: " + str(self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")


        self.comms.parseLine()
        cal2 = self.nub.copy()
        time.sleep(1)
        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLine()
        cal3 = self.nub.copy()

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLine()
        cal4 = self.nub.copy()

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLine()
        cal5 = self.nub.copy()

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.calVals = [(cal1[0] + cal2[0] + cal3[0] + cal4[0] + cal5[0]) / 5,
                        (cal1[1] + cal2[1] + cal3[1] + cal4[1] + cal5[1]) / 5,
                        (cal1[2] + cal2[2] + cal3[2] + cal4[2] + cal5[2]) / 5,
                        (cal1[3] + cal2[3] + cal3[3] + cal4[3] + cal5[3]) / 5,
                        (cal1[4] + cal2[4] + cal3[4] + cal4[4] + cal5[4]) / 5]

    def calibrateR(self):

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLineR()
        cal1 = self.nub.copy()
        time.sleep(1)
        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints)+ "    Joints targ: " + str(self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")


        self.comms.parseLineR()
        cal2 = self.nub.copy()
        time.sleep(1)
        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLineR()
        cal3 = self.nub.copy()

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLineR()
        cal4 = self.nub.copy()

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.comms.parseLineR()
        cal5 = self.nub.copy()

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        time.sleep(1)
        self.calVals = [(cal1[0] + cal2[0] + cal3[0] + cal4[0] + cal5[0]) / 5,
                        (cal1[1] + cal2[1] + cal3[1] + cal4[1] + cal5[1]) / 5,
                        (cal1[2] + cal2[2] + cal3[2] + cal4[2] + cal5[2]) / 5,
                        (cal1[3] + cal2[3] + cal3[3] + cal4[3] + cal5[3]) / 5,
                        (cal1[4] + cal2[4] + cal3[4] + cal4[4] + cal5[4]) / 5]

    def demoZ(self):
        self.comms.parseLine()
        # print(self.joints)
        print(self.nub)
        calz1 = self.nub[1]-self.calVals[1]
        calz2 = self.nub[4]-self.calVals[4]
        torque = calz1-calz2
        force = calz1+calz2

        gainF = .0001
        gainT = .00001

        dZ = gainF*force
        dP = gainT*torque

        # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[1],self.joints[0])
        # self.worldTarg = self.world.copy()
        # self.jointTargets = self.invKin(self.worldTarg)

        self.jointTargets[1] = int(self.joints[1] -dP)
        self.jointTargets[0] = int(self.joints[0] -dZ)

        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES


        time.sleep(.1)

    def inv_kin_closest(self,X, Y, Z, THX, THY, THZ, XTH0, XTH1, XTH2, XTH3, XTH4):

        TH3 = math.asin((self.__T - Z - self.__HH * math.cos(-THY) + self.__HL * math.sin(-THY)) / (-self.__TL))
        THK = - THY - TH3
        dt = self.__TL * math.cos(TH3) + self.__HH * math.sin(-THY) + self.__HL * math.cos(-THY)
        P2X = X - dt * math.cos(THZ)
        P2Y = Y - dt * math.sin(THZ)
        THR = math.atan2(P2Y, P2X)
        R = math.sqrt(P2X ** 2 + P2Y ** 2)
        GAMMA = math.acos((self.__L0 ** 2 + R ** 2 - self.__L1 ** 2) / (2 * self.__L0 * R))

        # RIGHT
        TH0R = THR - GAMMA
        TH1R = 2 * GAMMA

        # LEFT
        TH0L = THR + GAMMA
        TH1L = -2 * GAMMA

        TH2R = THZ - TH0R - TH1R
        TH2L = THZ - TH0L - TH1L

        leftError = (TH0L - XTH0) ** 2 + (TH1L - XTH1) ** 2 + (TH2L - XTH2) ** 2
        rightError = (TH0R - XTH0) ** 2 + (TH1R - XTH1) ** 2 + (TH2R - XTH2) ** 2

        if rightError > leftError:
            TH0 = TH0L
            TH1 = TH1L
            TH2 = TH2L
        else:
            TH0 = TH0R
            TH1 = TH1R
            TH2 = TH2R

        PKX = self.__TL * math.cos(TH3) + self.__FL * math.sin(TH3 + THK)
        PKY = self.__TL * math.sin(TH3) - self.__FL * math.cos(TH3 + THK)

        F = math.sqrt(PKX ** 2 + (PKY) ** 2)
        K = math.sqrt(PKX ** 2 + (PKY + self.__GL) ** 2)

        ALPHA = math.acos((self.__GL ** 2 + K ** 2 - F ** 2) / (self.__GL * 2 * K))
        BETA = math.acos((K ** 2 + self.__LL ** 2 - self.__BL ** 2) / (K * self.__LL * 2))
        TH4 = math.pi - ALPHA - BETA

        return TH0, TH1, TH2, TH3, TH4


    def fwd_planar_partial_kin(self, TH0, TH1):
        P2X = self.__L0 * math.cos(TH0) + self.__L1 * math.cos(TH0 + TH1)
        P2Y = self.__L0 * math.sin(TH0) + self.__L1 * math.sin(TH0 + TH1)

        return P2X, P2Y

    def inv_planar_partial_kin_closest(self,P2X, P2Y, XTH0, XTH1):

        THR = math.atan2(P2Y, P2X)
        R = math.sqrt((P2X ** 2) + (P2Y ** 2))
        GAMMA = math.acos((self.__L0 ** 2 + R ** 2 - self.__L1 ** 2) / (2 * self.__L0 * R))

        # RIGHT
        TH0R = THR - GAMMA
        TH1R = 2 * GAMMA

        # LEFT
        TH0L = THR + GAMMA
        TH1L = -2 * GAMMA

        # leftError = ((TH0L - XTH0 + (2*math.pi)) % (2*math.pi)) ** 2 + ((TH1L - XTH1 + (2*math.pi)) % (2*math.pi)) ** 2
        # rightError = ((TH0R - XTH0 + (2*math.pi)) % (2*math.pi)) ** 2 + ((TH1R - XTH1 + (2*math.pi)) % (2*math.pi)) ** 2

        # leftError = abs((TH0L - XTH0 + (2*math.pi)) % (2*math.pi)) + abs((TH1L - XTH1 + (2*math.pi)) % (2*math.pi))
        # rightError = abs((TH0R - XTH0 + (2*math.pi)) % (2*math.pi)) + abs((TH1R - XTH1 + (2*math.pi)) % (2*math.pi))
        leftError = abs((TH0L - XTH0 + (2*math.pi)) % (2*math.pi))
        if leftError>math.pi:
            leftError = math.pi*2 - leftError
        rightError = abs((TH0R - XTH0 + (2*math.pi)) % (2*math.pi))
        if rightError>math.pi:
            rightError = math.pi*2 - rightError

        # print("TH0L, TH0R, XTH0: " +str([TH0L, TH0R, XTH0])+" TH1L, TH1R, XTH1: "+str( [TH1L, TH1R, XTH1])+" P2X, P2Y: "+str([P2X, P2Y]))

        # if rightError > leftError:
        if True:
            # print("arm left!    LE: " +str(leftError)+" RE: "+str(rightError))
            TH0 = TH0L
            TH1 = TH1L
            # TH2 = TH2L
        else:
            # print("arm right!    LE: " +str(leftError)+" RE: "+str(rightError))
            TH0 = TH0R
            TH1 = TH1R
            # TH2 = TH2R

        valid = R < (self.__L0+self.__L1-0.1)

        return (TH0 + (2*math.pi))%(2*math.pi), (TH1 + (2*math.pi))%(2*math.pi), valid


    def demoZK(self):
        self.comms.parseLine()
        calz1 = self.nub[1]-self.calVals[1]
        calz2 = self.nub[4]-self.calVals[4]
        torque = calz1-calz2
        force = calz1+calz2

        # gainF = 0.000000001
        # gainT = 0.000000001

        # gainF = 0.00035   # .0001 # 0002
        # gainT = 0.000000000
        gainF = 0.0000001   # .0001 # 0002
        gainT = 0.0000000100

        dZ = gainF*force
        dP = gainT*torque

        if abs(dZ) > 3:
            dz = 3*abs(dZ)/dZ

        # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # print("DZ DP: "+ str((dZ,dP)))
        # print("F T: "+ str((force, torque)))
        print("nub: " + str(self.nub)+"    Joints deg: " + str(self.joints))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        try:
                # 189
                # 211
                # 000
                # 000
                # 000

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            groomedJoints[0] = -(groomedJoints[0]-189)
            groomedJoints[1] = -(groomedJoints[1]-211)
            groomedJoints[3] = 30
            groomedJoints[4] = 170
            groomedJoints[2] = 170
            # print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy*math.pi/180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            #WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2],groomedJoints[3],groomedJoints[4],groomedJoints[1],groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))

            #WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg[2] = self.worldTarg[2] - dZ
            self.worldTarg[4] = self.worldTarg[4] - dP

            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4= self.invKin(self.worldTarg[0],self.worldTarg[1],self.worldTarg[2],self.worldTarg[3],self.worldTarg[4],self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy*180/math.pi for xyxyxy in jointTargs]
            jointTargs[0] = -jointTargs[0]+189
            jointTargs[1] = -jointTargs[1]+211
            self.jointTargets = jointTargs.copy()
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)


            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            # time.sleep(1)
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)


    def demoZKF(self):
        i = -1
        while True:
            self.comms.parseLine()
            calz1 = self.nub[1]-self.calVals[1]
            calz2 = self.nub[4]-self.calVals[4]
            torque = calz1-calz2
            force = calz1+calz2

            i = (i+17)%360

            # gainF = 0.000000001
            # gainT = 0.000000001

            dZ = math.sin(i*math.pi/180)*12
            dP = 0
            print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            print("DZ DP: "+ str((dZ,dP)))
            print("F T: "+ str((force, torque)))
            print("nub: " + str(self.nub))
            print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

            try:
                    # 189
                    # 211
                    # 000
                    # 000
                    # 000

                # GROOM JOINTS

                groomedJoints = [189, 211, 000, 180, 000]
                groomedJoints[0] = -(groomedJoints[0]-189)
                groomedJoints[1] = -(groomedJoints[1]-211)
                groomedJoints[3] = 180
                print("Groomed Joints deg: " + str(groomedJoints))

                groomedJoints = [xxxyyy*math.pi/180. for xxxyyy in groomedJoints]
                print("Groomed Joints: "+str(groomedJoints))
                #WORLDSPACE
                X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2],groomedJoints[3],groomedJoints[4],groomedJoints[1],groomedJoints[0])
                self.world = [X, Y, Z, THX, THY, THZ]
                print("World Spa e: "+str(self.world))

                #WORLD TARGETS
                self.worldTarg = self.world.copy()
                self.worldTarg[2] = self.worldTarg[2] - dZ
                self.worldTarg[4] = self.worldTarg[4] - dP

                print("World Target: "+str(self.worldTarg))

                # JOINT TARGETS
                TH0, TH1, TH2, TH3, TH4= self.invKin(self.worldTarg[0],self.worldTarg[1],self.worldTarg[2],self.worldTarg[3],self.worldTarg[4],self.worldTarg[5])
                jointTargs = [TH4, TH3, TH0, TH1, TH2]

                print("Joint Target: "+str(jointTargs))
                # GROOM JOINT TARGETS
                jointTargs = [xyxyxy*180/math.pi for xyxyxy in jointTargs]
                jointTargs[0] = -jointTargs[0]+189
                jointTargs[1] = -jointTargs[1]+211
                self.jointTargets = jointTargs.copy()
                print("Joint Target Groomed: "+str(jointTargs))

                # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
                # self.worldTarg = self.world.copy()
                # self.jointTargets = self.invKin(self.worldTarg)


                # ################################################# #
                # self.jointTargets[1] = int(self.joints[1] -dP)    #
                # self.jointTargets[0] = int(self.joints[0] -dZ)    #
                # ################################################# #
            except:
                print("Kin Broke")
                time.sleep(10)
            # POSITIVE DOWN
            # NUB ARRAY 2ND AND 5TH VALUES
            # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

            time.sleep(.05)
            # SPEED ISSUES
            # time.sleep(.2)


    def demoZKXYZ(self):
        self.comms.parseLine()
        calz1 = self.nub[1]-self.calVals[1]
        calz2 = self.nub[4]-self.calVals[4]
        torque = calz1-calz2
        force = calz1+calz2

        # gainF = 0.000000001
        # gainT = 0.000000001

        # gainF = 0.00035   # .0001 # 0002
        # gainT = 0.000000000
        gainF = 0.01   # .0001 # 0002
        gainT = 0.0000000100

        dZ = gainF*force
        dP = gainT*torque

        if abs(dZ) > .5:
            dz = .5*abs(dZ)/dZ

        # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # print("DZ DP: "+ str((dZ,dP)))
        # print("F T: "+ str((force, torque)))
        print("nub: " + str(self.nub)+"    Joints deg: " + str(self.joints))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        try:
                # 189
                # 211
                # 000
                # 000
                # 000

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            groomedJoints[0] = -(groomedJoints[0]-189)
            groomedJoints[1] = -(groomedJoints[1]-211)
            groomedJoints[3] = 30
            groomedJoints[4] = 170
            groomedJoints[2] = 170
            # print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy*math.pi/180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            #WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2],groomedJoints[3],groomedJoints[4],groomedJoints[1],groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))

            #WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg[2] = self.worldTarg[2] - dZ
            self.worldTarg[4] = self.worldTarg[4] - dP

            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4= self.invKin(self.worldTarg[0],self.worldTarg[1],self.worldTarg[2],self.worldTarg[3],self.worldTarg[4],self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy*180/math.pi for xyxyxy in jointTargs]
            jointTargs[0] = -jointTargs[0]+189
            jointTargs[1] = -jointTargs[1]+211
            self.jointTargets = jointTargs.copy()
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)


            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            # time.sleep(1)
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)

    def demoZPOP(self):
        self.comms.parseLine()
        # print(self.joints)
        print(self.nub)
        calz1 = self.nub[1]-self.calVals[1]
        calz2 = self.nub[4]-self.calVals[4]
        torque = calz1-calz2
        force = calz1+calz2

        gainF = .0002
        gainT = .00000001

        dZ = gainF*force
        dP = gainT*torque
        #
        # if dZ > 0:
        #     dZ = dZ/2

        # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[1],self.joints[0])
        # self.worldTarg = self.world.copy()
        # self.jointTargets = self.invKin(self.worldTarg)

        self.jointTargets[1] = int(self.joints[1] - dZ )
        self.jointTargets[0] = int(self.joints[0] - 0)

        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES


        time.sleep(.1)

    def lastgood327(self):
        # print("Returning to last good position")
        #
        # self.jointTargets = self.
        home = [85, 150, 0, 0, 0]
        print("moving a bit towards home")
        # self.jointTargets = self.lastgood
        calibrated = [(9*a_i+ b_i)/10 for a_i, b_i in zip(self.lastgood,home)]
    #     MAKE THIS CLOSED ROTATION AVERAGE
    #     THIS WILL BREAK THINGS AT SEAMS OF PLANAR MOTION IN ITS CURRENT STATE
        self.jointTargets = calibrated.copy()

    def demoZK327(self):
        self.comms.parseLine()
        calz1 = self.nub[1] - self.calVals[1]
        calz2 = self.nub[4] - self.calVals[4]
        torque = calz1 - calz2
        force = calz1 + calz2

        # gainF = 0.000000001
        # gainT = 0.000000001

        # gainF = 0.00035   # .0001 # 0002
        # gainT = 0.000000000
        # gainF = 0.0000016  # .0001 # 0002
        # gainF = 0.0000021  # .0001 # 0002
        gainF = 0.0000041  # .0001 # 0002 #BEST SO FAR
        # gainF = 0.0000081  # .0001 # 0002
        # gainF = 0.0000061  # .0001 # 0002
        gainT = 0.00000000000  # 100


        dZ = gainF * force
        dP = gainT * torque

        if abs(dZ) > .15:
            dz = .15 * abs(dZ) / dZ

        # dZ = 0

        # if abs(dZ) > .25:
        #     dz = .25 * abs(dZ) / dZ

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("DZ DP: "+ str((dZ,dP)))
        print("F T: "+ str((force, torque)))
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints)+ "    Joints targ: " + str(self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        """nub = [y1, z1, x, y2, z2]"""

        try:

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            groomedJoints[0] = -(groomedJoints[0] - 85)
            groomedJoints[1] = -(groomedJoints[1] - 150)
            # groomedJoints[0] = (groomedJoints[0] - 75)
            # groomedJoints[1] = (groomedJoints[1] - 150)
            groomedJoints[3] = 30
            groomedJoints[4] = 170
            groomedJoints[2] = 170
            print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            # WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],
                                                 groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))
            # self.lastgood = groomedJoints.copy()
            self.lastgood = self.joints.copy()

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg[2] = self.worldTarg[2] + dZ
            self.worldTarg[4] = self.worldTarg[4] + dP
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP

            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                  self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            jointTargs[0] = -jointTargs[0] + 85
            jointTargs[1] = -jointTargs[1] + 150
            jointTargs[2] = 0
            jointTargs[3] = 0
            jointTargs[4] = 0

            self.jointTargets = jointTargs.copy()
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)

            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            self.lastgood327()
            # # time.sleep(1)
            # groomedJoints = self.lastgood.copy()
            # X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
            # self.world = [X, Y, Z, THX, THY, THZ]
            # # print("World Spa e: "+str(self.world))
            #
            # # WORLD TARGETS
            # self.worldTarg = self.world.copy()
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP
            #
            # # print("World Target: "+str(self.worldTarg))
            # try:
            #     # JOINT TARGETS
            #     TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
            #                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            #     jointTargs = [TH4, TH3, TH0, TH1, TH2]
            #
            #     # print("Joint Target: "+str(jointTargs))
            #     # GROOM JOINT TARGETS
            #     jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            #     jointTargs[0] = -jointTargs[0] + 85
            #     jointTargs[1] = -jointTargs[1] + 150
            #     self.jointTargets = jointTargs.copy()
            #     # print("Joint Target Groomed: "+str(jointTargs))
            #
            #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            #     # self.worldTarg = self.world.copy()
            #     # self.jointTargets = self.invKin(self.worldTarg)
            #
            #     # ################################################# #
            #     # self.jointTargets[1] = int(self.joints[1] -dP)    #
            #     # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            #     # ################################################# #
            # except:
            #     print("trying to move OOB")
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        # time.sleep(.08)
        # time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)

    # def demoZK327(self):
    #     self.comms.parseLine()
    #     calz1 = self.nub[1] - self.calVals[1]
    #     calz2 = self.nub[4] - self.calVals[4]
    #     torque = calz1 - calz2
    #     force = calz1 + calz2
    #
    #     # gainF = 0.000000001
    #     # gainT = 0.000000001
    #
    #     # gainF = 0.00035   # .0001 # 0002
    #     # gainT = 0.000000000
    #     gainF = 0.0000016  # .0001 # 0002
    #     gainT = 0.00000000000  # 100
    #
    #     dZ = gainF * force
    #     dP = gainT * torque
    #
    #     if abs(dZ) > .15:
    #         dz = .15 * abs(dZ) / dZ
    #
    #     print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    #     print("DZ DP: "+ str((dZ,dP)))
    #     print("F T: "+ str((force, torque)))
    #     print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints)+ "    Joints targ: " + str(self.jointTargets))
    #     print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    #
    #     try:
    #
    #         # GROOM JOINTS
    #
    #         groomedJoints = self.joints.copy()
    #
    #         groomedJoints[0] = -(groomedJoints[0] - 85)
    #         groomedJoints[1] = -(groomedJoints[1] - 150)
    #         # groomedJoints[0] = (groomedJoints[0] - 75)
    #         # groomedJoints[1] = (groomedJoints[1] - 150)
    #         groomedJoints[3] = 30
    #         groomedJoints[4] = 170
    #         groomedJoints[2] = 170
    #         print("Groomed Joints deg: " + str(groomedJoints))
    #
    #         groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
    #         # print("Groomed Joints: "+str(groomedJoints))
    #         # WORLDSPACE
    #         X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],
    #                                              groomedJoints[0])
    #         self.drawf(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],groomedJoints[0])
    #         self.world = [X, Y, Z, THX, THY, THZ]
    #         # print("World Spa e: "+str(self.world))
    #         self.lastgood = groomedJoints.copy()
    #
    #         # WORLD TARGETS
    #         self.worldTarg = self.world.copy()
    #         self.worldTarg[2] = self.worldTarg[2] + dZ
    #         self.worldTarg[4] = self.worldTarg[4] + dP
    #         # self.worldTarg[2] = self.worldTarg[2] - dZ
    #         # self.worldTarg[4] = self.worldTarg[4] - dP
    #
    #         # print("World Target: "+str(self.worldTarg))
    #
    #         # JOINT TARGETS
    #         TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
    #                                               self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
    #         jointTargs = [TH4, TH3, TH0, TH1, TH2]
    #
    #         # print("Joint Target: "+str(jointTargs))
    #         # GROOM JOINT TARGETS
    #         jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
    #         jointTargs[0] = -jointTargs[0] + 85
    #         jointTargs[1] = -jointTargs[1] + 150
    #         jointTargs[2] = 0
    #         jointTargs[3] = 0
    #         jointTargs[4] = 0
    #
    #         self.jointTargets = jointTargs.copy()
    #         # print("Joint Target Groomed: "+str(jointTargs))
    #
    #         # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
    #         # self.worldTarg = self.world.copy()
    #         # self.jointTargets = self.invKin(self.worldTarg)
    #
    #         # ################################################# #
    #         # self.jointTargets[1] = int(self.joints[1] -dP)    #
    #         # self.jointTargets[0] = int(self.joints[0] -dZ)    #
    #         # ################################################# #
    #     except:
    #         print("Kin Broke")
    #         # # time.sleep(1)
    #         # groomedJoints = self.lastgood.copy()
    #         # X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
    #         # self.world = [X, Y, Z, THX, THY, THZ]
    #         # # print("World Spa e: "+str(self.world))
    #         #
    #         # # WORLD TARGETS
    #         # self.worldTarg = self.world.copy()
    #         # self.worldTarg[2] = self.worldTarg[2] - dZ
    #         # self.worldTarg[4] = self.worldTarg[4] - dP
    #         #
    #         # # print("World Target: "+str(self.worldTarg))
    #         # try:
    #         #     # JOINT TARGETS
    #         #     TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
    #         #                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
    #         #     jointTargs = [TH4, TH3, TH0, TH1, TH2]
    #         #
    #         #     # print("Joint Target: "+str(jointTargs))
    #         #     # GROOM JOINT TARGETS
    #         #     jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
    #         #     jointTargs[0] = -jointTargs[0] + 85
    #         #     jointTargs[1] = -jointTargs[1] + 150
    #         #     self.jointTargets = jointTargs.copy()
    #         #     # print("Joint Target Groomed: "+str(jointTargs))
    #         #
    #         #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
    #         #     # self.worldTarg = self.world.copy()
    #         #     # self.jointTargets = self.invKin(self.worldTarg)
    #         #
    #         #     # ################################################# #
    #         #     # self.jointTargets[1] = int(self.joints[1] -dP)    #
    #         #     # self.jointTargets[0] = int(self.joints[0] -dZ)    #
    #         #     # ################################################# #
    #         # except:
    #         #     print("trying to move OOB")
    #     # POSITIVE DOWN
    #     # NUB ARRAY 2ND AND 5TH VALUES
    #     # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))
    #
    #     # time.sleep(.05)
    #     time.sleep(.05)
    #     # SPEED ISSUES
    #     # time.sleep(.2)

    def demoThurs(self):
        self.comms.parseLine()
        calibrated = [a_i - b_i for a_i, b_i in zip(self.nub, self.calVals)]
        # torque = calz1 - calz2
        # force = calz1 + calz2
        forces = [
            calibrated[0]+calibrated[0], # X ??????????????????????????
            calibrated[0]+calibrated[0], # Y ??????????????????????????????????????
            calibrated[0]+calibrated[0],# Z ??????????????????????????????????????
            0,                              # THX ??????????????????????????????????????
            calibrated[0]-calibrated[0],# THY ??????????????????????????????????????
            calibrated[0]-calibrated[0]# THZ ??????????????????????????????????????
        ]
        gains = [
            .0000001,  # X ??????????????????????????
            .0000001,  # Y ??????????????????????????????????????
            .0000001,  # Z ??????????????????????????????????????
            0.,        # THX ??????????????????????????????????????
            .0000001,  # THY ??????????????????????????????????????
            .0000001   # THZ ??????????????????????????????????????
        ]
        deltas = [a_i * b_i for a_i, b_i in zip(forces, gains)]

        # clampedDeltas =

        # if abs(dZ) > 3:
        #     dz = 3 * abs(dZ) / dZ

        # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # print("DZ DP: "+ str((dZ,dP)))
        # print("F T: "+ str((force, torque)))
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        try:
            # 189
            # 211
            # 000
            # 000
            # 000

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            groomedJoints[0] = -(groomedJoints[0] - 189)
            groomedJoints[1] = -(groomedJoints[1] - 211)
            groomedJoints[3] = 30
            groomedJoints[4] = 170
            groomedJoints[2] = 170
            # print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            # WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))
            self.lastgood = groomedJoints.copy()

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg[2] = self.worldTarg[2] - dZ
            self.worldTarg[4] = self.worldTarg[4] - dP

            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                  self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            jointTargs[0] = -jointTargs[0] + 189
            jointTargs[1] = -jointTargs[1] + 211
            self.jointTargets = jointTargs.copy()
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)

            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            # time.sleep(1)
            groomedJoints = self.lastgood.copy()
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg[2] = self.worldTarg[2] - dZ
            self.worldTarg[4] = self.worldTarg[4] - dP

            # print("World Target: "+str(self.worldTarg))
            try:
                # JOINT TARGETS
                TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                      self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
                jointTargs = [TH4, TH3, TH0, TH1, TH2]

                # print("Joint Target: "+str(jointTargs))
                # GROOM JOINT TARGETS
                jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
                jointTargs[0] = -jointTargs[0] + 189
                jointTargs[1] = -jointTargs[1] + 211
                self.jointTargets = jointTargs.copy()
                # print("Joint Target Groomed: "+str(jointTargs))

                # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
                # self.worldTarg = self.world.copy()
                # self.jointTargets = self.invKin(self.worldTarg)

                # ################################################# #
                # self.jointTargets[1] = int(self.joints[1] -dP)    #
                # self.jointTargets[0] = int(self.joints[0] -dZ)    #
                # ################################################# #
            except:
                print("trying to move OOB")
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)

    # Do this one
    def rotationalAverage(self,a,b,weight_a=1,weight_b=1):
        diff = ((a - b + 180 + 360 + 360) % 360) - 180
        angle = (360 + 360 + b + (diff * weight_a / (weight_a + weight_b))) % 360
        return angle

    def lastgoodPLANAR(self):
        # print("Returning to last good position")
        #
        # self.jointTargets = self.
        home = [85, 150, 180, 180, 180]
        print("moving a bit towards home")
        # self.jointTargets = self.lastgood
        calibrated = [ self.rotationalAverage(a_i,b_i,9,1) for a_i, b_i in zip(self.lastgood,home)]
        self.jointTargets = calibrated.copy()

    def demoPLANAR(self):
        self.comms.parseLine()
        # calz1 = self.nub[1] - self.calVals[1]
        # calz2 = self.nub[4] - self.calVals[4]
        calNub = [a_i - b_i for a_i, b_i in zip(self.nub, self.calVals)]

        # torque = calz1 - calz2
        # force = calz1 + calz2
        """nub = [y1, z1, x, y2, z2]"""
        forces = [calNub[2],
                  calNub[0] + calNub[3],
                  calNub[1] + calNub[4],
                  0,
                  calNub[1] - calNub[4],
                  calNub[0] - calNub[3]]
        # [X, Y, Z, THX, THY, THZ]

        # gains = [0.0000041, 0.0000041, 0.0000041, 0, 0, 0]
        gains = [0, 0, 0, 0, 0, 0]

        deltas = [a_i * b_i for a_i, b_i in zip(gains, forces)]

        # gainF = 0.0000016  # .0001 # 0002
        # gainF = 0.0000021  # .0001 # 0002
        gainF = 0.0000041  # .0001 # 0002 #BEST SO FAR

        deltas = [(.15 * abs(delta) / delta) if abs(delta) > .15 else delta for delta in deltas]

        # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # # print("DZ DP: "+ str((dZ,dP)))
        # # print("F T: "+ str((force, torque)))
        # print("nub: " + str(self.nub) + " deltas: " + str(deltas) + "  Joints deg: " + str(self.joints) + "  Joints targ: " + str(self.jointTargets))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #
        # """nub = [y1, z1, x, y2, z2]"""

        try:

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            jointOffsets = [85, 150, 180, 180, 180]

            # groomedJoints[0] = -(groomedJoints[0] - 85)
            # groomedJoints[1] = -(groomedJoints[1] - 150)
            # # groomedJoints[0] = (groomedJoints[0] - 75)
            # # groomedJoints[1] = (groomedJoints[1] - 150)
            # groomedJoints[3] = 30
            # groomedJoints[4] = 170
            # groomedJoints[2] = 170
            groomedJoints = [-(a_i - b_i) for a_i, b_i in zip(groomedJoints, jointOffsets)]

            # print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            # WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],
                                                 groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))
            # self.lastgood = groomedJoints.copy()
            self.lastgood = self.joints.copy()

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg = self.worldTarg + deltas
            # self.worldTarg = self.worldTarg - deltas
            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                  self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            # jointTargs[0] = -jointTargs[0] + 85
            # jointTargs[1] = -jointTargs[1] + 150
            # jointTargs[2] = 0
            # jointTargs[3] = 0
            # jointTargs[4] = 0
            jointTargs = [((-a_i + b_i) + 720) % 360 for a_i, b_i in zip(jointTargs, jointOffsets)]

            # self.jointTargets = jointTargs.copy()

            print("joints: " + str(self.joints) + "targs: " + str(jointTargs) + "ds: " + str(deltas))
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)

            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            self.lastgoodPLANAR()
            # # time.sleep(1)
            # groomedJoints = self.lastgood.copy()
            # X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
            # self.world = [X, Y, Z, THX, THY, THZ]
            # # print("World Spa e: "+str(self.world))
            #
            # # WORLD TARGETS
            # self.worldTarg = self.world.copy()
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP
            #
            # # print("World Target: "+str(self.worldTarg))
            # try:
            #     # JOINT TARGETS
            #     TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
            #                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            #     jointTargs = [TH4, TH3, TH0, TH1, TH2]
            #
            #     # print("Joint Target: "+str(jointTargs))
            #     # GROOM JOINT TARGETS
            #     jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            #     jointTargs[0] = -jointTargs[0] + 85
            #     jointTargs[1] = -jointTargs[1] + 150
            #     self.jointTargets = jointTargs.copy()
            #     # print("Joint Target Groomed: "+str(jointTargs))
            #
            #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            #     # self.worldTarg = self.world.copy()
            #     # self.jointTargets = self.invKin(self.worldTarg)
            #
            #     # ################################################# #
            #     # self.jointTargets[1] = int(self.joints[1] -dP)    #
            #     # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            #     # ################################################# #
            # except:
            #     print("trying to move OOB")
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        time.sleep(.08)
        # time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)

    def demoPLANAR90DTOTFIX(self):
        self.comms.parseLine()
        # calz1 = self.nub[1] - self.calVals[1]
        # calz2 = self.nub[4] - self.calVals[4]
        calNub = [a_i - b_i for a_i, b_i in zip(self.nub,self.calVals)]

        # torque = calz1 - calz2
        # force = calz1 + calz2
        """nub = [y1, z1, x, y2, z2]"""
        forces = [calNub[2],
                  calNub[0] + calNub[3],
                  calNub[1] + calNub[4],
                  0,
                  calNub[1] - calNub[4],
                  calNub[0] - calNub[3]]
                # [X, Y, Z, THX, THY, THZ]

        # gains = [0.0000041, 0.0000041, 0.0000041, 0, 0, 0]
        # gains = [0, 0.000005, 0, 0, 0, 0]
        gains = [0, 0.0000025, 0, 0, 0, 0]

        deltas = [a_i * b_i for a_i, b_i in zip(gains,forces)]


        # gainF = 0.0000016  # .0001 # 0002
        # gainF = 0.0000021  # .0001 # 0002
        gainF = 0.0000041  # .0001 # 0002 #BEST SO FAR

        # deltas = [(1.5 * abs(delta) / delta) if abs(delta) > 1.5 else delta for delta in deltas]
        deltas = [(.15 * abs(delta) / delta) if abs(delta) > .15 else delta for delta in deltas]


        # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # # print("DZ DP: "+ str((dZ,dP)))
        # # print("F T: "+ str((force, torque)))
        # print("nub: " + str(self.nub) + " deltas: " + str(deltas) + "  Joints deg: " + str(self.joints) + "  Joints targ: " + str(self.jointTargets))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #
        # """nub = [y1, z1, x, y2, z2]"""

        try:

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            jointOffsets = [85, 150, 180, 90, 180]

            # groomedJoints[0] = -(groomedJoints[0] - 85)
            # groomedJoints[1] = -(groomedJoints[1] - 150)
            # # groomedJoints[0] = (groomedJoints[0] - 75)
            # # groomedJoints[1] = (groomedJoints[1] - 150)
            # groomedJoints[3] = 30
            # groomedJoints[4] = 170
            # groomedJoints[2] = 170
            groomedJoints = [-(a_i - b_i) for a_i, b_i in zip(groomedJoints, jointOffsets)]



            # print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            # WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],
                                                 groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))
            # self.lastgood = groomedJoints.copy()
            self.lastgood = self.joints.copy()

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg = [ai + bi for ai, bi in zip(self.worldTarg, deltas)]
            # self.worldTarg = self.worldTarg - deltas
            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                  self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            # jointTargs[0] = -jointTargs[0] + 85
            # jointTargs[1] = -jointTargs[1] + 150
            # jointTargs[2] = 0
            # jointTargs[3] = 0
            # jointTargs[4] = 0
            jointTargs = [((-a_i + b_i)+720)%360 for a_i, b_i in zip(jointTargs, jointOffsets)]

            self.jointTargets = jointTargs.copy()

            print("forces: " + str(forces) + "  joints: " + str(self.joints) + "  targs: " + str(jointTargs) + "  ds: " + str(deltas))
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)

            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            self.lastgoodPLANAR()
            # # time.sleep(1)
            # groomedJoints = self.lastgood.copy()
            # X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
            # self.world = [X, Y, Z, THX, THY, THZ]
            # # print("World Spa e: "+str(self.world))
            #
            # # WORLD TARGETS
            # self.worldTarg = self.world.copy()
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP
            #
            # # print("World Target: "+str(self.worldTarg))
            # try:
            #     # JOINT TARGETS
            #     TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
            #                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            #     jointTargs = [TH4, TH3, TH0, TH1, TH2]
            #
            #     # print("Joint Target: "+str(jointTargs))
            #     # GROOM JOINT TARGETS
            #     jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            #     jointTargs[0] = -jointTargs[0] + 85
            #     jointTargs[1] = -jointTargs[1] + 150
            #     self.jointTargets = jointTargs.copy()
            #     # print("Joint Target Groomed: "+str(jointTargs))
            #
            #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            #     # self.worldTarg = self.world.copy()
            #     # self.jointTargets = self.invKin(self.worldTarg)
            #
            #     # ################################################# #
            #     # self.jointTargets[1] = int(self.joints[1] -dP)    #
            #     # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            #     # ################################################# #
            # except:
            #     print("trying to move OOB")
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        time.sleep(.2)
        # time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)

    def demoPLANAR90DHOTFIXYEET(self):
        self.comms.parseLine()
        # calz1 = self.nub[1] - self.calVals[1]
        # calz2 = self.nub[4] - self.calVals[4]
        calNub = [a_i - b_i for a_i, b_i in zip(self.nub,self.calVals)]

        # torque = calz1 - calz2
        # force = calz1 + calz2
        """nub = [y1, z1, x, y2, z2]"""
        forces = [calNub[2],
                  calNub[0] + calNub[3],
                  calNub[1] + calNub[4],
                  0,
                  calNub[1] - calNub[4],
                  calNub[0] - calNub[3]]
                # [X, Y, Z, THX, THY, THZ]

        # gains = [0.0000041, 0.0000041, 0.0000041, 0, 0, 0]
        # gains = [0, 0.000005, 0, 0, 0, 0]
        gains = [0, 0.000025, 0, 0, 0, 0]
        # gains = [0, 0, 0, 0, 0, 0]

        deltas = [a_i * b_i for a_i, b_i in zip(gains,forces)]

        # deltas = [(1.5 * abs(delta) / delta) if abs(delta) > 1.5 else delta for delta in deltas]
        deltas = [(.15 * abs(delta) / delta) if abs(delta) > .15 else delta for delta in deltas]


        # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        # # print("DZ DP: "+ str((dZ,dP)))
        # # print("F T: "+ str((force, torque)))
        # print("nub: " + str(self.nub) + " deltas: " + str(deltas) + "  Joints deg: " + str(self.joints) + "  Joints targ: " + str(self.jointTargets))
        # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #
        # """nub = [y1, z1, x, y2, z2]"""

        try:

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            jointOffsets = [85, 150, 180, 90, 180]

            # groomedJoints[0] = -(groomedJoints[0] - 85)
            # groomedJoints[1] = -(groomedJoints[1] - 150)
            # # groomedJoints[0] = (groomedJoints[0] - 75)
            # # groomedJoints[1] = (groomedJoints[1] - 150)
            # groomedJoints[3] = 30
            # groomedJoints[4] = 170
            # groomedJoints[2] = 170
            groomedJoints = [-(a_i - b_i) for a_i, b_i in zip(groomedJoints, jointOffsets)]



            # print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            # WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1],
                                                 groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))
            # self.lastgood = groomedJoints.copy()
            self.lastgood = self.joints.copy()

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg = [ai + bi for ai, bi in zip(self.worldTarg, deltas)]
            # self.worldTarg = self.worldTarg - deltas
            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4 = self.inv_kin_closest(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5],
                                                           groomedJoints[2], groomedJoints[3], groomedJoints[4],
                                                           groomedJoints[1], groomedJoints[0])

            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            # jointTargs[0] = -jointTargs[0] + 85
            # jointTargs[1] = -jointTargs[1] + 150
            # jointTargs[2] = 0
            # jointTargs[3] = 0
            # jointTargs[4] = 0
            jointTargs = [((-a_i + b_i)+720)%360 for a_i, b_i in zip(jointTargs, jointOffsets)]

            self.jointTargets = jointTargs.copy()

            print("forces: " + str(forces) + "  joints: " + str(self.joints) + "  targs: " + str(jointTargs) + "  ds: " + str(deltas))
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)

            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            self.lastgoodPLANAR()
            # # time.sleep(1)
            # groomedJoints = self.lastgood.copy()
            # X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
            # self.world = [X, Y, Z, THX, THY, THZ]
            # # print("World Spa e: "+str(self.world))
            #
            # # WORLD TARGETS
            # self.worldTarg = self.world.copy()
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP
            #
            # # print("World Target: "+str(self.worldTarg))
            # try:
            #     # JOINT TARGETS
            #     TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
            #                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            #     jointTargs = [TH4, TH3, TH0, TH1, TH2]
            #
            #     # print("Joint Target: "+str(jointTargs))
            #     # GROOM JOINT TARGETS
            #     jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            #     jointTargs[0] = -jointTargs[0] + 85
            #     jointTargs[1] = -jointTargs[1] + 150
            #     self.jointTargets = jointTargs.copy()
            #     # print("Joint Target Groomed: "+str(jointTargs))
            #
            #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            #     # self.worldTarg = self.world.copy()
            #     # self.jointTargets = self.invKin(self.worldTarg)
            #
            #     # ################################################# #
            #     # self.jointTargets[1] = int(self.joints[1] -dP)    #
            #     # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            #     # ################################################# #
            # except:
            #     print("trying to move OOB")
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        # time.sleep(.2)
        # time.sleep(.08)
        # time.sleep(.1)
        # SPEED ISSUES
        time.sleep(.2)


    def demoPLANAR90DHOTFIXYEET2electricboogaloo(self):
        # time.sleep(.2)
        # time.sleep(1)
        # time.sleep(.1)
        # time.sleep(.2)

        # self.comms.parseLineR()
        self.comms.parseLine()


        calNub = [a_i - b_i for a_i, b_i in zip(self.nub,self.calVals)]
        # calNub = [a_i - b_i for a_i, b_i in zip(self.nub,[0]*len(self.nub))]


        """nub = [y1, z1, x, y2, z2]"""
        forces = [calNub[2],
                  calNub[0] + calNub[3],
                  calNub[1] + calNub[4],
                  0,
                  calNub[1] - calNub[4],
                  calNub[0] - calNub[3]]
                # [X, Y, Z, THX, THY, THZ]

        # gains = [0, 0.0000025, 0, 0, 0, 0]
        # gains = [0, 0.000005, 0, 0, 0, 0]
        # gains = [0, 0, 0, 0, 0, 0]
        # gains = [0.000005, 0, 0, 0, 0, 0]

        gains = [0.000005, 0.000005, 0, 0, 0, 0]

        deltas = [a_i * b_i for a_i, b_i in zip(gains,forces)]

        # deltas = [(1.5 * abs(delta) / delta) if abs(delta) > 1.5 else delta for delta in deltas]
        # deltas = [(.15 * abs(delta) / delta) if abs(delta) > .15 else delta for delta in deltas]
        # deltas = [(.05 * abs(delta) / delta) if abs(delta) > .05 else delta for delta in deltas]
        deltas = [(.1 * abs(delta) / delta) if abs(delta) > .1 else delta for delta in deltas]

        # deltas = [.005,0,0,0,0,0]

        try:

            # GROOM JOINTS

            # groomedJoints = self.joints.copy()
            groomedJoints = self.lastTarget.copy()
            jointOffsets = [85, 150, 0, -90, 0]

            groomedJoints[0] = 0
            groomedJoints[1] = 0
            groomedJoints[2] = groomedJoints[2]
            groomedJoints[3] = -((groomedJoints[3]+90)%360)
            groomedJoints[4] = 0
            groomedJoints = [deg * math.pi / 180 for deg in groomedJoints]

            P2X, P2Y = self.fwd_planar_partial_kin(groomedJoints[2],groomedJoints[3])
            P2X = P2X + deltas[0]
            P2Y = P2Y + deltas[1]
            TH0, TH1, valid = self.inv_planar_partial_kin_closest(P2X,P2Y,groomedJoints[2],groomedJoints[3])

            abso = 180/math.pi*math.atan2(P2Y + deltas[1],P2X + deltas[0])
            rela = 180/math.pi*math.atan2(deltas[1],deltas[0])


            jointTargs = [0, 0, TH0, TH1, 0]
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]

            jointTargs[0] = 85
            jointTargs[1] = 150
            jointTargs[2] = jointTargs[2]
            jointTargs[3] = (((-jointTargs[3])-90)+720)%360
            jointTargs[4] = 0

            print("attempted move: " + str([P2X,P2Y]) + " rela: " + str(rela) +" abs: " + str(abso))

            if valid:
                self.jointTargets = jointTargs.copy()
            else:
                print("invalid position attempted")
            fs = [ "%08.0f"%f for f in forces ]
            # print("forces: " + str(fs).strip("'") + "  joints: " + str(self.joints) + "  targs: " + str(jointTargs) + "  ds: " + str(deltas))
            # print("forces: " + str(fs).strip("'") + " nub: " + str([f'{n:15}' for n in calNub]) + "  joints: " + str(self.joints) + "  targs: " + str(
            #     jointTargs) + "  timestamp: " + str(time.time()-self.ts) + "  ds: " + str(deltas))

            # print("nub: "+str([ "%08.0f"%f for f in calNub]))

            # print("nub: " + str([f'{n:15}' for n in calNub]))
            self.lastTarget = self.jointTargets.copy()


        except:
            print("Kin Broke")
            print("joints: " + str(self.joints) + "  targs: " + str(self.jointTargets))

            # self.lastgoodPLANAR()


    def demoPLANAR90DHOTFIXYEET2electricboogalooTWEKRD(self):
        # time.sleep(.2)
        # time.sleep(1)
        # time.sleep(.1)
        # time.sleep(.2)

        # self.comms.parseLineR()
        self.comms.parseLine()


        calNub = [a_i - b_i for a_i, b_i in zip(self.nub,self.calVals)]
        # calNub = [a_i - b_i for a_i, b_i in zip(self.nub,[0]*len(self.nub))]


        """nub = [y1, z1, x, y2, z2]"""
        forces = [calNub[2],
                  calNub[0] + calNub[3],
                  calNub[1] + calNub[4],
                  0,
                  calNub[1] - calNub[4],
                  calNub[0] - calNub[3]]
                # [X, Y, Z, THX, THY, THZ]
        # forces = [0 if abs(force) < 20 else force for force in forces[0:3]] +[0 if abs(torque) < 10000 else torque for torque in forces[3:6]]

        # gains = [0, 0.0000025, 0, 0, 0, 0]
        # gains = [0, 0.000005, 0, 0, 0, 0]
        # gains = [0, 0, 0, 0, 0, 0]
        # gains = [0.000005, 0, 0, 0, 0, 0]

        # gains = [0.000005, 0.000005, 0, 0, 0, 0]
        # gains = [0.00000, 0.00000, 0, 0, 0, 0.000005]
        # gains = [0.000005, 0.000005, 0, 0, 0, 0.000005]


        gains = [0.000005, 0.0000025, 0, 0, 0, 0.000005]
        # gains = [0, 0.0000025, 0, 0, 0, 0.0000025]

        deltas = [a_i * b_i for a_i, b_i in zip(gains,forces)]

        # deltas = [(1.5 * abs(delta) / delta) if abs(delta) > 1.5 else delta for delta in deltas]
        # deltas = [(.15 * abs(delta) / delta) if abs(delta) > .15 else delta for delta in deltas]
        # deltas = [(.05 * abs(delta) / delta) if abs(delta) > .05 else delta for delta in deltas]
        # deltas = [(.1 * abs(delta) / delta) if abs(delta) > .1 else delta for delta in deltas]
        deltas = [(.1 * abs(delta) / delta) if abs(delta) > .1 else delta for delta in deltas[0:3]] +[(4 * abs(delta) / delta) if abs(delta) > 4 else delta for delta in deltas[3:6]]

        # deltas = [.005,0,0,0,0,0]

        try:

            # GROOM JOINTS

            # groomedJoints = self.joints.copy()
            groomedJoints = self.lastTarget.copy()
            jointOffsets = [85, 150, 0, -90, 0]

            groomedJoints[0] = 0
            groomedJoints[1] = 0
            groomedJoints[2] = groomedJoints[2]
            groomedJoints[3] = -((groomedJoints[3]+90)%360)
            # groomedJoints[4] = 0
            groomedJoints[4] = groomedJoints[4]

            # TH2 = groomedJoints[4] - (groomedJoints[2]-groomedJoints[3])

            # print("hi1")
            groomedJoints = [deg * math.pi / 180 for deg in groomedJoints]

            P2X, P2Y = self.fwd_planar_partial_kin(groomedJoints[2],groomedJoints[3])
            P2X = P2X + deltas[0]
            P2Y = P2Y + deltas[1]
            TH0, TH1, valid = self.inv_planar_partial_kin_closest(P2X,P2Y,groomedJoints[2],groomedJoints[3])
            # print("TH2, deltas[5],TH2 + deltas[5]: "+str([TH2, deltas[5],TH2 + deltas[5]]))
            # print("hi2")

            # abso = 180/math.pi*math.atan2(P2Y + deltas[1],P2X + deltas[0])
            # rela = 180/math.pi*math.atan2(deltas[1],deltas[0])


            jointTargs = [0, 0, TH0, TH1, 0]
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]

            # TH2 = ((TH2 + deltas[5]+(jointTargs[2]-jointTargs[3])) + 720) % 360

            self.free1 = (self.free1 + deltas[5] + 360)

            jointTargs[0] = 85
            jointTargs[1] = 150
            jointTargs[2] = jointTargs[2]
            jointTargs[3] = (((-jointTargs[3])-90)+720)%360
            # jointTargs[4] = 0
            # jointTargs[4] = ((jointTargs[2] - jointTargs[3]) + 720) % 360
            jointTargs[4] = ((jointTargs[2] - jointTargs[3]) + self.free1 + 720) % 360
            # jointTargs[4] = TH2

            # print("attempted move: " + str([P2X,P2Y]) + " rela: " + str(rela) +" abs: " + str(abso))

            # print("hi3")


            if valid:
                self.jointTargets = jointTargs.copy()
            else:
                print("invalid position attempted")
                self.jointTargets[4] = jointTargs[4]
            # print("hi4")
            fs = [ "%08.0f"%f for f in forces ]
            # print("forces: " + str(fs).strip("'") + "  joints: " + str(self.joints) + "  targs: " + str(jointTargs) + "  ds: " + str(deltas))
            print("forces: " + str(fs).strip("'") + " nub: " + str([f'{n:15}' for n in calNub]) + "  joints: " + str(self.joints) + "  targs: " + str(
                jointTargs) + "  timestamp: " + str(time.time()-self.ts) + "  ds: " + str(deltas))
            # print("Y: "+fs[2])
            # print("nub: "+str([ "%08.0f"%f for f in calNub]))

            # print("nub: " + str([f'{n:15}' for n in calNub]))
            self.lastTarget = self.jointTargets.copy()
            # print("hi5")

        except Exception as e:
            print("Kin Broke")
            print(e)
            print("joints: " + str(self.joints) + "  targs: " + str(self.jointTargets))

            # self.lastgoodPLANAR()

    def demoPLANAR90DHOTFIXYEET2electricboogalooTWEKRDthe3rdDimensionMuthafuckaaa(self):
        # time.sleep(.2)
        # time.sleep(1)
        # time.sleep(.1)
        # time.sleep(.2)

        # self.comms.parseLineR()
        self.comms.parseLine()


        calNub = [a_i - b_i for a_i, b_i in zip(self.nub,self.calVals)]
        # calNub = [a_i - b_i for a_i, b_i in zip(self.nub,[0]*len(self.nub))]


        """nub = [y1, z1, x, y2, z2]"""
        forces = [calNub[2],
                  calNub[0] + calNub[3],
                  calNub[1] + calNub[4],
                  0,
                  calNub[1] - calNub[4],
                  calNub[0] - calNub[3]]
                # [X, Y, Z, THX, THY, THZ]
        # forces = [0 if abs(force) < 20 else force for force in forces[0:3]] +[0 if abs(torque) < 10000 else torque for torque in forces[3:6]]

        # gains = [0, 0.0000025, 0, 0, 0, 0]
        # gains = [0, 0.000005, 0, 0, 0, 0]
        # gains = [0, 0, 0, 0, 0, 0]
        # gains = [0.000005, 0, 0, 0, 0, 0]

        # gains = [0.000005, 0.000005, 0, 0, 0, 0]
        # gains = [0.00000, 0.00000, 0, 0, 0, 0.000005]
        # gains = [0.000005, 0.000005, 0, 0, 0, 0.000005]


        # gains = [0.000005, 0.0000025, 0, 0, 0, 0.000005]
        # gains = [0, 0.0000025, 0, 0, 0, 0.0000025]


        # gains = [0.00000, 0.00000, 0.0000025, 0, 0.0000007, 0.00000]
        # gains = [0.000005, 0.0000025, 0, 0, 0, 0.000005]
        # gains = [0.000005, 0.0000025, 0, 0, 0, 0.000005]
        # gains = [0.000005, 0.0000025, 0, 0, 0, 0.000005]

        # gains = [0.000005, 0.0000025, 0.0000025, 0, 0.0000007, 0.000005]
        #
        # gains = [0.000004, 0.0000025, 0.0000035, 0, 0.0000007, 0.00001]
        # gains = [0.000004, 0.0000025, 0.00000, 0, 0.000000, 0.0000]
        # gains = [0.00000035, 0.00000025, 0.000004, 0, 0.0000003, 0.000005]
        # gains = [0.0000035, 0.0000025, 0.000004, 0, 0.000007, 0.000005] # last good
        # gains = [0.0000035, 0.0000025, 0.00000, 0, 0.0000, 0.000005]
        # gains = [0.0000035, 0.0000025, 0.00000, 0, 0.0000, 0.000000]
        # gains = [0.00000, 0.00000, 0.000025, 0, -0.000037, 0.000000]

        # gains = [0.00000, 0.0000025, 0.00000, 0, 0.0000, 0.000000]
        # ############################################
        # gains = [-0.0000008, -0.000001, 0.000008, 0, -0.0000005, 0.000005]
        # gains = [0.00000035, 0.00000025, 0.00000, 0, 0.0000, 0.0000005]
        # FULL BEST WORKING #######################################
        # gains = [-0.0000008, -0.000001, 0.000008, 0, -0.0000005, 0.000005]
        # gains = [-0.0000008, -0.000001, 0.000008, 0, -0.0000005, 0.000005]
        # gains = [-0.0000008, -0.000001, 0.000008, 0, -0.0000005, 0.000005]
        # gains = [-0.0000008, -0.000001, 0.000008, 0, -0.0000005, 0.000005]

        # gains = [-0.000002, -0.000001, 0.000008, 0, -0.000000, 0.00000]
        gains = [-0.0000025, -0.000001, 0.00000, 0, -0.000000, 0.00000]
        # gains = [-0.00000, -0.00000, 0.00000, 0, -0.000000, 0.00000]


        # gains = [-0.000000, -0.00000, 0.00000, 0, -0.00000, 0.0000085]



        deltas = [a_i * b_i for a_i, b_i in zip(gains,forces)]

        # deltas = [(1.5 * abs(delta) / delta) if abs(delta) > 1.5 else delta for delta in deltas]
        # deltas = [(.15 * abs(delta) / delta) if abs(delta) > .15 else delta for delta in deltas]
        # deltas = [(.05 * abs(delta) / delta) if abs(delta) > .05 else delta for delta in deltas]
        # deltas = [(.1 * abs(delta) / delta) if abs(delta) > .1 else delta for delta in deltas]
        deltas = [(.2 * abs(delta) / delta) if abs(delta) > .2 else delta for delta in deltas[0:3]] +[(4 * abs(delta) / delta) if abs(delta) > 4 else delta for delta in deltas[3:6]]

        # deltas = [.005,0,0,0,0,0]

        try:

            # GROOM JOINTS

            # groomedJoints = self.joints.copy()
            groomedJoints = self.lastTarget.copy()
            jointOffsets = [85, 150, 0, -90, 0]

            groomedJoints[0] = (groomedJoints[0] - 85)
            groomedJoints[1] = (groomedJoints[1] - 150)
            groomedJoints[2] = groomedJoints[2]
            groomedJoints[3] = -((groomedJoints[3]+90)%360)
            # groomedJoints[4] = 0
            groomedJoints[4] = groomedJoints[4]

            # TH2 = groomedJoints[4] - (groomedJoints[2]-groomedJoints[3])

            # print("hi1")
            groomedJoints = [deg * math.pi / 180 for deg in groomedJoints]
            TH0 = groomedJoints[2]
            TH1 = groomedJoints[3]
            try:
                P2X, P2Y = self.fwd_planar_partial_kin(groomedJoints[2],groomedJoints[3])
                P2X = P2X + deltas[0]
                P2Y = P2Y + deltas[1]
                TH0, TH1, valid = self.inv_planar_partial_kin_closest(P2X,P2Y,groomedJoints[2],groomedJoints[3])
            except Exception as e:
                print(e)
                valid = False
                print("planar kin dead")
            # print("TH2, deltas[5],TH2 + deltas[5]: "+str([TH2, deltas[5],TH2 + deltas[5]]))
            # print("hi2")
            TH3 = groomedJoints[1]
            TH4 = groomedJoints[0]
            try:
                xxx, yyy, zzz, txxx, tyyy, tzzz = self.fwdKin(1.1*math.pi,1.1*math.pi,1.1*math.pi, groomedJoints[1], groomedJoints[0])
                # print("hi2.1")
                zzz = zzz-deltas[2]
                tyyy = tyyy+deltas[4]
                trash, trash, trash, TH3, TH4 = self.invKin(xxx, yyy, zzz, txxx, tyyy, tzzz)
                # trash, trash, trash, TH3, TH4 = self.invKin(xxx, yyy, 21, txxx, tyyy, tzzz)

                # print("hi2.3")

            except Exception as e:
                print(e)
                print("zeds ded")
            except:
                print("zeds ded")

            # print("hi2.99")
            # abso = 180/math.pi*math.atan2(P2Y + deltas[1],P2X + deltas[0])
            # rela = 180/math.pi*math.atan2(deltas[1],deltas[0])


            # jointTargs = [0, 0, TH0, TH1, 0]
            jointTargs = [TH4, TH3, TH0, TH1, 0]
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]

            # TH2 = ((TH2 + deltas[5]+(jointTargs[2]-jointTargs[3])) + 720) % 360

            self.free1 = (self.free1 + deltas[5] + 360)

            jointTargs[0] = jointTargs[0]+85
            jointTargs[1] = jointTargs[1]+150
            jointTargs[2] = jointTargs[2]
            jointTargs[3] = (((-jointTargs[3])-90)+720)%360
            # jointTargs[4] = 0
            # jointTargs[4] = ((jointTargs[2] - jointTargs[3]) + 720) % 360
            jointTargs[4] = ((jointTargs[2] - jointTargs[3]) + self.free1 + 720) % 360
            # jointTargs[4] = TH2

            if jointTargs[0] < 35: jointTargs[0] = 35
            if jointTargs[0] > 105: jointTargs[0] = 105
            if jointTargs[1] < 115: jointTargs[1] = 115
            if jointTargs[1] > 170: jointTargs[1] = 170


            # print("attempted move: " + str([P2X,P2Y]) + " rela: " + str(rela) +" abs: " + str(abso))

            # print("hi3")


            if valid:
                self.jointTargets = jointTargs.copy()
            else:
                print("invalid position attempted")
                self.jointTargets[4] = jointTargs[4]
            # print("hi4")
            fs = [ "%08.0f"%f for f in forces ]
            # print("forces: " + str(fs).strip("'") + "  joints: " + str(self.joints) + "  targs: " + str(jointTargs) + "  ds: " + str(deltas))
            # print("forces: " + str(fs).strip("'") + " nub: " + str([f'{n:15}' for n in calNub]) + "  joints: " + str(self.joints) + "  targs: " + str(
            #     jointTargs) + "  timestamp: " + str(time.time()-self.ts) + "  ds: " + str(deltas))

            # print("Y: "+fs[2])
            # print("nub: "+str([ "%08.0f"%f for f in calNub]))

            # print("nub: " + str([f'{n:15}' for n in calNub]))
            self.lastTarget = self.jointTargets.copy()
            # print("hi5")

        except Exception as e:
            print("Kin Broke")
            print(e)
            print("joints: " + str(self.joints) + "  targs: " + str(self.jointTargets))

            # self.lastgoodPLANAR()

    def demoZK327torque(self):
        self.comms.parseLine()
        calz1 = self.nub[1] - self.calVals[1]
        calz2 = self.nub[4] - self.calVals[4]
        torque = calz1 - calz2
        force = calz1 + calz2

        # gainF = 0.000000001
        # gainT = 0.000000001

        # gainF = 0.00035   # .0001 # 0002
        # gainT = 0.000000000
        # gainF = 0.0000016  # .0001 # 0002
        # gainF = 0.0000021  # .0001 # 0002
        gainF = 0.0000041  # .0001 # 0002 #BEST SO FAR
        # gainF = 0.0000081  # .0001 # 0002
        # gainF = 0.0000061  # .0001 # 0002
        gainT = 0.00000000000  # 100

        dZ = gainF * force
        dP = gainT * torque

        if abs(dZ) > .15:
            dz = .15 * abs(dZ) / dZ

        # dZ = 0

        # if abs(dZ) > .25:
        #     dz = .25 * abs(dZ) / dZ

        print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("DZ DP: " + str((dZ, dP)))
        print("F T: " + str((force, torque)))
        print("nub: " + str(self.nub) + "    Joints deg: " + str(self.joints) + "    Joints targ: " + str(
            self.jointTargets))
        print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

        """nub = [y1, z1, x, y2, z2]"""

        try:

            # GROOM JOINTS

            groomedJoints = self.joints.copy()

            groomedJoints[0] = -(groomedJoints[0] - 85)
            groomedJoints[1] = -(groomedJoints[1] - 150)
            # groomedJoints[0] = (groomedJoints[0] - 75)
            # groomedJoints[1] = (groomedJoints[1] - 150)
            groomedJoints[3] = 30
            groomedJoints[4] = 170
            groomedJoints[2] = 170
            print("Groomed Joints deg: " + str(groomedJoints))

            groomedJoints = [xxxyyy * math.pi / 180. for xxxyyy in groomedJoints]
            # print("Groomed Joints: "+str(groomedJoints))
            # WORLDSPACE
            X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4],
                                                 groomedJoints[1],
                                                 groomedJoints[0])
            self.world = [X, Y, Z, THX, THY, THZ]
            # print("World Spa e: "+str(self.world))
            # self.lastgood = groomedJoints.copy()
            self.lastgood = self.joints.copy()

            # WORLD TARGETS
            self.worldTarg = self.world.copy()
            self.worldTarg[2] = self.worldTarg[2] + dZ
            self.worldTarg[4] = self.worldTarg[4] + dP
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP

            # print("World Target: "+str(self.worldTarg))

            # JOINT TARGETS
            TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
                                                  self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            jointTargs = [TH4, TH3, TH0, TH1, TH2]

            # print("Joint Target: "+str(jointTargs))
            # GROOM JOINT TARGETS
            jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            jointTargs[0] = -jointTargs[0] + 85
            jointTargs[1] = -jointTargs[1] + 150
            jointTargs[2] = 0
            jointTargs[3] = 0
            jointTargs[4] = 0

            self.jointTargets = jointTargs.copy()
            # print("Joint Target Groomed: "+str(jointTargs))

            # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            # self.worldTarg = self.world.copy()
            # self.jointTargets = self.invKin(self.worldTarg)

            # ################################################# #
            # self.jointTargets[1] = int(self.joints[1] -dP)    #
            # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            # ################################################# #
        except:
            print("Kin Broke")
            self.lastgood327()
            # # time.sleep(1)
            # groomedJoints = self.lastgood.copy()
            # X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2], groomedJoints[3], groomedJoints[4], groomedJoints[1], groomedJoints[0])
            # self.world = [X, Y, Z, THX, THY, THZ]
            # # print("World Spa e: "+str(self.world))
            #
            # # WORLD TARGETS
            # self.worldTarg = self.world.copy()
            # self.worldTarg[2] = self.worldTarg[2] - dZ
            # self.worldTarg[4] = self.worldTarg[4] - dP
            #
            # # print("World Target: "+str(self.worldTarg))
            # try:
            #     # JOINT TARGETS
            #     TH0, TH1, TH2, TH3, TH4 = self.invKin(self.worldTarg[0], self.worldTarg[1], self.worldTarg[2],
            #                                           self.worldTarg[3], self.worldTarg[4], self.worldTarg[5])
            #     jointTargs = [TH4, TH3, TH0, TH1, TH2]
            #
            #     # print("Joint Target: "+str(jointTargs))
            #     # GROOM JOINT TARGETS
            #     jointTargs = [xyxyxy * 180 / math.pi for xyxyxy in jointTargs]
            #     jointTargs[0] = -jointTargs[0] + 85
            #     jointTargs[1] = -jointTargs[1] + 150
            #     self.jointTargets = jointTargs.copy()
            #     # print("Joint Target Groomed: "+str(jointTargs))
            #
            #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
            #     # self.worldTarg = self.world.copy()
            #     # self.jointTargets = self.invKin(self.worldTarg)
            #
            #     # ################################################# #
            #     # self.jointTargets[1] = int(self.joints[1] -dP)    #
            #     # self.jointTargets[0] = int(self.joints[0] -dZ)    #
            #     # ################################################# #
            # except:
            #     print("trying to move OOB")
        # POSITIVE DOWN
        # NUB ARRAY 2ND AND 5TH VALUES
        # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))

        # time.sleep(.05)
        # time.sleep(.08)
        # time.sleep(.1)
        # SPEED ISSUES
        # time.sleep(.2)



if __name__ == '__main__':
    r = Robot()
    # r.testKin()
    r.jointTargets = [85, 150, 00, 0, 0]
    last_time = time.clock()
    try:
        while True:
            # print("Exec time: "+str(time.clock()-last_time)+"s")
            # print('hoi')
            # r.main()
            # r.demoZ()
            # r.demoZK()
            # r.demoZKF()
            # r.demoZKXYZ()
            # r.demoZPOP()
            # r.demoZK327()
            # r.comms.getStatus()
            # print(r.nub)
            # r.demoPLANAR()
            # r.demoPLANAR90DTOTFIX()
            # r.demoPLANAR90DHOTFIXYEET()

            # r.demoPLANAR90DHOTFIXYEET2electricboogaloo()
            # r.demoZK327torque()
            # r.demoPLANAR90DHOTFIXYEET2electricboogalooTWEKRD()
            r.demoPLANAR90DHOTFIXYEET2electricboogalooTWEKRDthe3rdDimensionMuthafuckaaa()
            # print("Exec time: "+str(time.clock()-last_time)+"s")
            time.sleep(.1)
            last_time = time.clock()
    except KeyboardInterrupt as e:
        print(e)
        r.comms.serialPort.close()
        print("peace out girl scout")
