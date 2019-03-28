import math
import serial
import time
from serial.tools.list_ports import comports

class KinematicsException(Exception):
    pass
class UpperLinkForwardError(Exception):
    pass
class UpperLinkInverseError(Exception):
    pass
class LowerLinkInverseError(Exception):
    pass


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
            if i.device == "COM12" or i.device == "COM14" or i.device == "COM16":
                # self.serialPort.setPort(i.device)
                print(i)
                self.serialPort = serial.Serial(i.device)
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
            # self.serialPort.write("000-000+000+000+000".encode("ASCII", "ignore"))
            v1 = str(abs(int(self.robot.jointTargets[0]))).zfill(3)
            v2 = str(abs(int(self.robot.jointTargets[1]))).zfill(3)
            v3 = str(abs(int(self.robot.jointTargets[2]))).zfill(3)
            v4 = str(abs(int(self.robot.jointTargets[3]))).zfill(3)
            v5 = str(abs(int(self.robot.jointTargets[4]))).zfill(3)

            self.serialPort.write((" "+v1+" "+v2+" "+v3+" "+v4+" "+v5).encode("ASCII", "ignore"))
            # print((" "+v1+" "+v2+" "+v3+" "+v4+" "+v5).encode("ASCII", "ignore"))


            if self.serialPort.in_waiting < 40:
                # print("waiting for message: "+str(self.serialPort.in_waiting))
                pass
            else:
                # jointAngles = self.serialPort.read(20)
                # nubValues = self.serialPort.read(20)

                message = self.serialPort.read(40)
                # print(message)

                joints = [0]*5
                for i in range(5):
                    joints[i] = int(message[i*4:i*4+4].decode("ASCII"))

                nub = [0]*5
                for i in range(5):
                    nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
                    if message[i * 4 - 20]==0 :
                        nub[i] = nub[i]
                    else:
                        nub[i] = -nub[i]

                # print(joints)
                # print(nub)

                self.robot.joints = joints
                self.robot.nub = nub

    def parseLineTest(self,yeet):
        # if self.serialPort.is_open():
        #     if self.serialPort.in_waiting < 40:
        #         # print("waiting for message: "+str(self.serialPort.in_waiting))
        #         pass
        #     else:
                # jointAngles = self.serialPort.read(20)
                # nubValues = self.serialPort.read(20)

                # message = self.serialPort.read(40)
        message = yeet[0:40]

        joints = [0, 0, 0, 0, 0]
        for i in range(5):
            joints[i] = int("1234")#message[i*4:i*4+4].decode("ASCII"))
            # joints[i] = int(message[i*4:i*4+4].decode("ASCII"))

        nub = [0, 0, 0, 0, 0]
        for i in range(5):
                    nub[i] = (message[i*4-19]<<16) + (message[i*4-18]<<8) + message[i*4-17]
                    if message[i * 4 - 20]==0 :
                        nub[i] = nub[i]
                    else:
                        nub[i] = -nub[i]

        print(joints)
        print(nub)

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
    __TL = 13.
    __BL = 14.
    __GL = 2.5
    __FL = 2.
    __LL = 4.
    __T = 21.
    __HH = 1.
    __HL = 2.
    __L0 = 4.
    __L1 = 4.


    def __init__(self):
        self.nub = [0]*5
        self.joints = [189, 211, 000, 000, 000]
        self.jointTargets = [189, 211, 000, 000, 000]

        self.comms = Comms()
        self.comms.registerRobot(self)
        self.calVals = self.nub.copy()

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

        try:
            ALPHA = math.acos((Q ** 2 + self.__TL ** 2 - P ** 2) / (2 * Q * self.__TL))
            BETA = math.acos((Q ** 2 + self.__FL ** 2 - self.__BL ** 2) / (2 * Q * self.__FL))
        except:
            raise UpperLinkForwardError("Upper links in an unsolvable config(Check TH3 and TH4)")
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
        try:
            TH3 = math.asin((self.__T - Z - self.__HH * math.cos(-THY) + self.__HL * math.sin(-THY)) / (-self.__TL))
        except:
            raise UpperLinkInverseError("could not solve for TH3 (Check Z and THY):")
        try:
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

    # def main(self):
    #     self.comms.parseLine()
    #     # print(self.joints)
    #     print(self.nub)
    #     time.sleep(.1)
    #
    # def calibrate(self):
    #     self.comms.parseLine()
    #     self.calVals = self.nub.copy()
    #     self.comms.parseLine()
    #     self.calVals = self.nub.copy()
    #
    #
    # def demoZ(self):
    #     self.comms.parseLine()
    #     # print(self.joints)
    #     print(self.nub)
    #     calz1 = self.nub[1]-self.calVals[1]
    #     calz2 = self.nub[4]-self.calVals[4]
    #     torque = calz1-calz2
    #     force = calz1+calz2
    #
    #     gainF = .0001
    #     gainT = .001
    #
    #     dZ = gainF*force
    #     dP = gainT*torque
    #
    #     # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[1],self.joints[0])
    #     # self.worldTarg = self.world.copy()
    #     # self.jointTargets = self.invKin(self.worldTarg)
    #
    #     self.jointTargets[1] = int(self.joints[1] -dP)
    #     self.jointTargets[0] = int(self.joints[0] -dZ)
    #
    #     # POSITIVE DOWN
    #     # NUB ARRAY 2ND AND 5TH VALUES
    #
    #
    #     time.sleep(.1)
    #
    #
    # def demoZK(self):
    #     self.comms.parseLine()
    #     calz1 = self.nub[1]-self.calVals[1]
    #     calz2 = self.nub[4]-self.calVals[4]
    #     torque = calz1-calz2
    #     force = calz1+calz2
    #
    #     # gainF = 0.000000001
    #     # gainT = 0.000000001
    #
    #     # gainF = 0.00035   # .0001 # 0002
    #     # gainT = 0.000000000
    #     gainF = 0.000   # .0001 # 0002
    #     gainT = 0.000000000
    #
    #     dZ = gainF*force
    #     dP = gainT*torque
    #
    #     if abs(dZ) > 3:
    #         dz = 3*abs(dZ)/dZ
    #
    #     # print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    #     # print("DZ DP: "+ str((dZ,dP)))
    #     # print("F T: "+ str((force, torque)))
    #     print("nub: " + str(self.nub)+"    Joints deg: " + str(self.joints))
    #     # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    #
    #     try:
    #             # 189
    #             # 211
    #             # 000
    #             # 000
    #             # 000
    #
    #         # GROOM JOINTS
    #
    #         groomedJoints = self.joints.copy()
    #
    #         groomedJoints[0] = -(groomedJoints[0]-189)
    #         groomedJoints[1] = -(groomedJoints[1]-211)
    #         groomedJoints[3] = 189
    #         # print("Groomed Joints deg: " + str(groomedJoints))
    #
    #         groomedJoints = [xxxyyy*math.pi/180. for xxxyyy in groomedJoints]
    #         # print("Groomed Joints: "+str(groomedJoints))
    #         #WORLDSPACE
    #         X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2],groomedJoints[3],groomedJoints[4],groomedJoints[1],groomedJoints[0])
    #         self.world = [X, Y, Z, THX, THY, THZ]
    #         # print("World Spa e: "+str(self.world))
    #
    #         #WORLD TARGETS
    #         self.worldTarg = self.world.copy()
    #         self.worldTarg[2] = self.worldTarg[2] - dZ
    #         self.worldTarg[4] = self.worldTarg[4] - dP
    #
    #         # print("World Target: "+str(self.worldTarg))
    #
    #         # JOINT TARGETS
    #         TH0, TH1, TH2, TH3, TH4= self.invKin(self.worldTarg[0],self.worldTarg[1],self.worldTarg[2],self.worldTarg[3],self.worldTarg[4],self.worldTarg[5])
    #         jointTargs = [TH4, TH3, TH0, TH1, TH2]
    #
    #         # print("Joint Target: "+str(jointTargs))
    #         # GROOM JOINT TARGETS
    #         jointTargs = [xyxyxy*180/math.pi for xyxyxy in jointTargs]
    #         jointTargs[0] = -jointTargs[0]+189
    #         jointTargs[1] = -jointTargs[1]+211
    #         self.jointTargets = jointTargs.copy()
    #         # print("Joint Target Groomed: "+str(jointTargs))
    #
    #         # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
    #         # self.worldTarg = self.world.copy()
    #         # self.jointTargets = self.invKin(self.worldTarg)
    #
    #
    #         # ################################################# #
    #         # self.jointTargets[1] = int(self.joints[1] -dP)    #
    #         # self.jointTargets[0] = int(self.joints[0] -dZ)    #
    #         # ################################################# #
    #     except:
    #         print("Kin Broke")
    #         # time.sleep(1)
    #     # POSITIVE DOWN
    #     # NUB ARRAY 2ND AND 5TH VALUES
    #     # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))
    #
    #     time.sleep(.05)
    #     # time.sleep(.1)
    #     # SPEED ISSUES
    #     # time.sleep(.2)
    #
    #
    # def demoZKF(self):
    #     i = -1
    #     while True:
    #         self.comms.parseLine()
    #         calz1 = self.nub[1]-self.calVals[1]
    #         calz2 = self.nub[4]-self.calVals[4]
    #         torque = calz1-calz2
    #         force = calz1+calz2
    #
    #         i = (i+3)%360
    #
    #         # gainF = 0.000000001
    #         # gainT = 0.000000001
    #
    #         dZ = math.sin(i*math.pi/180)*8
    #         dP = 0
    #         print("\n\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    #         print("DZ DP: "+ str((dZ,dP)))
    #         print("F T: "+ str((force, torque)))
    #         print("nub: " + str(self.nub))
    #         print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    #
    #         try:
    #                 # 189
    #                 # 211
    #                 # 000
    #                 # 000
    #                 # 000
    #
    #             # GROOM JOINTS
    #
    #             groomedJoints = [189, 211, 000, 180, 000]
    #             groomedJoints[0] = -(groomedJoints[0]-189)
    #             groomedJoints[1] = -(groomedJoints[1]-211)
    #             groomedJoints[3] = 180
    #             print("Groomed Joints deg: " + str(groomedJoints))
    #
    #             groomedJoints = [xxxyyy*math.pi/180. for xxxyyy in groomedJoints]
    #             print("Groomed Joints: "+str(groomedJoints))
    #             #WORLDSPACE
    #             X, Y, Z, THX, THY, THZ = self.fwdKin(groomedJoints[2],groomedJoints[3],groomedJoints[4],groomedJoints[1],groomedJoints[0])
    #             self.world = [X, Y, Z, THX, THY, THZ]
    #             print("World Spa e: "+str(self.world))
    #
    #             #WORLD TARGETS
    #             self.worldTarg = self.world.copy()
    #             self.worldTarg[2] = self.worldTarg[2] - dZ
    #             self.worldTarg[4] = self.worldTarg[4] - dP
    #
    #             print("World Target: "+str(self.worldTarg))
    #
    #             # JOINT TARGETS
    #             TH0, TH1, TH2, TH3, TH4= self.invKin(self.worldTarg[0],self.worldTarg[1],self.worldTarg[2],self.worldTarg[3],self.worldTarg[4],self.worldTarg[5])
    #             jointTargs = [TH4, TH3, TH0, TH1, TH2]
    #
    #             print("Joint Target: "+str(jointTargs))
    #             # GROOM JOINT TARGETS
    #             jointTargs = [xyxyxy*180/math.pi for xyxyxy in jointTargs]
    #             jointTargs[0] = -jointTargs[0]+189
    #             jointTargs[1] = -jointTargs[1]+211
    #             self.jointTargets = jointTargs.copy()
    #             print("Joint Target Groomed: "+str(jointTargs))
    #
    #             # self.world = self.fwdKin(self.joints[2],self.joints[3],self.joints[4],self.joints[0],self.joints[1])
    #             # self.worldTarg = self.world.copy()
    #             # self.jointTargets = self.invKin(self.worldTarg)
    #
    #
    #             # ################################################# #
    #             # self.jointTargets[1] = int(self.joints[1] -dP)    #
    #             # self.jointTargets[0] = int(self.joints[0] -dZ)    #
    #             # ################################################# #
    #         except:
    #             print("Kin Broke")
    #             time.sleep(10)
    #         # POSITIVE DOWN
    #         # NUB ARRAY 2ND AND 5TH VALUES
    #         # print(" joints: "+str(self.joints)+" nub: "+str(self.nub)+" jointTargets: "+str(self.jointTargets) +" dZ,dP: "+str((dZ,dP)))
    #
    #         # time.sleep(.05)
    #         # SPEED ISSUES
    #         time.sleep(.2)
