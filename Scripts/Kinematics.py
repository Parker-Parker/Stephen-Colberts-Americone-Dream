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
