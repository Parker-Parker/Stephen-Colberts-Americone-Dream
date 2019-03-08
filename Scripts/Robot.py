import math
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Robot:
    TL = 13
    BL = 14
    GL = 2.5
    FL = 2
    LL = 4
    t = 21
    HH = 1
    HL = 2
    L0 = 4
    L1 = 4


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

        PPY = -self.GL - self.LL * math.cos(TH4)
        PPX = self.LL * math.sin(TH4)

        PQY = self.TL * math.sin(TH3)
        PQX = self.TL * math.cos(TH3)


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

        ALPHA = math.acos((Q ** 2 + self.TL ** 2 - P ** 2) / (2 * Q * self.TL))
        BETA = math.acos((Q ** 2 + self.FL ** 2 - self.BL ** 2) / (2 * Q * self.FL))

        THK = ALPHA + BETA - math.pi / 2



        # ###################################################################
        # % -THY = TH3 + THK
        # % Z = self.t + 12 * math.sin(TH3) - self.HH * math.cos(TH3 + THK) + self.HL * math.sin(TH3 + THK)
        # ###################################################################

        Z = self.t + self.TL * math.sin(TH3) - self.HH * math.cos(TH3 + THK) + self.HL * math.sin(TH3 + THK)
        THY = -(TH3 + THK)

        dt = self.TL * math.cos(TH3) + self.HH * math.sin(-THY) + self.HL * math.cos(-THY)

        THZ = TH0 + TH1 + TH2

        X = self.L0 * math.cos(TH0) + self.L1 * math.cos(TH0 + TH1) + dt * math.cos(THZ)
        Y = self.L0 * math.sin(TH0) + self.L1 * math.sin(TH0 + TH1) + dt * math.sin(THZ)

        THX = 0

        return X, Y, Z, THX, THY, THZ;

    def invKin(self,X, Y, Z, THX, THY, THZ):

        TH3 = math.asin((self.t - Z - self.HH * math.cos(-THY) + self.HL * math.sin(-THY)) / (-self.TL))
        THK = - THY - TH3
        dt = self.TL * math.cos(TH3) + self.HH * math.sin(-THY) + self.HL * math.cos(-THY)
        P2X = X - dt * math.cos(THZ)
        P2Y = Y - dt * math.sin(THZ)
        THR = math.atan2(P2Y, P2X)
        R = math.sqrt(P2X ** 2 + P2Y ** 2)
        GAMMA = math.acos((self.L0 ** 2 + R ** 2 - self.L1 ** 2) / (2 * self.L0 * R))
        # % RIGHT
        TH0 = THR - GAMMA
        TH1 = 2 * GAMMA
        # % % LEFT
        # % TH0 = THR + GAMMA;
        # % TH1 = -2 * GAMMA;
        TH2 = THZ - TH0 - TH1
        PKX = self.TL * math.cos(TH3) + self.FL * math.sin(TH3 + THK)
        PKY = self.TL * math.sin(TH3) - self.FL * math.cos(TH3 + THK)

        F = math.sqrt(PKX ** 2 + (PKY) ** 2)
        K = math.sqrt(PKX ** 2 + (PKY + self.GL) ** 2)

        ALPHA = math.acos((self.GL ** 2 + K ** 2 - F ** 2) / (self.GL * 2 * K))
        BETA = math.acos((K ** 2 + self.LL ** 2 - self.BL ** 2) / (K * self.LL * 2))
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

        self.drawf(TH0, TH1, TH2, TH3, TH4)

    def fwdKinPTS(self, TH0, TH1, TH2, TH3, TH4):
        PPY = -self.GL - self.LL * math.cos(TH4)
        PPX = self.LL * math.sin(TH4)
        PQY = self.TL * math.sin(TH3)
        PQX = self.TL * math.cos(TH3)

        P = math.sqrt(PPY ** 2 + PPX ** 2)
        Q = math.sqrt((PPY - PQY) ** 2 + (PPX - PQX) ** 2)

        ALPHA = math.acos((Q ** 2 + self.TL ** 2 - P ** 2) / (2 * Q * self.TL))
        BETA = math.acos((Q ** 2 + self.FL ** 2 - self.BL ** 2) / (2 * Q * self.FL))

        THK = ALPHA + BETA - math.pi / 2

        XPTS = []
        XPTS.append(0)
        XPTS.append(XPTS[0] + self.L0 * math.cos(TH0))
        XPTS.append(XPTS[1] + self.L1 * math.cos(TH0 + TH1))
        XPTS.append(XPTS[2] + 0)
        XPTS.append(XPTS[3] + self.TL * math.cos(TH0 + TH1 + TH2) * math.cos(TH3))
        XPTS.append(XPTS[4] + self.HH * math.cos(TH0 + TH1 + TH2) * math.sin(TH3 + THK))
        XPTS.append(XPTS[5] + self.HL * math.cos(TH0 + TH1 + TH2) * math.cos(TH3 + THK))

        YPTS = []
        YPTS.append(0)
        YPTS.append(YPTS[0] + self.L0 * math.sin(TH0))
        YPTS.append(YPTS[1] + self.L1 * math.sin(TH0 + TH1))
        YPTS.append(YPTS[2] + 0)
        YPTS.append(YPTS[3] + self.TL * math.sin(TH0 + TH1 + TH2) * math.cos(TH3))
        YPTS.append(YPTS[4] + self.HH * math.sin(TH0 + TH1 + TH2) * math.sin(TH3 + THK))
        YPTS.append(YPTS[5] + self.HL * math.sin(TH0 + TH1 + TH2) * math.cos(TH3 + THK))

        ZPTS = []
        ZPTS.append(0)
        ZPTS.append(ZPTS[0] + 0)
        ZPTS.append(ZPTS[1] + 0)
        ZPTS.append(ZPTS[2] + self.t)
        ZPTS.append(ZPTS[3] + self.TL * math.sin(TH3))
        ZPTS.append(ZPTS[4] - self.HH * math.cos(TH3 + THK))
        ZPTS.append(ZPTS[5] + self.HL * math.sin(TH3 + THK))
        return [XPTS, YPTS, ZPTS]

    def drawf(self, TH0, TH1, TH2, TH3, TH4):
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d', )

        [xs, ys, zs] = self.fwdKinPTS(TH0, TH1, TH2, TH3, TH4)
        self.ax.axis('equal')

        self.ax.plot(xs, ys, zs)
        # print((xs, ys, zs))
        self.fig.show()


if __name__ == '__main__':
    r = Robot()
    r.testKin()



