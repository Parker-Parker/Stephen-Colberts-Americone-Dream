import math


class Robot:
    TL = 13
    BL = 14
    GL = 2
    FL = 2
    LL = 4
    t = 21
    HH = 1
    HL = 2
    L0 = 4
    L1 = 4

    def fwdKin(self, TH0, TH1, TH2, TH3, TH4):


            #     function[X, Y, Z, THX, THY, THZ] = Forwardplswork(TH0, TH1, TH2, TH3, TH4, HH, HL)
            #
            #         TL = 13;
            #         BL = 14;
            #         GL = 2;
            #         FL = 2;
            #         LL = 4;
            #         t = 21;
            #
            #         PPY = -GL - LL * cos(TH4);
            #         PPX = LL * sin(TH4);
            #
            #         PQY = TL * sin(TH3);
            #         PQX = TL * cos(TH3);

        PPY = -self.GL - self.LL * math.cos(TH4)
        PPX = self.LL * math.sin(TH4)

        PQY = self.TL * math.sin(TH3)
        PQX = self.TL * math.cos(TH3)


            #
            #         P = sqrt(PPY ^ 2 + PPX ^ 2);
            #         Q = sqrt((PPY - PQY) ^ 2 + (PPX - PQX) ^ 2);
            #
            #         ALPHA = acos((Q ^ 2 + TL ^ 2 - P ^ 2) / (2 * Q * TL));
            #         BETA = acos((Q ^ 2 + FL ^ 2 - BL ^ 2) / (2 * Q * FL));
            #
            #         THK = ALPHA + BETA - pi / 2;
            #

        P = math.sqrt(PPY ** 2 + PPX ** 2)
        Q = math.sqrt((PPY - PQY) ** 2 + (PPX - PQX) ** 2)

        ALPHA = math.acos((Q ** 2 + self.TL ** 2 - P ** 2) / (2 * Q * self.TL))
        BETA = math.acos((Q ** 2 + self.FL ** 2 - self.BL ** 2) / (2 * Q * self.FL))

        THK = ALPHA + BETA - math.pi / 2

            #     % -THY = TH3 + THK
            #     % Z = t + 12 * sin(TH3) - HH * cos(TH3 + THK) + HL * sin(TH3 + THK)
            #
            #     Z = t + TL * sin(TH3) - HH * cos(TH3 + THK) + HL * sin(TH3 + THK);
            #     THY = -(TH3 + THK);
            #
            #     dt = TL * cos(TH3) + HH * sin(-THY) + HL * cos(-THY);
            #
            #     THZ = TH0 + TH1 + TH2;
            #
            #     X = 4 * cos(TH0) + 4 * cos(TH0 + TH1) + dt * cos(THZ);
            #     Y = 4 * sin(TH0) + 4 * sin(TH0 + TH1) + dt * sin(THZ);
            #
            #     THX = 0;
            #
            #
            # end

        # ###################################################################
        # % -THY = TH3 + THK
        # % Z = t + 12 * sin(TH3) - HH * cos(TH3 + THK) + HL * sin(TH3 + THK)
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
        K = math.sqrt(PKX ** 2 + (PKY + 2) ** 2)

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


    """

    def invKin(self,X, Y, Z, THX, THY, THZ):
        # function[TH0, TH1, TH2, TH3, TH4] = Inverseplswork(X, Y, Z, THX, THY, THZ, HH, HL)
        # % UNTITLED
        # Summary
        # of
        # this
        # function
        # goes
        # here
        # % Detailed
        # explanation
        # goes
        # here
        # TL = 13;
        # BL = 14;
        # L0 = 4;
        # L1 = 4;
        # % -THY = TH3 + THK
        # % Z = t + 12 * sin(TH3) - HH * cos(TH3 + THK) + HL * sin(TH3 + THK)
        # %
        # % Z = t + 12 * sin(TH3) - HH * cos(-THY) + HL * sin(-THY)
        # % -12 * sin(TH3) = t - Z - HH * cos(-THY) + HL * sin(-THY)
        #
        # TH3 = asin((21 - Z - HH * cos(-THY) + HL * sin(-THY)) / (-TL));
        # THK = -THY - TH3;
        #
        # dt = TL * cos(TH3) + HH * sin(-THY) + HL * cos(-THY);
        #
        # % X = P2X + dt * cos(THZ)
        # P2X = X - dt * cos(THZ);
        # % Y = P2Y + dt * sin(THZ);
        # P2Y = Y - dt * sin(THZ);
        #




            ##############################################################################################
        # % -THY = TH3 + THK
        # % Z = t + 12 * sin(TH3) - HH * cos(TH3 + THK) + HL * sin(TH3 + THK)
        # %
        # % Z = t + 12 * sin(TH3) - HH * cos(-THY) + HL * sin(-THY)
        # % -12 * sin(TH3) = t - Z - HH * cos(-THY) + HL * sin(-THY)
            ##############################################################################################

        TH3 = math.asin((self.t - Z - self.HH * math.cos(-THY) + self.HL * math.sin(-THY)) / (-self.TL))
        THK = -THY - TH3

        dt = self.TL * math.cos(TH3) + self.HH * math.sin(-THY) + self.HL * math.cos(-THY)

        # % X = P2X + dt * math.cos(THZ)
        P2X = X - dt * math.cos(THZ);
        # % Y = P2Y + dt * math.sin(THZ);
        P2Y = Y - dt * math.sin(THZ);


        # % TH2 = 2345676543456789876567890 %???????????????? KHAAAAAAAANNNNNN!!!!!!!!!
        #
        # % P2X = 4 * cos(TH0) + 4 * cos(TH0 + TH1);
        # % P2Y = 4 * sin(TH0) + 4 * sin(TH0 + TH1);
        #
        # % % % % % TH0 = 2 * atan2((8 * P2Y + (- P2X ^ 4 - 2 * P2X ^ 2 * P2Y ^ 2 + 64 * P2X ^ 2 - P2Y ^ 4 + 64 * P2Y ^ 2) ^ (1 / 2)),(P2X ^ 2 + 8 * P2X + P2Y ^ 2));
        # % % % % % TH0 = 2 * atan2((8 * P2Y - (- P2X ^ 4 - 2 * P2X ^ 2 * P2Y ^ 2 + 64 * P2X ^ 2 - P2Y ^ 4 + 64 * P2Y ^ 2) ^ (1 / 2)),(P2X ^ 2 + 8 * P2X + P2Y ^ 2));
        # % % % % %
        # % % % % % TH1 = -2 * atan2((-(P2X ^ 2 + P2Y ^ 2) * (P2X ^ 2 + P2Y ^ 2 - 64)) ^ (1 / 2), (P2X ^ 2 + P2Y ^ 2));
        # % % % % % TH1 = 2 * atan2((-(P2X ^ 2 + P2Y ^ 2) * (P2X ^ 2 + P2Y ^ 2 - 64)) ^ (1 / 2), (P2X ^ 2 + P2Y ^ 2));
        #
        # THR = atan2(P2Y, P2X);
        # R = sqrt(P2X ^ 2 + P2Y ^ 2);
        #
        # GAMMA = acos((L0 ^ 2 + R ^ 2 - L1 ^ 2) / (2 * L0 * R));
        #
        # % RIGHT
        # TH0 = THR - GAMMA;
        # TH1 = 2 * GAMMA;
        THR = math.atan2(P2Y, P2X)
        R = math.sqrt(P2X ** 2 + P2Y ** 2)

        GAMMA = math.acos((self.L0 ** 2 + R ** 2 - self.L1 ^ 2) / (2 * self.L0 * R));

        # % RIGHT
        TH0 = THR - GAMMA
        TH1 = 2 * GAMMA

        # ################################
        # % % % % % % LEFT
        # % % % % % TH0 = THR + GAMMA;
        # % % % % % TH1 = -2 * GAMMA;
        # ################################

        #
        # % NOW
        # IS
        # THE
        # TIME
        # TH2 = THZ - TH0 - TH1;
        # % TO
        # DIME
        #
        # % THK: TH4
        # DUDE
        # its
        # its
        # own
        # thing...like
        # holy
        # shit
        # % TH4 = "KEKSAUCE"
        # % F = sqrt((2 * 2 * TL) * cos(pi / 2 + THK) - TL ^ 2 - 2 ^ 2);
        #
        # PKX = TL * cos(TH3) + 2 * sin(TH3 + THK);
        # PKY = TL * sin(TH3) - 2 * cos(TH3 + THK);
        #
        # F = sqrt(PKX ^ 2 + (PKY) ^ 2);
        # K = sqrt(PKX ^ 2 + (PKY + 2) ^ 2);
        #
        # ALPHA = acos((2 ^ 2 + K ^ 2 - F ^ 2) / (2 * 2 * K));
        # BETA = acos((K ^ 2 + 4 ^ 2 - BL ^ 2) / (K * 4 * 2));
        # TH4 = pi - ALPHA - BETA;

        # % TH4 = "KEKSAUCE"
        # % F = sqrt((2 * 2 * TL) * cos(pi / 2 + THK) - TL ^ 2 - 2 ^ 2);

        PKX = self.TL * math.cos(TH3) + 2 * math.sin(TH3 + THK)
        PKY = self.TL * math.sin(TH3) - 2 * math.cos(TH3 + THK)

        F = math.sqrt(PKX ** 2 + (PKY) ** 2)
        K = math.sqrt(PKX ** 2 + (PKY + 2) ** 2)

        ALPHA = math.acos((2 ** 2 + K ** 2 - F ** 2) / (2 * 2 * K))
        BETA = math.acos((K ** 2 + 4 ** 2 - self.BL ** 2) / (K * 4 * 2))
        TH4 = math.pi - ALPHA - BETA

        # % ITS
        # THE
        # TIME
        # OF
        # THE
        # SEASON
        # outputArg1 = 1;
        # outputArg2 = 2;
        #
        # % SOMETHIN
        # SOMETHIN
        # GOOD
        # REASON
        # end
        return TH0, TH1, TH2, TH3, TH4
        
        """


r = Robot()
r.testKin()



