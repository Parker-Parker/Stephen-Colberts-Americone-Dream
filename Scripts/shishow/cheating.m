function ee = dofwdfast2(TH0, TH1, TH2, TH3, TH4, HH, HL, varargin)
    THK = -pi/2 + acos((sin(TH3) + 2*sin(TH3 - TH4) + 6)/(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)) + acos((16*cos(TH4) + 48*sin(TH3) + 96*sin(TH3 - TH4) - 1)/(8*(4*cos(TH4) + 12*sin(TH3) + 24*sin(TH3 - TH4) + 41)^(1/2)));
    PI = pi;
          L(1) =   Link( [  TH0,  2,4,     0])
           L(2) =   Link(   [   TH1, 2,4, 0])
          L(3) =   Link(    [  TH2, 17,  0,  PI/2 ])
          L(4) =   Link(    [ TH3,  0, 12,     0     ])
          L(5) =   Link(    [ THK-PI/2,  0, HH,     0])
          L(6) =   Link(    [ PI/2, 0,HL,  -PI/2])   %STATIC FRAME OF END EFFECTOR

          
          Robot = SerialLink(L)
          Robot.name = "kms"
          Robot.plot.
          
          
          
          
          
          
          
          
          
          
          % ik = robotics.InverseKinematics;