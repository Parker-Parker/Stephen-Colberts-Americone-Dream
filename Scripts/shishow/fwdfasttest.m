function [avg, times] = fwdfasttest(loops)
times = 1:loops;
    for n = 1:loops
        T0 = 2*pi*rand();
        T1 = 2*pi*rand();
        T2 = 2*pi*rand();
        T3 = 20*pi/360 - 20*pi/180*rand();
        T4 = 20*pi/360 - 20*pi/180*rand();
        
        tic;
        dofwdfast(T0, T1, T2, T3, T4, 1, 2);
        times(n) = toc;
%         vals
    end
    avg = mean(times);
end