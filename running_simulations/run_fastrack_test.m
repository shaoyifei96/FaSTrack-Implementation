close all; clear
load Dubin4D1.5_43_veryHigh_manydays.mat
TEB= zeros(10, 1)
spds= zeros(10, 1)
v_maxmax = 1.5;
n = 20;
% for i = 1: n
% spd = i*v_maxmax/n;
 spd = ;
A = fastrack_agent;
A.LLC.TEB.sD.dynSys.v_max = spd;
T = [0 10 20 30];
U = zeros(A.n_inputs,length(T));
Z = [0 2 2 2 ; 0 0 2 0; 0 0 0 0; 0.2 0.2 0.2 0.2];
A.move(29,T,U,Z);
% spds(i) = spd;
% TEB(i) = A.TEB_max;
%%
 plot(A)
 
 figure(1) ; axis equal ; hold on ; grid on ;
end
