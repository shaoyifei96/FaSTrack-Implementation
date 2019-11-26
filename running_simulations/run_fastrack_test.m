close all; clear
load Dubin4D1mpers_veryHigh.mat
TEB= zeros(10, 1)
spds= zeros(10, 1)
v_maxmax = 1.0;
n = 10;
for i = 3: n
spd = i*v_maxmax/n;
%  spd = 0.45;
A = fastrack_agent;
A.LLC.TEB.sD.dynSys.v_max = spd;
T = [0 10 20 30 ];
U = zeros(A.n_inputs,length(T));
Z = [0 2 2 2 ; 0 0 2 2; 0 0 0 0 ; 0.2 0.2 0.2 0 ];
A.move(25,T,U,Z);
spds(i) = spd;
TEB(i) = A.TEB_max;
%%

 
  figure(1) ; axis equal ; hold on ; grid on ;
   plot(A)
end
