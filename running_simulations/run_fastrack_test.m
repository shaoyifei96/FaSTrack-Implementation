close all; clear
tebfile=load('Dubin4D2.0_0.3_40_vhigh_debugged.mat');
TEB= zeros(10, 1);
spds= zeros(10, 1);
v_maxmax = 1.5;
plan_spd = 0.3;
T_interval = 10;
dist = T_interval * plan_spd;
n = 10;
% for i = 3: n
% spd = i*v_maxmax/n;
 spd = 100;
A = fastrack_agent(tebfile);
% A.integrator_type = 'ode45';
A.LLC.TEB.sD.dynSys.max_spd = spd;
T = [0 T_interval 2*T_interval 3*T_interval ];
U = zeros(A.n_inputs,length(T));
%set reference Z as 4 states, with planning speed, so it should intropolate
%properly between reference points, and not too fast
Z = [0 dist dist dist ; 0 0 dist 0; 0 0 0 0 ; plan_spd plan_spd plan_spd plan_spd ];
A.move(T(end) -1,T,U,Z);
% spds(i) = spd;
% TEB(i) = A.TEB_max;


 
  figure(1) ; axis equal ; hold on ; grid on ;
   plot(A)
   A.animate()
% end

%%
% close all; clear
% load Dubin3D50_dt010_tMax_converge.mat
% % TEB= zeros(10, 1)
% % spds= zeros(10, 1)
% % v_maxmax = 1.5;
% plan_spd = 0.2;
% T_interval = 20;
% dist = T_interval * plan_spd;
% % n = 10;
% % for i = 3: n
% % spd = i*v_maxmax/n;
% %  spd = 0.2;
% A = turtlebot_3D_agent;
% % % A.LLC.TEB.sD.dynSys.v_max = spd;
% % % A.use_performance = 0; 
% T = [0 T_interval 2*T_interval 3*T_interval ];
% U = zeros(A.n_inputs,length(T));
% Z = [0 dist dist dist ; 0 0 dist dist; 0 0 0 0 ; plan_spd plan_spd plan_spd 0 ];
% A.move(T(end) -1,T,U,Z);
% %%
%   figure(1) ; axis equal ; hold on ; grid on ;
%    plot(A)
