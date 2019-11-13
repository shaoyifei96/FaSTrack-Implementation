A = fastrack_agent
T = [0 10];
U = zeros(A.n_inputs,2)
Z = [0 2. ; 0 -2. ; 0 0 ; 0.2 0.2];
A.move(9,T,U,Z);
close all; 
plot(A)
figure(1) ; axis equal ; hold on ; grid on ;
plot_path(Z,'b--')
plot(A)