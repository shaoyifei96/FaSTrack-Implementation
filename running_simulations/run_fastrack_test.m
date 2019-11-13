A = fastrack_agent
T = [0 10 20];
U = zeros(A.n_inputs,3)
Z = [0 0 0 ; 0 2 0; 0 0 0; 0.2 0.2 0.2];
A.move(19,T,U,Z);
close all; 
plot(A)
figure(1) ; axis equal ; hold on ; grid on ;
plot_path(Z,'b--')
plot(A)