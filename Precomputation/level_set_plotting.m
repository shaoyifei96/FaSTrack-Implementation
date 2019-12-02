
% [g3D, data3D] = proj(sD.grid,data,[0 0 0],'min');
[g3D, data3D] = proj(sD.grid,data,[0 0 0 1],'min');
%%
close all; 

  small = 0.01;
%     subplot(2,3,1)
%     h0 = visSetIm(g3D, sqrt(data03D), 'blue', levels(1)+small);
%     h0.FaceAlpha = alpha;
    hold on
    levels= [0;0; 0.84
        ]
    theta=0:0.01:2*pi;
    x=0.2*sin(theta);
    y = 0.2*cos(theta);
figure(1)
    h = visSetIm(g3D, sqrt(data3D), 'red', levels(3));
    hold on; 
    plot(x,y)
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small -pi pi])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$l$','interpreter','latex');
    
    %%
    x = g3D.vs{1};
    y = g3D.vs{2};
    z = g3D.vs{3};
    skipn = 5;
  [X, Y, Z] = meshgrid(x(1:skipn:end),y(1:skipn:end),z(1:skipn:end)); 
  
[gradx,grady,gradz] = gradient(data3D(1:skipn:end,1:skipn:end,1:skipn:end));
% gradx = reshape(gradx,size(X));
% grady = reshape(grady,size(Y));
% gradz = reshape(gradz,size(Z));

figure(2)
quiver3(X,Y,Z,gradx,grady,gradz)
axis equal
xlabel 'x'
ylabel 'y'
zlabel 'z'
title('Quiver Plot of Estimated Gradient of Solution')