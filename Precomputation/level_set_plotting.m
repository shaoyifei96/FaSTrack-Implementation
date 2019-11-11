close all; 
[g3D, data3D] = proj(sD_X.grid,data,[0 1 1 1 0 1 1 1 0 1],'min');
  small = 0.01;
%     subplot(2,3,1)
%     h0 = visSetIm(g3D, sqrt(data03D), 'blue', levels(1)+small);
%     h0.FaceAlpha = alpha;
    hold on
    levels= [0;0; 3.5 ]
    h = visSetIm(g3D, sqrt(data3D), 'red', levels(3));
    axis([-levels(3)-small levels(3)+small ...
        -levels(3)-small levels(3)+small -pi pi])
    axis square
    xlabel('$r_x$','interpreter','latex');
    ylabel('$r_y$','interpreter','latex');
    zlabel('$l$','interpreter','latex');