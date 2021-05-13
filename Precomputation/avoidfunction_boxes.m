% function [g,Value,Deriv] = avoidfunction(centers,radius)
% [g, data] = avoidfunction(centers,radius)
% Solves for spatial gradients of the value function to avoid a set of 
% cylindrical obstacles with given centers and fixed radius. 
%
% Normally you would apply the optimal safe control on the boundary where 
% You would do this by finding the argmax of the hamiltonian.  This means 
% taking the inner product between the spatial gradients of the value 
% function (Deriv) and the dynamics of the system. When maximizing over 
% that you will find the optimal safe control.
%
% for a particular state in your dynamic system dynSys, this means:
%     %find deriv at exactly x
%     deriv = eval_u(g, Deriv, dynSys.x); 
%
%     % plug into function for optimal control given dynamics
%     u = dynSys.optCtrl(tau(tEarliest), dynSys.x, deriv, uMode);
%
%     % plug that optimal control into the system
%     dynSys.updateState(u, dtSmall, dynSys.x);
%
% However, ANY control such that that inner product (the hamiltonian) is
% non-negative will be a sufficient safe control.  You just want to make 
% sure the resulting hamiltonian is not negative on the boundary.
%
% For questions, contact Sylvia Herbert, sherbert@ucsd.edu
%
% ----- How to use this function -----
%
% Inputs:
%   centers         - (cell) list of centers of obstacles. Example:
%                       centers = {[0,0], [1,1]} would create two
%                       obstacles with centers at positions (0,0) and (1,1)
%   radius          - (double) radius of all obstacles. Can easily turn
%                       into a list of varying radii if you prefer.
%
% Outputs:
%   g               - (struct) grid over state space
%   Value           - (matrix) value function defined over that grid.
%   Deriv           - (cell) spatial gradients of the value function 
%                       defined over that grid.
clear
for i = 1:20
% if nargin < 1
rng(i);
centers = cell(0);
for obs_idx = 1:4
    rx = -3 + (5-3).*rand(1,2);
    centers{end+1} = [rx]; %obstacle centers
end
rec_length = 1;

% end

% if nargin <2
% radius = 1; %not used
% end

%% Grid
% grid_min = [-5; -5; -pi]; % Lower corner of computation domain
% grid_max = [5; 5; pi];    % Upper corner of computation domain
% N = [50; 50; 50];         % Number of grid points per dimension
% pdDims = 3;               % 3rd dimension is periodic
% g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions
%% Simon's Grid
% gN = ones(4, 1)*40;
gN = [80, 80, 40, 37];
% end
pdDims = 3;               % 3rd dimension is periodic

% default to visualizing
% if nargin < 2
%     visualize = 1;

   max_spd = 2;
% grid bounds in x, y, theta (relative dynamics)

x_bounds = [-5, 5];
y_bounds = [-5, 5];
spd_bounds = [-2, 2];
bound_add = 0.25;
grid_min = [x_bounds(1)-bound_add; y_bounds(1)-bound_add;
    -pi;  -max_spd-bound_add]; %should be size of the state space
grid_max = [x_bounds(2)+bound_add;  y_bounds(2)+bound_add;
    pi;  max_spd+bound_add];

% create grid with 3rd dimension periodic
g = createGrid(grid_min, grid_max, gN,pdDims);

%% target set

% data0 = shapeCylinder(g, 3, [centers{1},0], radius);
% data0 = shapeRectangleByCenter(g, [centers{1},0, 0], [1 1 pi*2 max_spd*2]);%size of rectangle is big
data0 = -shapeRectangleByCorners(g, [x_bounds(1), y_bounds(1), -inf, -inf], [x_bounds(2), y_bounds(2), inf, inf]);
for ii = 1:length(centers)
   newobs = shapeRectangleByCenter(g, [centers{ii},0,0], [rec_length, rec_length, inf inf]);
   data0 = shapeUnion(data0,newobs);
end


%% time vector
t0 = 0;
tMax = 10;
% tMax = 1;
dt = 0.05;
tau = t0:dt:tMax;

% stop the computation once it converges
HJIextraArgs.stopConverge = 1;
HJIextraArgs.convergeThreshold = .01;

%% problem parameters

% input bounds
% speed = 1;
% wMax = 1;
w_max = 2; % rad/s
acc_max = 2;
p1_limit = 0;%what to set this? 
p2_limit = 0;
dCar = Unicycle4DRelDubins([0, 0, 0,0],acc_max, w_max, p1_limit, p2_limit,max_spd);

% control trying to min or max value function?
uMode = 'max';

%% Pack problem parameters

% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
% dCar = DubinsCar([0, 0, 0], wMax, speed); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'medium'; %set accuracy
schemeData.uMode = uMode;
HJIextraArgs.targetFunction = data0;

%% Visualization


HJIextraArgs.visualize.plotColorVS0 = 'r';
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.visualize.initialValueSet = 1;
HJIextraArgs.visualize.figNum = 1; %set figure number
HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.visualize.viewAxis = ...
    [grid_min(1) grid_max(1)...
    grid_min(2) grid_max(2) ...
    grid_min(2) grid_max(3)];

%% Compute value function
%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
[Value, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'minVWithL', HJIextraArgs);
Value = Value(:,:,:,:,end);
Deriv = computeGradients(g, Value);
%%
save("avoid_fun_box_data_"+num2str(i)+".mat",'Value','Deriv','schemeData')
end
% end