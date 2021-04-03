%% user parameters
% agent
close all
desired_speed = 1 ; % m/s
if ~exist('TEB_data','var')
    disp('Loading frs')
    TEB_data = load("Dubin4D2.0_0.3_40_vhigh_debugged.mat");
    avoid_data = load("avoid_fun_box_data.mat");
else
    disp('TEB already loaded') ;
end
rng(100)
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
% N_wall_obstacles = 5 ;
bounds = [-5,5,-5,5] ;
start = [-4;-4] ;
goal = [4;4] ;
goal_radius = 0.5 ;
box_size = 1 ;

% planner time limits  
t_plan = 0.5 ;
t_move = 0.5 ;

% RTD params
RTD_buffer = 0.01 ;

% simulation
verbose_level = 5 ;
plot_HLP_flag = true ;

% file i/o
save_summaries_flag = false ;
save_file_location = '~/MATLAB/fastrack_comparison_data/' ;

%% (automated from here) make agents
% make agents
A_RTD =  turtlebot_agent;
A_FTK = fastrack_agent(TEB_data) ;
A_FTK.use_performance = "Fastrack";
A_FTK.LLC.TEBadj = 0.21; %Acoording to slides shared last meeting, with this loaded file. TEB limit is at 0.21m
A_FTK.LLCP.gains.acceleration = 1.8;
% so if planner and dynamics gets too far away, use safety controller.
A_FTK_avoid = fastrack_agent([]) ; % Use the same agent for avoid formulation as well.
% Since most things are the same, except the limit is opposite, If value
% function less than 1+footprint, it is too close to obstacles.
A_FTK_avoid.LLC = fastrack_avoid_LLC(avoid_data);
A_FTL_avoid.LLC.TEBadj = A_FTK_avoid.footprint;
A_FTK_avoid.use_performance  =  "Avoid";
% make agent cell array for simulator
A = {A_FTK, A_FTK_avoid,A_RTD, A_RTD} ;

%% make planners
% fastrack params
fastrack_buffer = A_FTK.LLC.TEB.TEB + A_FTK.footprint ;
avoid_buffer = A_FTK.footprint;
% fastrack planner
P_FTK = turtlebot_RRT_planner('verbose',verbose_level,'buffer',fastrack_buffer,...
    't_plan',t_plan,'t_move',t_move,'desired_speed',desired_speed,...
    'plot_HLP_flag',plot_HLP_flag) ;

P_FTK_avoid = turtlebot_RRT_planner('verbose',verbose_level,'buffer',avoid_buffer,...
    't_plan',t_plan,'t_move',t_move,'desired_speed',desired_speed,...
    'plot_HLP_flag',plot_HLP_flag) ;

% RTD planner
P_RTD_1 = turtlebot_RTD_planner_static('verbose',verbose_level,'buffer',RTD_buffer,...
                                 't_plan',t_plan,'t_move',t_move,'HLP',RRT_HLP(),...
                                 'plot_HLP_flag',plot_HLP_flag) ;
                             
% RTD planner w/ straight line HLP
P_RTD_2 = turtlebot_RTD_planner_static('verbose',verbose_level,'buffer',RTD_buffer,...
                                 't_plan',t_plan,'t_move',t_move,...
                                 'plot_HLP_flag',plot_HLP_flag) ;

% make planner input
P = {P_FTK, P_FTK_avoid, P_RTD_1, P_RTD_2} ;

%% make world
W = static_box_world('bounds',bounds,'N_obstacles',0,...
    'verbose',verbose_level,'goal_radius',goal_radius,...
    'obstacle_size_bounds',obstacle_size_bounds,...
    'buffer',fastrack_buffer,'start',start,'goal',goal) ;

% get goal and y bounds
g = W.goal ;
ylo = W.bounds(3) + fastrack_buffer ;
yhi = W.bounds(4) - fastrack_buffer ;

% make list of obstacle centers
% enters = {[-1,-2], [-0.5, -0.5], [2, 3], [1,0]};
c = [[-1,-2]; [-0.5, -0.5]; [2, 3]; [1,0]];
O = [] ;
% for idx = 1:N_wall_obstacles
%     o = make_circle(1,30,c(1,:)); %+ repmat([W.goal(1);0] + [-1;c(idx)],1,5) ;
%     O = [O, nan(2,1), o] ;
% end
for idx = 1:size(c,1)
    o = make_box(box_size,c(idx,:)); %+ repmat([W.goal(1);0] + [-1;c(idx)],1,5) ;
    O = [O, nan(2,1), o] ;
end
O = O(:,2:end) ;

W.obstacles = O ;
W.N_obstacles = 4;
% W.N_obstacles = N_wall_obstacles ;
W.setup() ;

%% run FTK simulation
S = simulator(A,W,P,'allow_replan_errors',false,'verbose',verbose_level,...
    'max_sim_time',30,'max_sim_iterations',1000,'plot_while_running',1) ;

S.run(2) %1. fastrack, 2. stamped HJB, 3. RTD RRT. 4. RTD straightline

