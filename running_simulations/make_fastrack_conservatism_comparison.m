%% user parameters
% agent
desired_speed = 1 ; % m/s
tebfile=load('Dubin4D2.0_0.3_40_vhigh_debugged.mat');
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_wall_obstacles = 5 ;
bounds = [-4,4,-2,2] ;
start = [-3;0] ;
goal = [3.5;0] ;
goal_radius = 0.5 ;
box_size = 0.25 ;

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
A_FTK = fastrack_agent(tebfile) ;
A_FTK.integrator_type = 'ode45';

% make agent cell array for simulator
A = {A_FTK, A_RTD, A_RTD} ;

%% make planners
% fastrack params
fastrack_buffer = A_FTK.LLC.TEB.TEB + A_FTK.footprint ;

% fastrack planner
P_FTK = turtlebot_RRT_planner('verbose',verbose_level,'buffer',fastrack_buffer,...
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
P = {P_FTK, P_RTD_1, P_RTD_2} ;

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
c = linspace(ylo,yhi,N_wall_obstacles) ;
O = [] ;
for idx = 1:N_wall_obstacles
    o = make_box(box_size) + repmat([W.goal(1);0] + [-1;c(idx)],1,5) ;
    O = [O, nan(2,1), o] ;
end
O = O(:,2:end) ;

W.obstacles = O ;
W.N_obstacles = N_wall_obstacles ;
W.setup() ;

%% run FTK simulation
S = simulator(A,W,P,'allow_replan_errors',false,'verbose',verbose_level,...
    'max_sim_time',30,'max_sim_iterations',1000,'plot_while_running',1) ;

S.run(1)
