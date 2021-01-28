clear ; clc ;
%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RRT to plan online and FasTrack as a low-level
% controller.
%
% Author: Shreyas Kousik and Simon Shao
% Created: 1 Nov 2019
% Updated: 4 Nov 2019
%
%% user parameters
% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 10 ;
bounds = [-4,4,-2,2] ;
goal_radius = 0.5 ;

% planner limit
desired_speed = 0.2 ; % m/s (THIS HAS TO BE SLOW ENOUGH FOR THE PLANNING MODEL OF THE TURTLEBOT)
t_plan = 0.5 ;
t_move = 0.5 ;
initialize_tree_mode = 'iter' ; % 'iter' or 'once'
grow_tree_mode = 'seed' ; % 'new' or 'seed' or 'keep'

% simulation
max_sim_iterations = 1000 ;
max_sim_time = 100 ; % real amount of time to simulate
verbose_level = 5 ;
plot_HLP_flag = true ;

%% automated from here
A = fastrack_agent ;

buffer = A.LLC.TEB.TEB + A.footprint ;

% world
W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,...
    'verbose',verbose_level,'goal_radius',goal_radius,...
    'obstacle_size_bounds',obstacle_size_bounds,...
    'buffer',buffer) ;

% fastrack planner
P = turtlebot_RRT_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan,'t_move',t_move,'desired_speed',desired_speed,...
    'plot_HLP_flag',plot_HLP_flag,...
    'initialize_tree_mode',initialize_tree_mode,...
    'HLP_grow_tree_mode',grow_tree_mode) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
    'max_sim_time',max_sim_time,'max_sim_iterations',max_sim_iterations,'plot_while_running',1) ;

%% run simulation
S.run()
