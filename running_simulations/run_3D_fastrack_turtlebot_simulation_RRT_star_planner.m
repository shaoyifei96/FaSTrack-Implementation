%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RRT* to plan online.
%
% Author: Shreyas Kousik
% Created: 19 Oct 2019
% Edited: Yifei Simon Shao, 19 Oct 2019 
%% user parameters
% agent
desired_speed = 0.2 ; % m/s

% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 3;
bounds = [-8,8,-4,4] ;
goal_radius = 0.5 ;

% planner limit
% planner step limit: take a step with radius 0.69 m, the controller will
% get u to be with in 0.49m of the goal position. so to be connservative,
% take a step of no further than 0.49 m everytime. SS
% TEB = 0.49 m  
t_plan = 1; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5; %making these values big will make the controller not work for some reason, there might be a bug
%keeps reducing stepsize.  SS
%works well when target is always 0.49 m away from current state, when they
%are close(t_move is too long), numerical instability occur...  
% simulation
verbose_level = 0 ;

%% automated from here
A = turtlebot_3D_agent ;
%AB = fastrack_agent ;% tried both ode4 and ode 113, all don't work really well, also produce 
%unsmooth trajectory. Part of the reason is due to that fastrack is just a
%safety controller, there needs to be a performance controller working together
%with it.
%

% this is needed to the agent to track the RRT* output
A.LLC.yaw_gain = 10 ;
A.LLC.lookahead_time = 0.05 ;
% AB.LLCP.yaw_gain = 10 ;
% AB.LLCP.lookahead_time = 0.05 ;
buffer = 0.3;
% buffer = AB.LLC.TEB.TEB ; % m. obs augmented by teb so if planning along the edge of augmented obs, 
% %real agent doesn't hit actual obs despite traching error. SS 

P = turtlebot_RRT_star_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan,'t_move',t_move,'desired_speed',desired_speed) ;
W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,'buffer',0.25,...
                     'verbose',verbose_level,'goal_radius',goal_radius,...
                     'obstacle_size_bounds',obstacle_size_bounds) ;

S = simulator(A,W,P,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',1000,'max_sim_iterations',1000) ;

%% run simulation
S.run() ;