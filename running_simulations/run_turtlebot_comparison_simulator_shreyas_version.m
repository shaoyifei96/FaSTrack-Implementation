clear ; clc ;
%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RRT* to plan online.
%
% Author: Shreyas Kousik and Simon Shao
% Created: 19 Oct 2019
% Updated: 30 Oct 2019
%
%% user parameters
% agent
desired_speed = 0.2; % m/s

% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
N_obstacles = 10;
bounds = [-6,6,-3,3] ;
goal_radius = 0.5 ;

% planner limit
% planner step limit: take a step with radius 0.69 m, the controller will
% get u to be with in 0.49m of the goal position. so to be connservative,
% take a step of no further than 0.49 m everytime. SS
% TEB = 0.49 m  
t_plan_fas = 15;
t_move_fas =100;
t_plan = 0.5; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ; %making these values big will make the controller not work for some reason, there might be a bug
%keeps reducing stepsize.  SS
%works well when target is always 0.49 m away from current state, when they
%are close(t_move is too long), numerical instability occur...  

% simulation
sim_start_idx = 1 ;
sim_end_idx = 200 ;
verbose_level = 0 ;
plot_HLP_flag = false ;
plot_simulator_flag = false;

% file i/o
save_summaries_flag = true ;
save_file_location = './' ;

%% automated from here
A1 =  turtlebot_agent;
A2 = fastrack_agent ;
A2.LLC.TEB.sD.dynSys.v_max = 0.48;
A2.LLC.TEB.TEB = 0.37;
% tried both ode4 and ode 113, all don't work really well, also produce 
% unsmooth trajectory. Part of the reason is due to that fastrack is just a
% safety controller, there needs to be a performance controller working together
% with it.
%

% this is needed to the agent to track the RRT* output
A1.LLC.gains.yaw = 10 ;
A1.LLC.lookahead_time = 0.1 ;

% put agents together
A_together = {A1 A2} ;

buffer = A2.LLC.TEB.TEB + A2.footprint ; % m. obs augmented by teb so if planning along the edge of augmented obs, 
%real agent doesn't hit actual obs despite traching error. SS 

% RTD planner
P1 = turtlebot_RTD_planner_static('verbose',verbose_level,'buffer',0.01,...
                                 't_plan',t_plan,'t_move',t_move,'HLP',RRT_HLP(),...
                                 'plot_HLP_flag',plot_HLP_flag) ;

% fastrack planner
P2 = turtlebot_RRT_star_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan_fas,'t_move',t_move_fas,'desired_speed',desired_speed,...
    'plot_HLP_flag',plot_HLP_flag) ;


 P_together = {P1 P2} ;
% A_together = A2 ;
% P_together = P2 ;

%% run many simulations
for idx = sim_start_idx:sim_end_idx
	idx
    W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,...
        'verbose',verbose_level,'goal_radius',goal_radius,...
        'obstacle_size_bounds',obstacle_size_bounds,...
        'buffer',buffer) ;
    
    S = simulator(A_together,W,P_together,'allow_replan_errors',true,'verbose',verbose_level,...
              'max_sim_time',90,'max_sim_iterations',1000,'plot_while_running',plot_simulator_flag) ;
    
    S.worlds{1} = W;
%     try
        S.run()
%     catch ME
%         disp('simulator errored!')
%         continue;
%     end
    summary = S.simulation_summary ;
    
    % generate filename
    save_filename = [save_file_location,'trial_',num2str(idx,'%03.f')] ;
    
    if save_summaries_flag
        save(save_filename,'summary','W')
    end
end
