% clear ; clc ;
%% description
% This script runs a simulation with the TurtleBot in the simulator
% framework, using RRT* to plan online.
%
%can use parallel computing to make things go faster, but RTD cannot run parallel since it has curly braces in { } in the dynamics. 
%parfor only support brackets()
%
% Author: Shreyas Kousik and Simon Shao
% Created: 19 Oct 2019
% Updated: 30 Oct 2019
% Updated by Simon: 18 Nov 2019
%
tebfile=load('Dubin4D2.0_0.3_40_vhigh_debugged.mat');
obs_array = [15];
for iii = 1:length(obs_array)
    N_obstacles= obs_array(iii);
%% user parameters
% agent
desired_speed = 0.3; % m/s


% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
bounds = [-6,6,-2,2] ;
goal_radius = 0.5 ;

% planner limit
% planner step limit: take a step with radius 0.69 m, the controller will
% get u to be with in 0.49m of the goal position. so to be connservative,
% take a step of no further than 0.49 m everytime. SS
% TEB = 0.49 m
t_plan_fas = 1;
t_move_fas = 1;
t_plan = 0.5 ; % if t_plan = t_move, then real time planning is enforced
t_move = 0.5 ; %making these values big will make the controller not work for some reason, there might be a bug
%keeps reducing stepsize.  SS
%works well when target is always 0.49 m away from current state, when they
%are close(t_move is too long), numerical instability occur...

% turtlebot RRT planner parameters
initialize_tree_mode = 'once' ; % 'iter' or 'once'
HLP_grow_tree_mode = 'new' ; % 'new' or 'seed' or 'keep' (only matters if using 'iter' above)
grow_tree_once_timeout = 4;
HLP_type = 'RRT*' ; % 'rrt' or 'rrt*' or 'connect' or 'connect*'

% simulation
sim_start_idx = 1;
sim_end_idx = 150 ;
verbose_level = 0 ;
plot_HLP_flag = true ;
plot_simulator_flag = true;

% file i/o
save_summaries_flag = true ;
save_file_location = './' ;

%% automated from here
A1 =  turtlebot_agent;
A2 = fastrack_agent(tebfile);
% A2.LLC.TEB.sD.dynSys.v_max =1.5;
A2.LLC.TEB.TEBadj = 0.25;
A2.LLC.TEB.TEB = 0.25;
% tried both ode4 and ode 113, all don't work really well, also produce
% unsmooth trajectory. Part of the reason is due to that fastrack is just a
% safety controller, there needs to be a performance controller working together
% with it.
%

% put agents together
A_together = {A1 A2} ;
% A_together = A1 ;
% A_together = A2 ;

buffer = A2.LLC.TEB.TEB + A2.footprint ; % m. obs augmented by teb so if planning along the edge of augmented obs,
%real agent doesn't hit actual obs despite traching error. SS

% RTD planner
HLP = RRT_star_HLP('grow_tree_mode','new') ;
P1 = turtlebot_RTD_planner_static('verbose',verbose_level,'buffer',0.01,...
    't_plan',t_plan,'t_move',t_move,'HLP',HLP,...
    'plot_HLP_flag',plot_HLP_flag) ;

% fastrack planner
P2 = turtlebot_RRT_planner('verbose',verbose_level,'buffer',buffer,...
    't_plan',t_plan_fas,'t_move',t_move_fas,'desired_speed',desired_speed,...
    'plot_HLP_flag',plot_HLP_flag,...
    'HLP_type',HLP_type,...
    'initialize_tree_mode',initialize_tree_mode,...
    'grow_tree_once_timeout',grow_tree_once_timeout,...
    'HLP_grow_tree_mode',HLP_grow_tree_mode) ;

P_together = {P1  P2} ;
% A_together = A2 ;
% P_together = P2 ;


%% run many simulations
for idx = sim_start_idx:sim_end_idx
    idx
    W = static_box_world('bounds',bounds,'N_obstacles',N_obstacles,...
        'verbose',verbose_level,'goal_radius',goal_radius,...
        'obstacle_size_bounds',obstacle_size_bounds,...
        'buffer',buffer) ;
%         W = data.W ;
    S = simulator(A_together,W,P_together,'allow_replan_errors',false,'verbose',verbose_level,...
        'max_sim_time',100,'max_sim_iterations',80,'plot_while_running',plot_simulator_flag) ;
    S.worlds{1} = W;
     %try
        S.run()
        summary = S.simulation_summary ;

        % generate filename
        save_filename = [save_file_location,'trial_',num2str(N_obstacles),'_',num2str(idx,'%03.f')] ;

        if save_summaries_flag
            parsave(save_filename,W,A_together,P_together,summary)
        end

%     catch ME
%         save_filename = [save_file_location,'trial_',num2str(N_obstacles),'_',num2str(idx,'%03.f'),'ERROR'] ;
%          disp('simulator errored!')
%          summary = struct;
%          parsave(save_filename,W,A_together,P_together,summary)
%          continue;
%     end
    
end
end
function parsave_simple(fname,C )
    save('-mat',fname,'C');
end
function parsave(fname, W,A_together,P_together,summary)
save(fname, 'W','A_together','P_together','summary','-v7.3')
end
