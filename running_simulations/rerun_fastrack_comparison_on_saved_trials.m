%% description
% This script runs through all the .mat files containing summaries in a
% folder, and re-runs them with a FasTrack agent/planner.
%
% Author: Shreyas Kousik%
% Created: 31 Oct 2019
% Updated: 31 Oct 2019
%
%% user parameters
% agent
desired_speed = 1 ; % m/s

% world
obstacle_size_bounds = [0.2, 0.3] ; % side length [min, max]
bounds = [-4,4,-2,2] ;
start = [-3;0] ;
goal = [3.5;0] ;
goal_radius = 0.5 ;
box_size = 0.25 ;

% planner time limits
t_plan = 0.5 ;
t_move = 0.5 ;

% simulation
verbose_level = 5 ;
plot_HLP_flag = true ;

% file i/o
save_summaries_flag = true ;
save_file_location = '~/MATLAB/fastrack_comparison_data/' ;

%% (automated from here)
% get all files
files = dir(save_file_location) ;

for idx = 3:length(files)
    n = files(idx).name ;
    if length(n) > 3 && strcmpi(n(end-2:end),'mat')
        % load the summary
        data = load(files(idx).name) ;
        summary = data.summary ;
        
        % make a fastrack agent
        A = fastrack_agent ;
        
        % make a fastrack planner
        fastrack_buffer = A.LLC.TEB.TEB + A.footprint ;
        P = turtlebot_RRT_planner('verbose',verbose_level,'buffer',fastrack_buffer,...
            't_plan',t_plan,'t_move',t_move,'desired_speed',desired_speed,...
            'plot_HLP_flag',plot_HLP_flag) ;
        
        % make a world
        try
            W = data.W ;
        catch
            W = static_box_world('bounds',bounds,'N_obstacles',0,...
                'verbose',verbose_level,'goal_radius',goal_radius,...
                'obstacle_size_bounds',obstacle_size_bounds,...
                'buffer',fastrack_buffer) ;
            
            % fill in the world's start, goal, and obstacles
            W.bounds = summary(1).bounds ;
            W.start = summary(1).start ;
            W.goal = summary(1).goal ;
            W.obstacles = summary(1).obstacles ;
            
            % run world setup
            W.setup() ;
        end
        
        % make the simulator
        S = simulator(A,W,P,'allow_replan_errors',false,'verbose',verbose_level,...
            'max_sim_time',30,'max_sim_iterations',1000,'plot_while_running',1) ;
        
        % run the simulation
        S.run()
        
        % get simulation summary
        summary = S.simulation_summary ;
        
        % generate filename
        save_filename = [save_file_location,'trial_fastrack_only_',num2str(idx,'%03.f'),'.mat'] ;
        
        if save_summaries_flag
            save(save_filename,'summary','W')
        end
    end
end