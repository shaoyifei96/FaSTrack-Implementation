%% description
% This script runs through all the .mat files containing summaries in a
% folder, and saves the number of collisions and goals for each planner.
%
% Author: Shreyas Kousik%
% Created: 30 Oct 2019
% Updated: 31 Oct 2019
%
%% user parameters
save_file_location = '~/MATLAB/fastrack_comparison_data/' ;

%% automated from here
files = dir(save_file_location) ;

N_trials = 0 ;

RTD_goals = [] ;
RTD_collisions = [] ;
RTD_peak_speed = [] ;
RTD_time_to_goal = [] ;

fastrack_goals = [] ;
fastrack_collisions = [] ;
fastrack_peak_speed = [] ;
fastrack_time_to_goal = [] ;

%% load and extract data
for idx = 3:length(files)
    n = files(idx).name ;
    if length(n) > 3 && strcmpi(n(end-2:end),'mat')
        data = load(files(idx).name) ;
        summary = data.summary ;
        
        % get RTD data
        RTD_goal = summary(1).goal_check ;
        RTD_goals = [RTD_goals, RTD_goal] ;
        RTD_collisions = [RTD_collisions, summary(1).collision_check] ;
        if RTD_goal
            RTD_time_to_goal = [RTD_time_to_goal, summary(1).total_simulated_time(end)] ;
        else
            RTD_time_to_goal = [RTD_time_to_goal, nan] ;
        end
        RTD_peak_speed = [RTD_peak_speed, max(summary(1).agent_info.state(4,:))] ;
        
        % get fastrack data
        fastrack_goal = summary(2).goal_check ;
        fastrack_goals = [fastrack_goals, fastrack_goal] ;
        fastrack_collisions = [fastrack_collisions, summary(2).collision_check] ;
        if fastrack_goal
            fastrack_time_to_goal = [fastrack_time_to_goal, summary(2).total_simulated_time(end)] ;
        else
            fastrack_time_to_goal = [fastrack_time_to_goal, nan] ;
        end
        fastrack_peak_speed = [fastrack_peak_speed, max(summary(2).agent_info.state(4,:))] ;
        
        % increment
        N_trials = N_trials + 1 ;
        
        if mod(N_trials,10) == 0
            disp(['# of files inspected: ',num2str(N_trials)])
        end
    end
end

%% analyze data
clc
disp('-- RTD --')
disp(['RTD goals: ',num2str(100*sum(RTD_goals)/N_trials,'%0.1f'),' %'])
disp(['RTD collisions: ',num2str(100*sum(RTD_collisions)/N_trials,'%0.1f'),' %'])
disp(['RTD peak speed: ',num2str(mean(RTD_peak_speed),'%0.1f'),' m/s'])
disp(['RTD time to goal: ',num2str(mean(RTD_time_to_goal,'omitnan'),'%0.1f'),' s'])

disp(' ')
disp('-- FasTrack --')
disp(['FasTrack goals: ',num2str(100*sum(fastrack_goals)/N_trials,'%0.1f'),' %']) ;
disp(['FasTrack collisions: ',num2str(100*sum(fastrack_collisions)/N_trials,'%0.1f'),' %']) ;
disp(['FasTrack peak speed: ',num2str(mean(fastrack_peak_speed),'%0.1f'),' m/s'])
disp(['FasTrack time to goal: ',num2str(mean(fastrack_time_to_goal,'omitnan'),'%0.1f'),' s'])
