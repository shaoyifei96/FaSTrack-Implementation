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

fastrack_goals = [] ;
fastrack_collisions = [] ;

%% load and extract data
for idx = 3:length(files)
    n = files(idx).name ;
    if length(n) > 3 && strcmpi(n(end-2:end),'mat')
        data = load(files(idx).name) ;
        summary = data.summary ;
        
%         if summary(1).collision_check
%             dbstop in inspect_fastrack_comparison_simulation_summaries at 35
%         end
        
        % get RTD data
        RTD_goals = [RTD_goals, summary(1).goal_check] ;
        RTD_collisions = [RTD_collisions, summary(1).collision_check] ;
        
        fastrack_goals = [fastrack_goals, summary(2).goal_check] ;
        fastrack_collisions = [fastrack_collisions, summary(2).collision_check] ;
        
        N_trials = N_trials + 1 ;
    end
end

%% analyze data
sum(RTD_goals)/N_trials
sum(fastrack_goals)/N_trials
sum(RTD_collisions)/N_trials
sum(fastrack_collisions)/N_trials
