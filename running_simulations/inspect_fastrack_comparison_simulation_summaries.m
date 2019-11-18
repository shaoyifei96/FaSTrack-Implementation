%% description
% This script runs through all the .mat files containing summaries in a
% folder, and saves the number of collisions and goals for each planner.
%
% Author: Shreyas Kousik%
% Created: 30 Oct 2019
% Updated: 1 Nov 2019
%
%% user parameters
save_file_location = './result' ;

%% automated from here
files = dir(save_file_location) ;

N_trials = 0 ;

goals = [] ;
collisions = [] ;
peak_speed = [] ;
time_to_goal = [] ;

%% load and extract data
for file_idx = 3:length(files)
    file_name = files(file_idx).name ;

    if length(file_name) > 3 && strcmpi(file_name(end-2:end),'mat')
        data = load(files(file_idx).name) ;

        summary = data.summary ;
        
        % add a column to the data
        n_planners = length(summary) ;
        goals = [goals, nan(n_planners,1)];
        collisions = [collisions, nan(n_planners,1)] ;
        peak_speed = [peak_speed, nan(n_planners,1)] ;
        time_to_goal = [time_to_goal, nan(n_planners,1)] ;
        
        % get the data
        for idx = 1:n_planners
            goals(idx,end) = summary(idx).goal_check ;
            collisions(idx,end) = summary(idx).collision_check ;
            peak_speed(idx,end) = max(summary(idx).trajectory(4,:)) ;
            time_to_goal(idx,end) = summary(idx).total_simulated_time(end) ;
        end
        
        % increment
        N_trials = N_trials + 1 ;
        
        if mod(N_trials,10) == 0
            disp(['# of files inspected: ',num2str(N_trials)])
        end
    end
end

%% analyze data
clc

for idx = 1:n_planners
    disp(summary(idx).planner_name)
    disp(['Goals: ',num2str(100*sum(goals(idx,:))/N_trials,'%0.1f'),' %'])
    disp(['Collisions: ',num2str(100*sum(collisions(idx,:))/N_trials,'%0.1f'),' %'])
    disp(['Peak Speed: ',num2str(mean(peak_speed(idx,:)),'%0.1f'),' m/s'])
    disp(['Time to Goal: ',num2str(mean(time_to_goal(idx,:),'omitnan'),'%0.1f'),' s'])
    disp(' ')
end

%% plot any RTD crashes
RTD_crash_idxs = find(collisions) ;

A = turtlebot_agent ;

% for idx = RTD_crash_idxs
file_idx = 311 ;
data = load(files(file_idx).name) ;
summary = data.summary ;

% set up agent
A.state = summary(1).agent_info.state ;
A.time = summary(1).agent_info.time ;

% set up world
W = static_box_world() ;
W.start = summary(1).start ;
W.goal = summary(1).goal ;
W.obstacles = summary(1).obstacles ;
W.obstacles_seen = W.obstacles ; 

% plot
figure(1) ; clf ; axis equal ;
plot(W)
plot(A)