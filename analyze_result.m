clear all; close all
clc;
fatrack_crashes=0;
RTD_crashes=0;
fastrack_goal=0;
RTD_goal=0;
fastrack_time=[];
RTD_time=[];



for i = 1:100
    try
        load("simulation_summary"+num2str(i)+".mat")
                if summary(1).goal_check
                    RTD_goal = RTD_goal+1
                    RTD_time=[RTD_time; summary(1).sim_time_vector(end)]
                end
               
                RTD_crashes = RTD_crashes+summary(1).crash_check;
                fatrack_crashes = fatrack_crashes+summary(2).crash_check

                if summary(2).goal_check
                    fastrack_goal = fastrack_goal+1
                    fastrack_time=[fastrack_time; summary(2).sim_time_vector(end)]
                end
                    
%                 if summary(1).crash_check
%                     RTD_crashes = RTD_crashes+1
%                 elseif summary(1).goal_check
%                     RTD_goal = RTD_goal+1
%                     RTD_time=[RTD_time; summary(1).sim_time_vector(end)]
%                 else
%                     
%                 end
%         
%                 if summary(2).crash_check
%                     fatrack_crashes = fatrack_crashes+1
%                 elseif summary(2).goal_check
%                     fastrack_goal = fastrack_goal+1
%                     fastrack_time=[fastrack_time; summary(2).sim_time_vector(end)]
%                 else
%                     
%                 end
        
    catch ME
        continue
    end
    
end
%%
RTD_avg_time = mean(RTD_time)
fastrack_avg_time = mean(fastrack_time)