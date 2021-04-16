# New README
## Dependencies
Simulator: https://github.com/skousik/simulator

RTD(yes the deprecated one): https://github.com/ramvasudevan/RTD

RTD_tutorial: https://github.com/skousik/RTD_tutorial

Spotless(and remember to install after clone): https://github.com/spot-toolbox/spotless

helperOC: https://github.com/HJReachability/helperOC (Seems like sylvia merged to master branch)

LevelSet Toolbox: https://www.cs.ubc.ca/~mitchell/ToolboxLS/

## Modes
To run simulation, use make_stamped_obs_scene.m 
1. Fastrack(Simon thinks it really works)
2. Stamped HJB(Im not so sure, I think most of the time it is avoiding obstacles, but without some gradient towards the goal, the safety controller doesn't always work well?)
3. RTD with RRT HLP (works)
4. RTD with straight line HLP (works)

## Simulator
The simulator has its own readme, but you need a agent, a planner, a world and a simulator file to run properly. Using the simulator can greatly simplify logging and visualization. They each have their conventions and parent class. so when you think the functions are not called anywhere, look inside the parent class.  @ Shreyas add more here

## Folders & Files
***dynSys*** saves the important dynamic systems, the only one there is the one I worked on long before for Fastrack

***Precomputation*** saves files for offline computation for HJB. 

*avoidfunction* came from Sylvia

*avoidfunction_boxes* is Simon trying to make boxes obstacles for the Unicycle4DRelDubins dynamics

*P4D_Q2D_RS_SIMON* is the only other useful file for fastrack offline computation. Not sure this will run since it's old

***running_simulation*** hosts online files, feel free to explore, but the useful ones are listed below. Maybe @Shreyas can fill in more below

*make_stamped_obs_scene* is the main one we need to focus on, runs comparison between the three methods, well, 4 since there are two RTDs.

*make_fastrack_conservatism_comparison* runs the comparison that Sylvia showed in her presentation

*run_fastrack_test* does a simple reference of a right angle for fastrack. I'm not so sure why Jason is having the error. I think it is mostly due to integrator issue, try another one see if that works A.integrator_type = 'ode45' , 'ode4', 'ode113'

***simulator_files*** hosts files needed for online and simulation

*Dubin4D2.0_0.3_40_vhigh_debugged* is TEB file, 2.0m/s is max tracking speed, 0.3 m/s is max planning speed, 40 is grid size, and vhigh is gradient accuracy.

*fastrack_agent* is the agent for HBJ based methods. The most important function is *dynamics*
1.  Get control input from a Low Level Controller(LLC) based on evaluation of offline computed file.
2.  Based on how close the value function is close to a threshold value, decide if we should use performance controller or just safety controller.

*fastrack_LLC* and *fastrack_avoid_LLC* hosts HJB files and their online evaluation

# Old README
Prereq:
To run online: 
1. you need helperOC
https://github.com/HJReachability/helperOC.git
and switch to branch sylvia_dev_updates
2. RTD repo
3. RTD_tutorial
4. simulator
To run precomputation:
You need levelset Toolbox
https://bitbucket.org/ian_mitchell/toolboxls/src/default/


Run offline:
P4D_Q2D_RS_SIMON

Run test:
run_fastrack_test

Run in a static obstacle env:
run_turtlebot_fastrack_simulation

1. Used in precomputation of value function:
P3D_Q2D_RS_SIMON is for  prusuer as a airplane with constant velocity 4m/s and turning speed w_max = 4rad/s arbitary planner that can plan at most 0.1 m away in both x and y dir
this uses the dynsys @P3D_Q2D_Rel in New_HelperOC/dynSys

P4D_Q2D_RS_SIMON is for pursuer same as shreyas' simulator and arbitary planner that can plan at most 0.2 m away in both x and y dir
this uses the dynsys @Unicycle4DRelDubins in New_HelperOC/dynSys

P5D_Q2D_RS_SIMON is for pursuer same as shreyas' simulator and a planner that plans with dx/dt=k2+k1*dy  dy/dt = k1*dx
this uses the dynsys @Unicycle5DRelDubins in New_HelperOC/dynSys

2. Simulator related files
P3D_Q2D_RS_SIMON (not tested recently)
correspond to turtlebot_3D_agent and LLC in simulator_files.
LLC needs the file Dubin3D50_dt010_tMax_converge.mat for lookup and optCtrl @P3D_Q2D_Rel class to evaluate control
TEB = 1.19m

P4D_Q2D_RS_SIMON 
correspond to fastrack_agent and LLC in simulator_files.
LLC needs the file Dubin4D2.0_0.3_40_vhigh_debugged.mat for lookup and optCtrl @Unicycle4DRelDubins class to evaluate control

P5D_Q2D_RS_SIMON
This agent is under construnction


Notes:
1. Currently safety controller and performance controller are both enabled for both 3D system and 4D system
to disable 4D system weighted controller, set A.use_performance = 0.







