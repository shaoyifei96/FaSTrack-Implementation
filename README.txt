To run FasTrack:
Prereq:
To run online: 
1. you need helperOC
https://github.com/HJReachability/helperOC/tree/sylvia_dev_updates
2. RTD repo
3. RTD_tutorial
4. simulator
To run precomputation:
You need levelset Toolbox
https://bitbucket.org/ian_mitchell/toolboxls/src/default/




1. Use things in precomputation:
P3D_Q2D_RS_SIMON is for  prusuer as a airplane with constant velocity 4m/s and turning speed w_max = 4rad/s arbitary planner that can plan at most 0.1 m away in both x and y dir
this uses the dynsys @P3D_Q2D_Rel in New_HelperOC/dynSys

P4D_Q2D_RS_SIMON is for pursuer same as shreyas' simulator and arbitary planner that can plan at most 0.2 m away in both x and y dir
this uses the dynsys @Unicycle4DRelDubins in New_HelperOC/dynSys

P5D_Q2D_RS_SIMON is for pursuer same as shreyas' simulator and a planner that plans with dx/dt=k2+k1*dy  dy/dt = k1*dx
this uses the dynsys @Unicycle5DRelDubins in New_HelperOC/dynSys

2. Simulator related things
P3D_Q2D_RS_SIMON 
correspond to turtlebot_3D_agent and LLC in simulator_files.
LLC needs the file Dubin3D50_dt010_tMax_converge.mat for lookup and optCtrl @P3D_Q2D_Rel class to evaluate control

P4D_Q2D_RS_SIMON 
correspond to fastrack_agent and LLC in simulator_files.
LLC needs the file Dubin4D30_dt010_tMax_converge.mat for lookup and optCtrl @Unicycle4DRelDubins class to evaluate control
Dubin4D30_dt010_t100converge.mat has lower accuracy and is the one we are more familiar with 0.49 m

P5D_Q2D_RS_SIMON
This agent is under construnction



