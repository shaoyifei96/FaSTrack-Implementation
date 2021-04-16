classdef fastrack_avoid_LLC < low_level_controller
    properties
        % control gains
        position_gain = 1 ;
        speed_gain = 3 ;
        accel_gain = 0 ;
        yaw_gain = 0 ;
        yaw_rate_gain = 1 ; %not useful constants for fastrack
        TEB;% = load("Dubin4D2.0_0.3_40_vhigh_debugged.mat");
        Q = [1 0; 0 1; 0 0; 0 0];
        TEBadj = 1.0;
        
        
    end
    
    methods
        %% constructor
        function LLC = fastrack_avoid_LLC(TEB,varargin)
            n_agent_states = 4 ;
            n_agent_inputs = 2 ;
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
            LLC.TEB = TEB;
        end
        
        %% get control inputs
        function [U, TEB_exp] = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
                %control, value function
            % get current state
            h_cur = z_cur(A.heading_index);
            v_cur = z_cur(A.speed_index) ;
            TEB_exp = 0;
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des) % I am assume this doesn't happen ..SS
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t_cur,T_des,U_des,'previous') ;
                v_des = 0 ;
                h_des = h_cur ;
            else
                % otherwise, we are doing feedback about a desired
                % trajectory
                
                % get distances along planned trajectory
                X_des = Z_des(1:2,:) ;
                d_along_plan = dist_polyline_cumulative(X_des) ;
                
            end
            %directly evaluate the value function from current state,
            %signed dist to obstacle
             TEB_exp = eval_u(LLC.TEB.schemeData.grid, LLC.TEB.Value, z_cur);
             
             deriv_Intropolated = eval_u(LLC.TEB.schemeData.grid, LLC.TEB.Deriv, z_cur);
             uMode = 'max';%here value function means how close to obs, so get away!
             U = LLC.TEB.schemeData.dynSys.optCtrl([],z_cur,deriv_Intropolated,uMode);
             U= [U{1};U{2}];

        end
    end
end