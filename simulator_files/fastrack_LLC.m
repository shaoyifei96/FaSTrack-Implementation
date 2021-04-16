classdef fastrack_LLC < low_level_controller
    properties
        % control gains
        position_gain = 1 ;
        speed_gain = 3 ;
        accel_gain = 0 ;
        yaw_gain = 0 ;
        yaw_rate_gain = 1 ; %not useful constants for fastrack
        TEB;% = load("Dubin4D2.0_0.3_40_vhigh_debugged.mat");
        Q = [1 0; 0 1; 0 0; 0 0];
        TEBadj = 0.21;
        
        
    end
    
    methods
        %% constructor
        function LLC = fastrack_LLC(TEB,varargin)
            n_agent_states = 4 ;
            n_agent_inputs = 2 ;
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
            LLC.TEB = TEB;
        end
        
        %% get control inputs
        function [U, TEB_exp] = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
            % get current state
            h_cur = z_cur(A.heading_index);
            v_cur = z_cur(A.speed_index) ;
            
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
                
                if d_along_plan(end) > 0
%                     % get closest point to current agent location
%                     d_cur = dist_point_on_polyline(z_cur(A.position_indices),X_des) ;
% 
%                     % get distance along traj to interpolate
%                     d_lookahead = 0.6 ; % per the comments
%                     d_des = min(d_cur + d_lookahead, d_along_plan(end)) ;
 
%                     d_des = 0.28 * 0.1 ;
% 
%                     % get desired state 0.49 m ahead on trajectory
%                     z_des = match_trajectories(d_des,d_along_plan,Z_des) 
                    z_des = match_trajectories(t_cur + 0.1, T_des, Z_des) ;
                else
                    z_des = Z_des(:,end) ;
                end
                % z_des = match_trajectories(t_cur,T_des,Z_des) ;
            end

%              z_des=[1;-1];
             z_des=LLC.Q*[z_des(1); z_des(2)];
             
             rel_z = z_cur - z_des;% Fastrack controller evaluate value function and gradient based on relative state
             % get relative state here
             %rel_z=[0.5;0.1;0;0];
             TEB_exp = sqrt((rel_z(1)^2+rel_z(2)^2)); %evaluate value func
%              TEB_exp = ; %make sure relative state doesn't exceed
             %teb, since we are choosing the next planned state, it can be
             %arbitarily close to the previous one to ensure our teb lookup
             %table doesn't go out of bound. SS
%           
             % Interpolate b/w grid points, what we want is the controller,
             % so only the sign of deriv{3},{4} is necessary to compute the
             % controller, but we still get the value of all deriv, which is
             % ok. SS
             deriv_Intropolated = eval_u(LLC.TEB.sD.grid, LLC.TEB.deriv, rel_z);
             uMode = 'min';
             % this is same controller function as the pursuit game,
             % controller tries to minimize the rel err, only
             U = LLC.TEB.sD.dynSys.optCtrl([],rel_z,deriv_Intropolated,uMode);
             U= [U{1};U{2}];

        end
    end
end