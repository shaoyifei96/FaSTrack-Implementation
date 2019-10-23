classdef turtlebot_3D_LLC < low_level_controller
    properties
        % lookahead time
        lookahead_time = 0 ;
        
        % control gains
        position_gain = 1 ;
        speed_gain = 3 ;
        accel_gain = 0 ;
        yaw_gain = 0 ;
        yaw_rate_gain = 1 ;
        TEB = load("Dubin3D50_dt010_tMax_converge.mat");
        Q = [1 0; 0 1; 0 0];
    end
    
    methods
        %% constructor
        function LLC = turtlebot_3D_LLC(varargin)
            n_agent_states = 3 ;
            n_agent_inputs = 1 ;
            
            LLC = parse_args(LLC,'n_agent_states',n_agent_states,...
                'n_agent_inputs',n_agent_inputs,varargin{:}) ;
        end
        
        %% get control inputs
        function U = get_control_inputs(LLC,A,t_cur,z_cur,T_des,U_des,Z_des)
            % get current state
            h_cur = z_cur(A.heading_index) ;
            
            % get time along traj to use for feedback
            t_fdbk = min(t_cur + LLC.lookahead_time, T_des(end)) ;
            
            % get desired state and inputs (assumes zero-order hold)
            if isempty(Z_des)
                % if no desired trajectory is passed in, then we assume
                % that we are emergency braking
                u_des = match_trajectories(t_fdbk,T_des,U_des,'previous') ;
                v_des = 0 ;
                h_des = h_cur ;
            else
                % otherwise, we are doing feedback about a desired
                X_des = Z_des(1:2,:) ;
                d_along_plan = dist_polyline_cumulative(X_des) ;

                % get closest point to current agent location
                d_cur = dist_point_on_polyline(z_cur(A.position_indices),X_des) ;

                % get distance along traj to interpolate
                d_lookahead = 0.2 ; % per the comments
                d_des = min(d_cur + d_lookahead, d_along_plan(end)) ;

                % get desired state 0.49 m ahead on trajectory
                z_des = match_trajectories(d_des,d_along_plan,Z_des) ;
            end
%             z_des=[-6; 0];
            z_des=LLC.Q*[z_des(1); z_des(2)];
            rel_z =  z_cur-z_des
           normalizer = sqrt((rel_z(1)^2+rel_z(2)^2))/ LLC.TEB.TEB; 
           if normalizer > 1
                % Don't know if this intropolation is valid....
                rel_z(1) = rel_z(1)/normalizer;
                rel_z(1) = rel_z(2)/normalizer;
           end
             deriv_Intropolated = eval_u(LLC.TEB.sD.grid, LLC.TEB.deriv, rel_z);
            
            uMode = 'min';
            % this is same controller function as the pursuit game,
            % controller tries to minimize the rel err, only
            U = LLC.TEB.sD.dynSys.optCtrl([],rel_z,deriv_Intropolated,uMode);
        end
    end
end