classdef turtlebot_3D_agent < RTD_agent_2D
% Class: fastrack_agent < RTD_agent_2D < agent
%
% T
    
    properties
        integrator_type = 'ode45' ; 
        integrator_time_discretization = 0.01 ; % for ode4
        % state limits
        max_speed = 2 ; % m/s (NOTE this is higher than in the specs, since
                        % the planner should not command speeds above its
                        % own limit, but the robot is not limited by the
                        % planning algorithm)
        
        % spd
        v= 5;
                        
        % input limits (NOTE these are higher than in the paper as well,
        % and are based on guesses of what the TurtleBot does in YouTube
        % videos, where it can spin and accelerate really fast)
        max_yaw_rate = 2.0 ; % rad/s     
        max_accel = 2.0 ; % m/s^2   
        LLCP= []
    end
    
    methods
        %% constructor
        function A = turtlebot_3D_agent(varargin)
            % set up default superclass values
            default_footprint = 0.35/2 ;
            n_states = 3 ;
            n_inputs = 2 ;
            stopping_time = 3.75 ;
            sensor_radius = 4 ;

            LLC = turtlebot_3D_LLC ;
            
            % create agent
            A@RTD_agent_2D('footprint',default_footprint,...
                'n_states',n_states,'n_inputs',n_inputs,...
                'stopping_time',stopping_time,'sensor_radius',sensor_radius,...
                'LLC',LLC,varargin{:}) ;
        end
        
        %% emergency stop
        % note, this ignores any previous trajectory the agent may have
        % been tracking; we have to define this different from the default
        % for the TurtleBot because it takes in acceleration as a control
        % input, as opposed to doing feedback about desired speed
        
        
        %% dynamics
        function zd = dynamics(A,t,z,T,U,Z)
            % handle no desired trajectory input
            if nargin < 6
                Z = [] ;
            end
            
           u = A.LLC.get_control_inputs(A,t,z,T,U,Z);
           
            u(isnan(u)) = 0 ; % safety check
            xd = A.v*cos(z(3)) 
            yd = A.v*sin(z(3))
            hd = u

            
            % calculate the derivatives

            
            % return state derivative
            zd = [xd ; yd ; hd ] ;
        end
        function [tout,zout] = integrator(A,fun,tspan,z0)
            switch A.integrator_type
                case 'ode45'
                   opts = odeset('RelTol',1e-2,'AbsTol',1e-2);

                    [tout,zout] = ode45(@(t,z) fun(t,z),tspan,z0(:),opts) ;
                case 'ode113'
                    [tout,zout] = ode113(@(t,z) fun(t,z),tspan,z0(:)) ;
                case {'ode4','RK4'}
                    dt = A.integrator_time_discretization ;
                    tout = tspan(1):dt:tspan(end) ;
                    if tout(end) ~= tspan(end)
                        tout = [tout, tspan(end)] ;
                    end
                    zout = ode4(@(t,z) fun(t,z),tout,z0(:)) ;
                otherwise
                    error('Please set A.integrator_type to either ode45 or ode4')
            end
            tout = tout(:)' ;
            zout = zout' ;
        end
    end
end