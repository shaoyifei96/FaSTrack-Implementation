
classdef Unicycle4DRelDubins < DynSys
%dynamics of unicycle pursuing dubin's car
  properties
    %low d model limits
    p1_lim
    p2_lim
    %hi d model limits
    a_max%same acc +/-
    w_max %same angular v +/-
    v_max

  end

  methods
    function obj = Unicycle4DRelDubins(x, a_max, w_max, p1_lim, p2_lim,v_max)
      % obj = Mass4DRelDubins(x, Mass4D, KinVeh2D)
      %
      % Constructor. Creates the dynamical system object with state x and
      % parameters from the input parameters
      %
      % Inputs:                   mix   mix     
      %   x           - state: [del x, del y, del theta v]
      % Output:
      %   obj         - Mass4DRelDubins object
      %
      % Note: dynamics specified in dynamics.m

      if numel(x) ~= 4
        error('Initial state does not have right dimension!');
      end

      if ~iscolumn(x)
        x = x';
      end

      obj.x = x;
      obj.xhist = obj.x;

      obj.a_max = a_max;
      obj.w_max = w_max

      obj.p1_lim = p1_lim
      
      obj.p2_lim = p2_lim

      obj.pdim = [1,2];% position dimensions
      obj.hdim = [3];% heading dimensions
      obj.vdim = [4];% velocity dimensions
    
      obj.v_max = v_max;
      
      obj.nx = 4;
      obj.nu = 2;
      obj.nd = 2;
    end

  end % end methods
end % end classdef