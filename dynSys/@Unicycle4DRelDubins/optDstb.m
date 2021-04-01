function dOpt = optDstb(obj, t, x, deriv, dMode, ~)
% dOpt = optDstb(obj, t, x, deriv, ~, ~)

% Note: dynamics specified in dynamics.m

%% Input processing
if nargin < 5
  dMode = 'min';
end

if iscell(deriv)
  if strcmp(dMode, 'max')
      dOpt = cell(obj.nd, 1);
      dOpt{1} = sign(deriv{1}).*obj.p1_lim;
      dOpt{2} = sign(deriv{2}).*obj.p2_lim;
  elseif strcmp(dMode, 'min')
      dOpt = cell(obj.nd, 1);
      dOpt{1} = -sign(deriv{1}).*obj.p1_lim;
      dOpt{2} = -sign(deriv{2}).*obj.p2_lim;
  else
    error('Unknown dMode!')
  end

else
     error('deriv not cell')

end


end