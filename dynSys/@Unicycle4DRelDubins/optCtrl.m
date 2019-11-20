function uOpt = optCtrl(obj, ~, x, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, x, deriv, uMode, dMode, MIEdims)

% Note: dynamics specified in dynamics.m

%% Input processing
if nargin < 5
	uMode = 'min';
end

%% Optimal control
if ~iscell(deriv)
  deriv = num2cell(deriv);
end
	if strcmp(uMode, 'min')
	uOpt = cell(obj.nu, 1);
    
	uOpt{1} =obj.w_max     * sign(deriv{3});
	%uOpt{2} =(x{4}<-1)*obj.a_max+(x{4}>1) *-obj.a_max +(abs(x{4})<=1)*obj.a_max .* sign(deriv{4});
    uOpt{2} =(x{4}<0)*obj.a_max+(x{4}>obj.max_spd ) *-obj.a_max +(abs(x{4})<=obj.max_spd )*obj.a_max .* sign(deriv{4});
    % uOpt{1} = deriv{3} ./ normalizer * obj.a_max_;
    % uOpt{2} = deriv{4} ./ normalizer * obj.a_max_;

	else
		error('Unknown uMode!(max not supported)')
	end


% 	uOpt = zeros(obj.nu, 1);
% 	val = (deriv(1) .*cos(x(3))+deriv(2).*sin(x(3)));
% 	uOpt(1) = obj.v_max_ * (val >= 0) + obj.v_min_ * (val < 0);
% 	uOpt(2) = obj.w_max_ * (deriv(3)>=0) + -obj.w_max_ * (deriv(3)>=0);
% 	if strcmp(uMode, 'max')
% 		uOpt(1) = -uOpt(1);
% 		uOpt(2) = -uOpt(2);
% 	elseif strcmp(uMode, 'min')

% 	else
% 		error('Unknown uMode!')
% 	end





end