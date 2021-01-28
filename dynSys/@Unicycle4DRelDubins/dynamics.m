function dx = dynamics(obj, ~, x, u, d, ~)
%
% Input constraints (for reference only)


if numel(u) ~= obj.nu
  error('Incorrect number of control dimensions!')
end


if iscell(x)
  delx=x{1};
  dely=x{2};
  ther=x{3};
  v   =x{4};

  p1  =d{1};
  p2  =d{2};

  uw  =u{1};
  uacc  =u{2};

  dx = cell(obj.nx, 1);
  dx{1}   = v.*cos(ther)+p1;
  dx{2}   = v.*sin(ther)+p2;
  dx{3}   = uw;
  dx{4}   = uacc;


else
  error('state not cell type')
end


end