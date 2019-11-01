%% Safe Nonholonomic Dynamics
% Get the nonholonomic dynamics value at (q,p). If the nh function returns
% a NaN or Inf, compute the holonomic value instead. The purpose of this
% function is to allow the nonholonomic dynamics to be used on the boundary
% of the region between nonholonomic and holonomic phase flow; it does NOT
% restrict where the nonholonomic dynamics may be used (e.g. this could
% allow the dynamics to be defined even if q < 0 or p < 0). If dynH and
% dynNH are to be swapped depending on the region, this function will NOT
% do that.
%
% Inputs:
% * q = coordinate to evaluate
% * p = momentum to evaluate
% * dynHfun = holonomic dynamics to be used when NH dynamics are NaN or Inf
% * dynNHfun = nonholonomic dynamics on which to evaluate (q,p).
%
% Output:
% * val = dynNHfun(q,p). If this is Inf or NaN, val = dynHfun(q,p)
%
% Usage:
% nhFun = @(q,p) safeNHDynamics(q,p,dynHfun,dynNHfun);
function val = safeNHDynamics(q,p,dynHfun,dynNHfun)
    % Compute the nonholonomic function
    val = dynNHfun(q,p);
    % If it's invalid, just use the holonomic dynamics instead because we
    % are on the boundary of where the nonholonomic dynamics are defined.
    if any(isnan(val)) | any(isinf(val))
        val = dynHfun(q,p);
    end
end