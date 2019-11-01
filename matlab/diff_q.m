% Differentiate a symbolic function of time with respect to an internal
% variable, for use with the chain rule
% Inputs:
% * ft = a symbolic function f(q(t))
% * qt = the symbolic internal function q(t)
%
% Outputs:
% dft = a symbolic function of time (df/dq)(q(t))
function dft = diff_q(ft, qt)
    % set a constant symbolic variable q, which will substitute for q(t)
    q = sym('diff_q_internal_variable_q',size(qt));
    % Insert q into the function ft
    ftconst = subs(ft,qt,q);
    % Differentiate ft with respect to this constant function
    dftconst = diff(ftconst, q);
    % Now resubstitute time into the derivative
    dft = simplify(subs(dftconst,q,qt));
end