% Solve for a time-varying variable in a symbolic expression (equality,
% inequality, etc).
% Inputs: 
% * expression = the symbolic expression of var
% * vt = the symbolic variable of time for which to solve
%
% Outputs:
% * soln = the solved expression, or an empty list if unsolvable

function [soln] = solve_for(expression, vt)
    % Define an alternate variable which will substitute for the
    % time-dependent variable
    vConst = sym('solve_for_internal_sym_vt');
    % Substitute for the symbolic variable in the expression
    expConst = subs(expression,vt,vConst);
    % Solve for vConst
    solution = solve(expConst,vConst);
    % If this is not solvable, stop now
    if isempty(solution)
        soln = solution;
        return;
    end
    % Replace vConst in the solved expression with vt
    soln = subs(solution, vConst, vt);
end