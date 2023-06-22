%% Hamiltonian Symbolic to Functional Format
% Convert Hamiltonian dynamics from symbolic form 
% [diff(q,t);diff(p,t)] = [f(q,p);g(q,p)] to the function handle
% @(x,y) [f(x,y);g(x,y)].
%
% Inputs:
% * dynamics = symbolic equation with the symbols {p,q} in the form
% [f(q,p);g(q,p)]. The symbols can be anything, but when symvar is called
% it must return the variable associated with p, then the variable
% associated with q.
% 
% Outputs:
% * dynf = function handle @(x,y) [f(x,y);g(x,y)] for the dynamics

function [dynf] = sym2fun(dynamics)
    % Get the symbolic variables for p and q, which are given in order
    % since symvar returns everything alphabetically.
    symbols = symvar(dynamics);
    psym = symbols(1); qsym = symbols(2);
    % Convert the symbolic equation into a function handle of the form
    % @(q,p)
    dynf = matlabFunction(dynamics,'vars',[qsym,psym]);
end

