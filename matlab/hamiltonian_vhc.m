%% Hamiltonian for VHC
% Get the Hamiltonian at specific points in the (q,p) plane where a VHC is
% applied, for (q,p) in RxR.
% Suppose the dynamics are given by:
%
% q_dot = phi(q)*p
% p_dot = Pi(q)
%
% then the EOM are Hamiltonian with the Hamiltonian given by :
%
% H(q,p_tilde) = 1/2 D(q)^{-1}*p_tilde^2 + V(q)
%
% where:
%
% p_tilde(q) = p/phi(q)
% D(q)^{-1} = phi^2(q)
% V(q) = - integral_0^q{ Pi(tau)/phi(tau)dtau} 
%
% This function converts (q,p) to (q,p_tilde) and computes the Hamiltonian
% of the canonical coordinates.
%
% Inputs:
% * q = a vector of coordinate values.
% * p = conjugate of momenta for q, with values in R.
% * dynH = the dynamics of the VHC. This is either a) a symbolic equation,
% with the symbols {p,q} in the equation [phi(q)p;Pi(q)] (in that order,
% since we use symvar which is alphabetical) or b) a function handle, with
% the format @(q,p) [phi(q)p;Pi(q)].
%
% Outputs:
% * H = the hamiltonian H(q,p_tilde) = 1/2 D(q)^{-1}p_tilde^2 + V(q) for
% each value of (q,p), where H(q,p)(i) = H(q(i),p(i)). Returned as a column
% vector of size numel(q)x1
%
% This function assumes the whole phase space uses the VHC dynamics, so
% the user must handle limiting (q,p) according to where dynH is viable.

function H = hamiltonian_vhc(q,p,dynH)
    % Get the size of the output
    n = numel(q);
    % Check that q and p are the same size
    if numel(p) ~= n
        error('hamiltonian_vhc: q and p must be the same size.');
    end
    % Reshape q and p into column vectors
    q = reshape(q,n,1);
    p = reshape(p,n,1);
    
    % Determine if this is a function handle or a symbolic equation. If it
    % is a symbolic equation, turn it into a function handle.
    if isa(dynH,'function_handle')
        f = dynH;
    else
        f = sym2fun(dynH);
    end

    % Get phi(q) and Pi(q) as single element functions which return a value 
    % in R. These will be used in the integral to compute V(q).
    phi_1 = @(x) [1,0]*f(x,1);
    Pi_1 =  @(x) [0,1]*f(x,0);
    
    % Compute the kinetic energy 1/2*phi(q)^2*p_tilde^2 = 1/2 * p^2
    % because p_tilde = p ./ phi(q)
    KE = 1/2 .* p.^2;
    
    % Compute the kinetic energy V(q) = integral_0^q{ Pi(tau)/phi(tau)dtau}
    % Each integral individually is not an array, but we need to use the 
    % ArrayValued flag since Pi and phi are described by array 
    % multiplication.
    % The arrayfun wraps the integral to evaluate V(q) for each input q.
    V = -arrayfun(@(q2)integral(@(tau)Pi_1(tau)/phi_1(tau),...
                                   0, q2, 'ArrayValued', true),q);
    
    % Return the Hamiltonian as a column vector
    H = KE + V;
end

%% Testing
% To test this, run the following code (where dynH is given).
% % Get the vector field of dynH
% symbols = symvar(dynH);
% psym = symbols(1); qsym = symbols(2);
% f = matlabFunction(dynH,'vars',[qsym,psym]);
% phi = @(q1) [1,0]*f(q1,1);
% Pi = @(q1) [0,1]*f(q1,0);
% % The true hamiltonian for a single value of (q,p), as tested before
% % vectorizing the hamiltonian_vhc code.
% H = @(q1,p1) 1/2.*phi(q1).^2 .* (p1./phi(q1)).^2 - ...
% integral(@(tau)Pi(tau)./phi(tau),0,q1,'ArrayValued',true);
% % Generate a bunch of random samples of q and p in ]-pi,pi[x[-5,5]
% N = 1000;
% qs = rand(N,1)*(2*pi)-pi;
% ps = rand(N,1)*10-5;
% % Compute the ground truth H-value, as derived by the single input
% % equation
% Htrue = zeros(N,1);
% for i = 1:N
% Htrue(i) = H(qs(i),ps(i));
% end
% % Verify that the hamiltonian_vhc provides the same result as the ground
% % truth equation
% all(Htrue == hamiltonian_vhc(qs,ps,dynH))