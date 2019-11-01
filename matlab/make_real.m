% Given a function of time and a list of variables which are supposed to be
% real, set real(q) = q, imag(q) = 0.
%
% Inputs:
% * ft = a function of qi(t)
% * q = a time-dependent list of variables qi(t)
%
% Outputs:
% * realF = a version of ft with real(q) = q and imag(q) = 0
function realF = make_real(ft, q)
    realF = ft;
    for qi = q
        realF = subs(realF,real(qi),qi);
        realF = subs(realF,imag(qi),0);
    end
end