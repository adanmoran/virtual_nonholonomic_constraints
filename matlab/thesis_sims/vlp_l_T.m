%% VLP Continuous Controller $l_T(\theta)$
% This function defines the continuous version of the time-optimal
% controller for the VLP. It attaches sinusoids of the correct frequency to
% the original time-optimal controller from
% from the paper "Pumping a Swing by Standing and Squatting".
%
% Inputs:
% * theta = atan2(p,q)
% * lu = lower actuator limit for l(theta). Domain is ]0,lb[
% * lb = upper actuator limit for l(theta). Domain is ]lu,inf[
% * T = amount by which we truncate each part of the time-optimal
% controller. Domain is [0,pi/2].
%
% Output:
% * l_T(theta). If T = 0, we return vlp_l_optimal. If T = pi/2, we return
% vlp_l_smooth.
function l = vlp_l_T(theta,lu,lb,T)
% Return vlp_l_optimal if T = 0
if T == 0
    l = vlp_l_optimal_gain(theta,lu,lb);
    return;
elseif T == pi/2
     l = vlp_l_smooth(theta,lu,lb);
    return;
end
% Set the frequency of the sinusoid
w = pi/T;
% Compute Deltal and lavg
dl = (lb - lu)/2;
lavg = (lb + lu)/2;
% If theta is within [-pi,-pi+T/2[ we use a neg. sinusoid shifted by -pi
if theta < -pi + T/2
    l = -dl*sin(w*(theta+pi)) + lavg;
% If it's within ]pi-T/2,pi] we have a neg. sin shifted by pi
elseif theta > pi-T/2
    l = -dl*sin(w*(theta-pi)) + lavg;
% If it's within T/2 of 0, it's a neg. sin with no shift
elseif -T/2 < theta && theta < T/2
    l = -dl*sin(w*theta) + lavg;
% If it's around +- pi/2, it's a positive sin with a shift of +- pi/2
elseif -pi/2 - T/2 < theta && theta < -pi/2 + T/2
    l = dl*sin(w*(theta+pi/2)) + lavg;
elseif pi/2 - T/2 < theta && theta < pi/2 + T/2
    l = dl*sin(w*(theta-pi/2)) + lavg;
% If it's none of these cases, then we are within the domain of the optimal
% controller, so we return the optimal result.
else
    l = vlp_l_optimal_gain(theta,lu,lb);
end
end

