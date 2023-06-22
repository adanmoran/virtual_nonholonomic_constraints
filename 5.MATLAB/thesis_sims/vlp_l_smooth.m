%% VLP Smooth Controller $l(\theta)$
% This function defines the smoothed version of the time-optimal
% controller for the VLP. It converts the results
% from the paper "Pumping a Swing by Standing and Squatting" into a
% sinusoid around the average length of the pendulum.
%
% Inputs:
% * theta = atan2(p,q)
% * lu = lower actuator limit for l(theta). Domain is ]0,lb[
% * lb = upper actuator limit for l(theta). Domain is ]lu,inf[
%
% Output:
% * l(theta) = -Delta * sin(2*theta) + lavg
function l = vlp_l_smooth(theta,lu,lb)
% Compute Delta_l and lavg
dl = (lb - lu)/2;
lavg = (lb + lu)/2;
% Return the smoothed controller
l = -dl*sin(2*theta) + lavg;
end