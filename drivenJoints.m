function [q,qd,qdd] = drivenJoints(data)
% Gives everything about driven joints
%%%
% Authors
%   De Gr�ef Christophe,
%   Greiner Philippe,
%   Lefebvre Martin, 
%   Raucq Simon
qd  = data.qd;
qdd = data.qdd;
q(5) = 0.1;
return