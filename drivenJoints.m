function [q,qd,qdd] = drivenJoints(data)
qd  = data.qd;
qdd = data.qdd;
q(5) = 0.1;
return