function [h,J] = cons_hJ(data)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
l = 0.5;
h = 0.25;
off_arm = -0.25;
q = data.q;

%% Constraints
h = zeros(2,1);
h(1) = l*cos(q(3)) - h*sin(q(3)+q(4)) - l*cos(q(2));
h(2) = l*sin(q(3)) + h*cos(q(3)+q(4)) - l*sin(q(2)) + off_arm;

%% Jacobian
J = zeros(2,6);
J(1,2) = l*sin(q(2));
J(1,3) = -(l*sin(q(3)) + h*cos(q(3)+q(4)));
J(1,4) = -h*cos(q(3)+q(4));
J(2,2) = -l*cos(q(2));
J(2,3) = l*cos(q(3)) - h*sin(q(3)+q(4));
J(2,4) = -h*sin(q(3)+q(4));
end

