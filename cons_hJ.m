function [h,J] = cons_hJ(data)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
l_arm = 0.5;
h_arm = 0.25;
off_arm = -0.25;
q = data.q;

%% Constraints
h = zeros(2,1);
h(1) = l_arm*cos(q(3)) - h_arm*sin(q(3)+q(4)) - l_arm*cos(q(2));
h(2) = l_arm*sin(q(3)) + h_arm*cos(q(3)+q(4)) - l_arm*sin(q(2)) + off_arm;

%% Jacobian
J = zeros(2,6);
J(1,2) = l_arm*sin(q(2));
J(1,3) = -(l_arm*sin(q(3)) + h_arm*cos(q(3)+q(4)));
J(1,4) = -h_arm*cos(q(3)+q(4));
J(2,2) = -l_arm*cos(q(2));
J(2,3) = l_arm*cos(q(3)) - h_arm*sin(q(3)+q(4));
J(2,4) = -h_arm*sin(q(3)+q(4));
end