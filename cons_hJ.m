function [h,J] = cons_hJ(data)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
l_arm = 0.5;
h_knuckle = 0.25;
off_arm = -0.25;

%% Constraints
h = zeros(2,1);
h(1) = l_arm*(cos(data.q(2)) - cos(data.q(3))) - cos(data.q(3)+data.q(4))*h_knuckle;
h(2) = l_arm*(sin(data.q(2)) - sin(data.q(3))) - sin(data.q(3)+data.q(4))*h_knuckle - off_arm;

%% Jacobian
J = zeros(2,6);
J(1,2) = -l_arm*sin(data.q(2));
J(1,3) = l_arm*sin(data.q(3)) + h_knuckle*sin(data.q(3) + data.q(4));
J(1,4) = h_knuckle*sin(data.q(3) + data.q(4));

J(2,2) = l_arm*cos(data.q(2));
J(2,3) = -l_arm*cos(data.q(3)) - h_knuckle*cos(data.q(3) + data.q(4));
J(2,4) = -h_knuckle*cos(data.q(3) + data.q(4));
end

