close all;
clear;
clc;

%% Definition des corps
% 0: Base
% 1: Chassis
% 2: UpperArm
% 3: LowerArm
% 4: Knuckle
% 5: Wheel
Chassis = struct('m',50,'I',zeros(3,3));
UpperArm = struct('m',1,'I',[0.01 0 0; 0 0 0; 0 0 0]);
LowerArm = struct('m',1,'I',[0.01 0 0; 0 0 0; 0 0 0]);
Knuckle = struct('m',5,'I',zeros(3,3));
Wheel = struct('m',5,'I',[0 0 0; 0 0.4 0; 0 0 0]);
AllData.Body = [Chassis UpperArm LowerArm Knuckle Wheel]; % Mettre la base ? body 0
AllData.inBody = [0 1 1 3 4];
AllData.NBody = length(AllData.Body);


%% Definition des ddl:
% 1: base->chassis, T3
% 2: chassis->upperArm, R1
% 3: chassis->LowerArm, R1
% 4: lowerArm->knuckle, R1
% 5: knuckle->axle, R1
% 6: axle->wheel, R2
AllData.q = [0 0 0 0 0 0];
AllData.qd = [0 0 0 0 0 0];
AllData.qdd = [0 0 0 0 0 0];
AllData.phi = [0 1 1 1 1 1];
AllData.psy = [1 0 0 0 0 0];

%% Calcul Forward Kinematic
AllData.alphac = zeros(1,AllData.NBody);
AllData.alphac(1) = -g;
AllData.omega = zeros(1,AllData.NBody);
AllData.omegacd = zeros(1,AllData.NBody);
AllData.OM = zeros(AllData.NBody, AllData.NBody);
AllData.AM = zeros(AllData.NBody, AllData.NBody);
for i=1:AllData.Nbody
    h = AllData.inBody(i);
    AllData.omega(i) = AllData.omega(h) + AllData.qd(i)*AllData.phi(i);
    AllData.omegacd(i) = AllData.omegacd(i) + blablabla à continuer
end