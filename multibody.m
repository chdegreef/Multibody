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
AllData.dz

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
% Initialization
AllData.alphac = zeros(1,AllData.NBody);
AllData.alphac(1) = -g;
AllData.omega = zeros(1,AllData.NBody);
AllData.omegacd = zeros(1,AllData.NBody);
AllData.OM = zeros(AllData.NBody, AllData.NBody);
AllData.AM = zeros(AllData.NBody, AllData.NBody);
% Other vectors
AllData.betac = zeros(1,AllData.NBody);
for i=1:AllData.Nbody
    h = AllData.inBody(i);
    AllData.omega(i) = AllData.omega(h) + AllData.qd(i)*AllData.phi(i);
    AllData.omegacd(i) = AllData.omegacd(h) + getTilde(AllData.omega(i))*phi(i)*AllData.qd(i);
    AllData.betac(i) = getTilde(AllData.omegacd(i)) + getTilde(AllData.omega(i))*getTilde(AllData.omega(i));
    AllData.alphac(i) = AllData.alphac(h) + AllData.betac(h)*
    for k=1:i
        delta = 0;
        if k==i
            delta = 1;
        else
            delta = 0;
        end
        AllData.OM(i,k) = AllData.OM(h,k) + delta*phi(i)
    end
end