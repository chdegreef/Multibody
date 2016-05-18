close all;
clear;
clc;

%% Definition des corps
% 1: Base
% 2: Chassis
% 3: UpperArm
% 4: LowerArm
% 5: Knuckle
% 6: Wheel
Chassis = struct('m',50,'I',zeros(3,3));
UpperArm = struct('m',1,'I',[0.01 0 0; 0 0 0; 0 0 0]);
LowerArm = struct('m',1,'I',[0.01 0 0; 0 0 0; 0 0 0]);
Knuckle = struct('m',5,'I',zeros(3,3));
Wheel = struct('m',5,'I',[0 0 0; 0 0.4 0; 0 0 0]);
Data.Body = [Chassis UpperArm LowerArm Knuckle Wheel];
Data.inBody = [0 1 2 2 4 5];
Data.NBody = length(Data.Body)+1;


%% Definition des ddl:
% 1: base->chassis, T3
% 2: chassis->upperArm, R1
% 3: chassis->LowerArm, R1
% 4: lowerArm->knuckle, R1
% 5: knuckle->axle, R1
% 6: axle->wheel, R2
Data.q = [0.7 -0.2 -0.3 0 -0.1 0.15];
Data.qd = zeros(1,Data.NBody);
Data.qdd = zeros(1,Data.NBody);
Data.phi = [0 1 1 1 1 0;
            0 0 0 0 0 1;    
            0 0 0 0 0 0];
Data.psi = [0 0 0 0 0 0;
            0 0 0 0 0 0;
            1 0 0 0 0 0];
Data.axis = [3 1 1 1 1 2]; 

Data.z = zeros(3,Data.NBody);
Data.z(:,1) = [0; 0; Data.q(1)];
Data.dij = zeros(3,Data.NBody,Data.NBody);  % Definition of the vectors between anchor points and centers of mass
Data.dij(:,1,2) = [0; 0.25; 0];
Data.dij(:,1,3) = [0; 0.25; -0.25];
Data.dij(:,2,2) = [0; 0.25; 0];
Data.dij(:,3,3) = [0; 0.25; 0];
Data.dij(:,3,4) = [0; 0.5; 0];
Data.dij(:,4,4) = [0; 0; 0.125];
Data.dij(:,4,5) = [0; 0.1; 0.125];

Data.joints = zeros(6,Data.NBody);  % For each body, contains all the joints that connect them to their parent, until a maximum of 6
Data.joints(1,2) = 1;
Data.joints(1,3) = 2;
Data.joints(1,4) = 3;
Data.joints(1,5) = 4;
Data.joints(1:2,6) = [5; 6];

%% Initialization
Data.alphac = zeros(3,Data.NBody);
Data.alphac(:,1) = [0; 0; -9.81];     % Gravity in the inertial frame
Data.betac = zeros(3,3,Data.NBody);
Data.omega = zeros(3,Data.NBody);
Data.omegacd = zeros(3,Data.NBody);
Data.OM = zeros(3, Data.NBody, Data.NBody);
Data.AM = zeros(3, Data.NBody, Data.NBody);

%% Forward Kinematics

for i=2:Data.NBody
    h = Data.inBody(i);
    
    R_ih = eye(3);
    Data.alphac(:,i) = zeros(3,1);
    joint = 1;
    while(Data.joints(joint,i) ~= 0)    % Allows to treat the case where two bodies are connected by more than one joint (ex: multiple rotations)
        if(Data.phi(Data.axis(Data.joints(joint,i)),Data.joints(joint,i)) == 1)
            R_ih_loc = rot(Data.axis(Data.joints(joint,i)),Data.q(Data.joints(joint,i)));
            R_ih = R_ih_loc*R_ih;
        else
            R_ih_loc = eye(3);
        end
        
        if (joint == 1)
            Data.omega(:,i) = R_ih_loc*Data.omega(:,h) + Data.phi(:,Data.joints(joint,i))*Data.qd(Data.joints(joint,i));
            Data.omegacd(:,i) = R_ih_loc*Data.omegacd(:,h) + tilde(Data.omega(:,i))*Data.phi(:,Data.joints(joint,i))*Data.qd(Data.joints(joint,i));
        else
            Data.omega(:,i) = R_ih_loc*Data.omega(:,i) + Data.phi(:,Data.joints(joint,i))*Data.qd(Data.joints(joint,i));
            Data.omegacd(:,i) = R_ih_loc*Data.omegacd(:,i) + tilde(Data.omega(:,i))*Data.phi(:,Data.joints(joint,i))*Data.qd(Data.joints(joint,i));
        end
        Data.alphac(:,i) = Data.alphac(:,i) + 2*tilde(Data.omega(:,i))*Data.psi(:,Data.joints(joint,i))*Data.qd(Data.joints(joint,i));
        joint = joint + 1;
    end
    
    % Definition of the rotation matrix between frame h and i
    %Data.omega(:,i) = R_ih*Data.omega(:,h) + Data.phi(:,i)*Data.qd(i);
    %Data.omegacd(:,i) = R_ih*Data.omegacd(:,h) + tilde(Data.omega(:,i))*Data.phi(:,i)*Data.qd(i);
    Data.betac(:,:,i) = tilde(Data.omegacd(:,i)) + tilde(Data.omega(:,i))*tilde(Data.omega(:,i));
    %Data.alphac(:,i) = R_ih*(Data.alphac(:,h) + Data.betac(:,:,h)*(Data.z(h) + Data.dij(:,h,i))) + 2*tilde(Data.omega(:,i))*Data.psi(:,i)*Data.qd(i);
    Data.alphac(:,i) = Data.alphac(:,i) + R_ih*(Data.alphac(:,h) + Data.betac(:,:,h)*(Data.z(h) + Data.dij(:,h,i)));
    
    for k=2:i
        Data.OM(:,i,k) = R_ih*Data.OM(:,h,k);% + (k==i)*Data.phi(:,i);
        Data.AM(:,i,k) = R_ih*(Data.AM(:,h,k) + tilde(Data.OM(:,h,k))*(Data.z(:,h) + Data.dij(:,h,i)));% + (k==i)*Data.psi(:,i);
        joint = 1;
        while(Data.joints(joint,i) ~= 0)
            Data.OM(:,i,k) = Data.OM(:,i,k) + (k==Data.joints(joint,i))*Data.phi(:,Data.joints(joint,i));
            Data.AM(:,i,k) = Data.AM(:,i,k) + (k==Data.joints(joint,i))*Data.psi(:,Data.joints(joint,i));
            joint = joint + 1;
        end
    end
    
end