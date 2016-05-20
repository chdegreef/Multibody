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
Axle = struct('m', 0,'I',zeros(3,3));
Wheel = struct('m',5,'I',[0 0 0; 0 0.4 0; 0 0 0]);
Data.Body = [Chassis UpperArm LowerArm Knuckle Axle Wheel];
Data.inBody = [1 2 2 4 5 6];
Data.NBody = length(Data.Body)+1;       % The +1 comes from the addition of the base


%% Definition des ddl:
% 1: base->chassis, T3
% 2: chassis->upperArm, R1
% 3: chassis->LowerArm, R1
% 4: lowerArm->knuckle, R1
% 5: knuckle->axle, R1
% 6: axle->wheel, R2

% Position, Speed and Acceleration
Data.q = [0.7 -0.2 0 0 0 0.1];
Data.qd = zeros(1,6);
Data.qdd = zeros(1,6);

% Characteristics of the joints
Data.phi = [0 1 1 1 1 0;
            0 0 0 0 0 1;    
            0 0 0 0 0 0];
Data.psi = [0 0 0 0 0 0;
            0 0 0 0 0 0;
            1 0 0 0 0 0];
Data.axis = [3 1 1 1 1 2];  % Axis along which the translation or rotation occurs

% Definition of the vectors between anchor points and centers of mass
Data.z = zeros(3,Data.NBody);
Data.z(:,2) = [0; 0; Data.q(1)];
Data.dij = zeros(3,Data.NBody,Data.NBody); 
Data.dij(:,2,3) = [0; 0.25; 0];
Data.dij(:,2,4) = [0; 0.25; -0.25];
Data.dij(:,3,3) = [0; 0.25; 0];
Data.dij(:,4,4) = [0; 0.25; 0];
Data.dij(:,4,5) = [0; 0.5; 0];
Data.dij(:,5,5) = [0; 0; 0.125];
Data.dij(:,5,6) = [0; 0.1; 0.125];

%% Initialization
Data.alphac = zeros(3,Data.NBody);
Data.alphac(:,1) = [0; 0; -9.81];     % Gravity in the inertial frame
Data.betac = zeros(3,3,Data.NBody);

Data.omega = zeros(3,Data.NBody);
Data.omegacd = zeros(3,Data.NBody);

Data.OM = zeros(3, Data.NBody, Data.NBody);
Data.AM = zeros(3, Data.NBody, Data.NBody);
Data.Rij = zeros(3,3,Data.NBody);

Data.F = zeros(3, Data.NBody);
Data.L = zeros(3, Data.NBody);
Data.Wc = zeros(3, Data.NBody);
Data.Fc = zeros(3, Data.NBody);
Data.Lc = zeros(3, Data.NBody);
Data.WM = zeros(3, Data.NBody, Data.NBody);
Data.FM = zeros(3, Data.NBody, Data.NBody);
Data.LM = zeros(3, Data.NBody, Data.NBody);

%% Forward Kinematics
Data.z(:,2) = [0; 0; Data.q(1)];            % Update of the z-vector of the chassis

for i=2:Data.NBody
    h = Data.inBody(i-1);                   % Index of the parent body
    
    R_ih = eye(3);
    if(Data.phi(Data.axis(i-1),i-1) == 1)   % If the degree of freedom is a rotation, we calculate the rotation matrix R_ih
        R_ih = rot(Data.axis(i-1),Data.q(i-1));
    end
    
    Data.Rij(:,:,i) = R_ih;                 % Save the rotation matrix between the ith body and its parent h ==> X_i = R_ih* X_h
    Data.omega(:,i) = R_ih*Data.omega(:,h) + Data.phi(:,i-1)*Data.qd(i-1);
    Data.omegacd(:,i) = R_ih*Data.omegacd(:,h) + tilde(Data.omega(:,i))*Data.phi(:,i-1)*Data.qd(i-1); 
    Data.betac(:,:,i) = tilde(Data.omegacd(:,i)) + tilde(Data.omega(:,i))*tilde(Data.omega(:,i));
    Data.alphac(:,i) = R_ih*(Data.alphac(:,h) + Data.betac(:,:,h)*(Data.z(:,h) + Data.dij(:,h,i))) + 2*tilde(Data.omega(:,i))*Data.psi(:,i-1)*Data.qd(i-1);
    
    for k=2:i
        Data.OM(:,i,k) = R_ih*Data.OM(:,h,k) + (k==i)*Data.phi(:,i-1);
        Data.AM(:,i,k) = R_ih*(Data.AM(:,h,k) + tilde(Data.OM(:,h,k))*(Data.z(:,h) + Data.dij(:,h,i))) + (k==i)*Data.psi(:,i-1);
    end
    
end

%% External forces
Data.Fext = extForces(Data);
Data.Lext = zeros(3, Data.NBody);

%% Backward Dynamics
i = Data.NBody;
while(i > 1)
    Data.Wc(:,i) = Data.Body(i-1).m*(Data.alphac(:,i) + Data.betac(:,:,i)*(Data.z(:,i) + Data.dij(:,i,i))) - Data.Fext(:,i);
    Data.Fc(:,i) = Data.Wc(:,i);
    Data.Lc(:,i) = tilde(Data.z(:,i) + Data.dij(:,i,i))*Data.Wc(:,i) - Data.Lext(:,i) + Data.Body(i-1).I*Data.omegacd(:,i) + tilde(Data.omega(:,i))*Data.Body(i-1).I*Data.omega(:,i);
    for j=2:Data.NBody
        if(Data.inBody(j-1) == i)     % The parent body is the current body <==> The jth body is the children of the ith body
            Data.Fc(:,i) = Data.Fc(:,i) + Data.Rij(:,:,j).' * Data.Fc(:,j);
            Data.Lc(:,i) = Data.Lc(:,i) + Data.Rij(:,:,j).' * Data.Lc(:,j) + tilde(Data.z(:,i) + Data.dij(:,i,j))*Data.Rij(:,:,j).' * Data.Fc(:,j);
        end
    end
    
    for k=2:i
        Data.WM(:,i,k) = Data.Body(i-1).m*(Data.AM(:,i,k) + tilde(Data.OM(:,i,k))*(Data.z(:,i) + Data.dij(:,i,i)));
        Data.FM(:,i,k) = Data.WM(:,i,k);
        Data.LM(:,i,k) = tilde(Data.z(:,i) + Data.dij(:,i,i))*Data.WM(:,i,k) + Data.Body(i-1).I*Data.OM(:,i,k);
        for j=2:Data.NBody
            if(Data.inBody(j-1) == i)     % The parent body is the current body <==> The jth body is the children of the ith body
                Data.FM(:,i,k) = Data.FM(:,i,k) + Data.Rij(:,:,j).' * Data.FM(:,j,k);
                Data.LM(:,i,k) = Data.LM(:,i,k) + Data.Rij(:,:,j).' * Data.LM(:,j,k) + tilde(Data.z(:,i) + Data.dij(:,i,j))*Data.Rij(:,:,j).' * Data.FM(:,j,k);
            end
        end
    end
    i = i-1;
end

%% Mass matrix M, non-linear term C, independent term Q
for i=2:Data.NBody
    Data.C(i-1) = Data.psi(:,i-1)'*Data.Fc(:,i) + Data.phi(:,i-1)'*Data.Lc(:,i);
    for j=2:i
        Data.M(i-1,j-1) = Data.psi(:,i-1)'*Data.FM(:,i,j) + Data.phi(:,i-1)'*Data.LM(:,i,j);
    end
end
Data.M(:,:) = (Data.M(:,:) + Data.M(:,:).')/2;