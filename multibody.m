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
global Data;
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
Data.q0 = [0.7 -0.2 0 0 0 0]';
Data.qd0 = zeros(1,6)';
Data.qdd0 = zeros(1,6)';
Data.q = Data.q0;
Data.qd = Data.qd0;
Data.qdd = Data.qdd0;

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


%% Definition qu,qv,qc
Data.qu = [1 2 6];
Data.qv = [3 4];
Data.qc = [5];
Data.nqu = length(Data.qu);
Data.nqv = length(Data.qv);
Data.nqc = length(Data.qc);

%% Starting integration
tInit = 0;
tEnd = 10;
tSpan = [tInit,tEnd];
y0 = [Data.q0(Data.qu); Data.qd0(Data.qu)];
[tOut, yOut] = ode45(@integrationODE45,tSpan,y0);

%% Results
Data.q = Data.q0;
Data.qd = Data.qd0;
result.tOut = tOut;
for i=1:length(tOut)
    integrationODE45(tOut(i), yOut(i,:)'); % appel en boucle de fprime pour refaire tous les calculs a chaque pas de la solution
    result.q(i,:)  = Data.q;
    result.qd(i,:) = Data.qd;   
    %result.F = MBS_user.F; % a condition d'avoir stocker MBS_user.F ailleurs dans le programme
end

%% Animation
dtAnim = 0.001;
% vecteur des instants auquels on calcule une image
tAnim=[tOut(1):dtAnim:tOut(end)]';
% reechantillonage de la solution
qAnim = interp1(tOut, result.q, tAnim);
% sauvegarde du fichier d'animation
tqAnim=[tAnim qAnim];
save(['qAnimSimu.anim'], 'tqAnim',  '-ASCII');
