%
%-------------------------------------------------------------
%
%	ROBOTRAN - Version 6.6 (build : february 22, 2008)
%
%	Copyright 
%	Universite catholique de Louvain 
%	Departement de Mecanique 
%	Unite de Production Mecanique et Machines 
%	2, Place du Levant 
%	1348 Louvain-la-Neuve 
%	http://www.robotran.be// 
%
%	==> Generation Date : Thu Apr 28 22:41:19 2016
%
%	==> Project name : Suspension
%	==> using XML input file 
%
%	==> Number of joints : 6
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 143
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [sens] = gensensor(s,tsim,usrfun,isens)

 sens.P = zeros(3,1);
 sens.R = zeros(3,3);
 sens.V = zeros(3,1);
 sens.OM = zeros(3,1);
 sens.A = zeros(3,1);
 sens.OMP = zeros(3,1);
 sens.J = zeros(6,6);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 
 
% Sensor Kinematics 



% = = Block_0_0_0_0_0_2 = = 
 
% Trigonometric Variables  

  C2 = cos(q(2));
  S2 = sin(q(2));

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C3 = cos(q(3));
  S3 = sin(q(3));
  C4 = cos(q(4));
  S4 = sin(q(4));
  C5 = cos(q(5));
  S5 = sin(q(5));
  C6 = cos(q(6));
  S6 = sin(q(6));

% = = Block_0_0_0_4_0_3 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;

% = = Block_0_0_0_5_0_3 = = 
 
% Trigonometric Variables  

  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(3) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = (1.0);
    sens.R(3,3) = (1.0);
    sens.V(3) = qd(1);
    sens.A(3) = qdd(1);
 
% 
case 2, 


% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = s.dpt(2,1);
    sens.P(3) = q(1);
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C2;
    sens.R(2,3) = S2;
    sens.R(3,2) = -S2;
    sens.R(3,3) = C2;
    sens.V(3) = qd(1);
    sens.OM(1) = qd(2);
    sens.A(3) = qdd(1);
    sens.OMP(1) = qdd(2);
 
% 
case 3, 


% = = Block_1_0_0_3_0_3 = = 
 
% Sensor Kinematics 


    POcp2_33 = q(1)+s.dpt(3,2);

% = = Block_1_0_0_3_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = s.dpt(2,2);
    sens.P(3) = POcp2_33;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C3;
    sens.R(2,3) = S3;
    sens.R(3,2) = -S3;
    sens.R(3,3) = C3;
    sens.V(3) = qd(1);
    sens.OM(1) = qd(3);
    sens.A(3) = qdd(1);
    sens.OMP(1) = qdd(3);
 
% 
case 4, 


% = = Block_1_0_0_4_0_3 = = 
 
% Sensor Kinematics 


    RLcp3_24 = s.dpt(2,4)*C3;
    RLcp3_34 = s.dpt(2,4)*S3;
    POcp3_24 = RLcp3_24+s.dpt(2,2);
    POcp3_34 = RLcp3_34+q(1)+s.dpt(3,2);
    OMcp3_14 = qd(3)+qd(4);
    ORcp3_24 = -RLcp3_34*qd(3);
    ORcp3_34 = RLcp3_24*qd(3);
    VIcp3_34 = ORcp3_34+qd(1);
    OPcp3_14 = qdd(3)+qdd(4);
    ACcp3_24 = -(ORcp3_34*qd(3)+RLcp3_34*qdd(3));
    ACcp3_34 = qdd(1)+ORcp3_24*qd(3)+RLcp3_24*qdd(3);

% = = Block_1_0_0_4_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = POcp3_24;
    sens.P(3) = POcp3_34;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C3p4;
    sens.R(2,3) = S3p4;
    sens.R(3,2) = -S3p4;
    sens.R(3,3) = C3p4;
    sens.V(2) = ORcp3_24;
    sens.V(3) = VIcp3_34;
    sens.OM(1) = OMcp3_14;
    sens.A(2) = ACcp3_24;
    sens.A(3) = ACcp3_34;
    sens.OMP(1) = OPcp3_14;
 
% 
case 5, 


% = = Block_1_0_0_5_0_3 = = 
 
% Sensor Kinematics 


    RLcp4_24 = s.dpt(2,4)*C3;
    RLcp4_34 = s.dpt(2,4)*S3;
    OMcp4_14 = qd(3)+qd(4);
    ORcp4_24 = -RLcp4_34*qd(3);
    ORcp4_34 = RLcp4_24*qd(3);
    OPcp4_14 = qdd(3)+qdd(4);
    RLcp4_25 = s.dpt(2,7)*C3p4-s.dpt(3,7)*S3p4;
    RLcp4_35 = s.dpt(2,7)*S3p4+s.dpt(3,7)*C3p4;
    POcp4_25 = RLcp4_24+RLcp4_25+s.dpt(2,2);
    POcp4_35 = RLcp4_34+RLcp4_35+q(1)+s.dpt(3,2);
    OMcp4_15 = OMcp4_14+qd(5);
    ORcp4_25 = -OMcp4_14*RLcp4_35;
    ORcp4_35 = OMcp4_14*RLcp4_25;
    VIcp4_25 = ORcp4_24+ORcp4_25;
    VIcp4_35 = ORcp4_34+ORcp4_35+qd(1);
    OPcp4_15 = OPcp4_14+qdd(5);
    ACcp4_25 = -(OMcp4_14*ORcp4_35+OPcp4_14*RLcp4_35+ORcp4_34*qd(3)+RLcp4_34*qdd(3));
    ACcp4_35 = qdd(1)+OMcp4_14*ORcp4_25+OPcp4_14*RLcp4_25+ORcp4_24*qd(3)+RLcp4_24*qdd(3);

% = = Block_1_0_0_5_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = POcp4_25;
    sens.P(3) = POcp4_35;
    sens.R(1,1) = (1.0);
    sens.R(2,2) = C5p3p4;
    sens.R(2,3) = S5p3p4;
    sens.R(3,2) = -S5p3p4;
    sens.R(3,3) = C5p3p4;
    sens.V(2) = VIcp4_25;
    sens.V(3) = VIcp4_35;
    sens.OM(1) = OMcp4_15;
    sens.A(2) = ACcp4_25;
    sens.A(3) = ACcp4_35;
    sens.OMP(1) = OPcp4_15;
 
% 
case 6, 


% = = Block_1_0_0_6_0_3 = = 
 
% Sensor Kinematics 


    ROcp5_26 = S6*S5p3p4;
    ROcp5_36 = -S6*C5p3p4;
    ROcp5_86 = -C6*S5p3p4;
    ROcp5_96 = C6*C5p3p4;
    RLcp5_24 = s.dpt(2,4)*C3;
    RLcp5_34 = s.dpt(2,4)*S3;
    OMcp5_14 = qd(3)+qd(4);
    ORcp5_24 = -RLcp5_34*qd(3);
    ORcp5_34 = RLcp5_24*qd(3);
    OPcp5_14 = qdd(3)+qdd(4);
    RLcp5_25 = s.dpt(2,7)*C3p4-s.dpt(3,7)*S3p4;
    RLcp5_35 = s.dpt(2,7)*S3p4+s.dpt(3,7)*C3p4;
    POcp5_25 = RLcp5_24+RLcp5_25+s.dpt(2,2);
    POcp5_35 = RLcp5_34+RLcp5_35+q(1)+s.dpt(3,2);
    OMcp5_15 = OMcp5_14+qd(5);
    ORcp5_25 = -OMcp5_14*RLcp5_35;
    ORcp5_35 = OMcp5_14*RLcp5_25;
    VIcp5_25 = ORcp5_24+ORcp5_25;
    VIcp5_35 = ORcp5_34+ORcp5_35+qd(1);
    OPcp5_15 = OPcp5_14+qdd(5);
    ACcp5_25 = -(OMcp5_14*ORcp5_35+OPcp5_14*RLcp5_35+ORcp5_34*qd(3)+RLcp5_34*qdd(3));
    ACcp5_35 = qdd(1)+OMcp5_14*ORcp5_25+OPcp5_14*RLcp5_25+ORcp5_24*qd(3)+RLcp5_24*qdd(3);
    OMcp5_26 = qd(6)*C5p3p4;
    OMcp5_36 = qd(6)*S5p3p4;
    OPcp5_26 = -(OMcp5_15*qd(6)*S5p3p4-qdd(6)*C5p3p4);
    OPcp5_36 = OMcp5_15*qd(6)*C5p3p4+qdd(6)*S5p3p4;

% = = Block_1_0_0_6_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = POcp5_25;
    sens.P(3) = POcp5_35;
    sens.R(1,1) = C6;
    sens.R(1,2) = ROcp5_26;
    sens.R(1,3) = ROcp5_36;
    sens.R(2,2) = C5p3p4;
    sens.R(2,3) = S5p3p4;
    sens.R(3,1) = S6;
    sens.R(3,2) = ROcp5_86;
    sens.R(3,3) = ROcp5_96;
    sens.V(2) = VIcp5_25;
    sens.V(3) = VIcp5_35;
    sens.OM(1) = OMcp5_15;
    sens.OM(2) = OMcp5_26;
    sens.OM(3) = OMcp5_36;
    sens.A(2) = ACcp5_25;
    sens.A(3) = ACcp5_35;
    sens.OMP(1) = OPcp5_15;
    sens.OMP(2) = OPcp5_26;
    sens.OMP(3) = OPcp5_36;

end


% ====== END Task 1 ====== 

  

