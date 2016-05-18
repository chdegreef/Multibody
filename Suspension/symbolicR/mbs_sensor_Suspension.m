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
%	==> Generation Date : Wed May 18 15:16:51 2016
%
%	==> Project name : Suspension
%	==> using XML input file 
%
%	==> Number of joints : 6
%
%	==> Function : F 6 : Sensors Kinematical Informations (sens) 
%	==> Flops complexity : 141
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [sens] = sensor(s,tsim,usrfun,isens)

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

% = = Block_0_0_0_1_0_3 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;

% ====== END Task 0 ====== 

% ===== BEGIN task 1 ===== 
 
switch isens

 
% 
case 1, 


% = = Block_1_0_0_1_0_3 = = 
 
% Sensor Kinematics 


    ROcp0_26 = S6*S5p3p4;
    ROcp0_36 = -S6*C5p3p4;
    ROcp0_86 = -C6*S5p3p4;
    ROcp0_96 = C6*C5p3p4;
    RLcp0_24 = s.dpt(2,4)*C3;
    RLcp0_34 = s.dpt(2,4)*S3;
    OMcp0_14 = qd(3)+qd(4);
    ORcp0_24 = -RLcp0_34*qd(3);
    ORcp0_34 = RLcp0_24*qd(3);
    OPcp0_14 = qdd(3)+qdd(4);
    RLcp0_25 = s.dpt(2,7)*C3p4-s.dpt(3,7)*S3p4;
    RLcp0_35 = s.dpt(2,7)*S3p4+s.dpt(3,7)*C3p4;
    POcp0_25 = RLcp0_24+RLcp0_25+s.dpt(2,2);
    POcp0_35 = RLcp0_34+RLcp0_35+q(1)+s.dpt(3,2);
    JTcp0_25_2 = -(RLcp0_34+RLcp0_35);
    JTcp0_35_2 = RLcp0_24+RLcp0_25;
    OMcp0_15 = OMcp0_14+qd(5);
    ORcp0_25 = -OMcp0_14*RLcp0_35;
    ORcp0_35 = OMcp0_14*RLcp0_25;
    VIcp0_25 = ORcp0_24+ORcp0_25;
    VIcp0_35 = ORcp0_34+ORcp0_35+qd(1);
    OPcp0_15 = OPcp0_14+qdd(5);
    ACcp0_25 = -(OMcp0_14*ORcp0_35+OPcp0_14*RLcp0_35+ORcp0_34*qd(3)+RLcp0_34*qdd(3));
    ACcp0_35 = qdd(1)+OMcp0_14*ORcp0_25+OPcp0_14*RLcp0_25+ORcp0_24*qd(3)+RLcp0_24*qdd(3);
    OMcp0_26 = qd(6)*C5p3p4;
    OMcp0_36 = qd(6)*S5p3p4;
    OPcp0_26 = -(OMcp0_15*qd(6)*S5p3p4-qdd(6)*C5p3p4);
    OPcp0_36 = OMcp0_15*qd(6)*C5p3p4+qdd(6)*S5p3p4;

% = = Block_1_0_0_1_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = POcp0_25;
    sens.P(3) = POcp0_35;
    sens.R(1,1) = C6;
    sens.R(1,2) = ROcp0_26;
    sens.R(1,3) = ROcp0_36;
    sens.R(2,2) = C5p3p4;
    sens.R(2,3) = S5p3p4;
    sens.R(3,1) = S6;
    sens.R(3,2) = ROcp0_86;
    sens.R(3,3) = ROcp0_96;
    sens.V(2) = VIcp0_25;
    sens.V(3) = VIcp0_35;
    sens.OM(1) = OMcp0_15;
    sens.OM(2) = OMcp0_26;
    sens.OM(3) = OMcp0_36;
    sens.J(2,3) = JTcp0_25_2;
    sens.J(2,4) = -RLcp0_35;
    sens.J(3,1) = (1.0);
    sens.J(3,3) = JTcp0_35_2;
    sens.J(3,4) = RLcp0_25;
    sens.J(4,3) = (1.0);
    sens.J(4,4) = (1.0);
    sens.J(4,5) = (1.0);
    sens.J(5,6) = C5p3p4;
    sens.J(6,6) = S5p3p4;
    sens.A(2) = ACcp0_25;
    sens.A(3) = ACcp0_35;
    sens.OMP(1) = OPcp0_15;
    sens.OMP(2) = OPcp0_26;
    sens.OMP(3) = OPcp0_36;
 
% 
case 2, 


% = = Block_1_0_0_2_0_3 = = 
 
% Sensor Kinematics 


    ROcp1_26 = S6*S5p3p4;
    ROcp1_36 = -S6*C5p3p4;
    ROcp1_86 = -C6*S5p3p4;
    ROcp1_96 = C6*C5p3p4;
    RLcp1_24 = s.dpt(2,4)*C3;
    RLcp1_34 = s.dpt(2,4)*S3;
    OMcp1_14 = qd(3)+qd(4);
    ORcp1_24 = -RLcp1_34*qd(3);
    ORcp1_34 = RLcp1_24*qd(3);
    OPcp1_14 = qdd(3)+qdd(4);
    RLcp1_25 = s.dpt(2,7)*C3p4-s.dpt(3,7)*S3p4;
    RLcp1_35 = s.dpt(2,7)*S3p4+s.dpt(3,7)*C3p4;
    POcp1_25 = RLcp1_24+RLcp1_25+s.dpt(2,2);
    POcp1_35 = RLcp1_34+RLcp1_35+q(1)+s.dpt(3,2);
    OMcp1_15 = OMcp1_14+qd(5);
    ORcp1_25 = -OMcp1_14*RLcp1_35;
    ORcp1_35 = OMcp1_14*RLcp1_25;
    VIcp1_25 = ORcp1_24+ORcp1_25;
    VIcp1_35 = ORcp1_34+ORcp1_35+qd(1);
    OPcp1_15 = OPcp1_14+qdd(5);
    ACcp1_25 = -(OMcp1_14*ORcp1_35+OPcp1_14*RLcp1_35+ORcp1_34*qd(3)+RLcp1_34*qdd(3));
    ACcp1_35 = qdd(1)+OMcp1_14*ORcp1_25+OPcp1_14*RLcp1_25+ORcp1_24*qd(3)+RLcp1_24*qdd(3);
    OMcp1_26 = qd(6)*C5p3p4;
    OMcp1_36 = qd(6)*S5p3p4;
    OPcp1_26 = -(OMcp1_15*qd(6)*S5p3p4-qdd(6)*C5p3p4);
    OPcp1_36 = OMcp1_15*qd(6)*C5p3p4+qdd(6)*S5p3p4;

% = = Block_1_0_0_2_1_0 = = 
 
% Symbolic Outputs  

    sens.P(2) = POcp1_25;
    sens.P(3) = POcp1_35;
    sens.R(1,1) = C6;
    sens.R(1,2) = ROcp1_26;
    sens.R(1,3) = ROcp1_36;
    sens.R(2,2) = C5p3p4;
    sens.R(2,3) = S5p3p4;
    sens.R(3,1) = S6;
    sens.R(3,2) = ROcp1_86;
    sens.R(3,3) = ROcp1_96;
    sens.V(2) = VIcp1_25;
    sens.V(3) = VIcp1_35;
    sens.OM(1) = OMcp1_15;
    sens.OM(2) = OMcp1_26;
    sens.OM(3) = OMcp1_36;
    sens.A(2) = ACcp1_25;
    sens.A(3) = ACcp1_35;
    sens.OMP(1) = OPcp1_15;
    sens.OMP(2) = OPcp1_26;
    sens.OMP(3) = OPcp1_36;

end


% ====== END Task 1 ====== 

  

