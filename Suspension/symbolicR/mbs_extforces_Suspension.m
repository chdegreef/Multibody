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
%	==> Generation Date : Thu May 19 17:32:49 2016
%
%	==> Project name : Suspension
%	==> using XML input file 
%
%	==> Number of joints : 6
%
%	==> Function : F19 : External Forces
%	==> Flops complexity : 120
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [frc,trq] = extforces(s,tsim,usrfun)

 frc = zeros(3,6);
 trq = zeros(3,6);

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

% = = Block_0_0_1_1_0_3 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
  S5p3p4 = C5*S3p4+S5*C3p4;
  C5p3p4 = C5*C3p4-S5*S3p4;
 
% Sensor Kinematics 


  ROcp1_26 = S6*S5p3p4;
  ROcp1_36 = -S6*C5p3p4;
  ROcp1_86 = -C6*S5p3p4;
  ROcp1_96 = C6*C5p3p4;
  RLcp1_24 = s.dpt(2,4)*C3;
  RLcp1_34 = s.dpt(2,4)*S3;
  OMcp1_14 = qd(3)+qd(4);
  ORcp1_24 = -qd(3)*RLcp1_34;
  ORcp1_34 = qd(3)*RLcp1_24;
  OPcp1_14 = qdd(3)+qdd(4);
  RLcp1_25 = -s.dpt(3,7)*S3p4;
  RLcp1_35 = s.dpt(3,7)*C3p4;
  OMcp1_15 = qd(5)+OMcp1_14;
  ORcp1_25 = -OMcp1_14*RLcp1_35;
  ORcp1_35 = OMcp1_14*RLcp1_25;
  OPcp1_15 = qdd(5)+OPcp1_14;
  RLcp1_26 = s.dpt(2,8)*C5p3p4;
  RLcp1_36 = s.dpt(2,8)*S5p3p4;
  ORcp1_26 = -OMcp1_15*RLcp1_36;
  ORcp1_36 = OMcp1_15*RLcp1_26;
  PxF1(1) = 0;
  PxF1(2) = RLcp1_24+RLcp1_25+RLcp1_26+s.dpt(2,2);
  PxF1(3) = q(1)+RLcp1_34+RLcp1_35+RLcp1_36+s.dpt(3,2);
  RxF1(1,1) = C6;
  RxF1(1,2) = ROcp1_26;
  RxF1(1,3) = ROcp1_36;
  RxF1(2,1) = 0;
  RxF1(2,2) = C5p3p4;
  RxF1(2,3) = S5p3p4;
  RxF1(3,1) = S6;
  RxF1(3,2) = ROcp1_86;
  RxF1(3,3) = ROcp1_96;
  VxF1(1) = 0;
  VxF1(2) = ORcp1_24+ORcp1_25+ORcp1_26;
  VxF1(3) = qd(1)+ORcp1_34+ORcp1_35+ORcp1_36;
  OMxF1(1) = OMcp1_15;
  OMxF1(2) = qd(6)*C5p3p4;
  OMxF1(3) = qd(6)*S5p3p4;
  AxF1(1) = 0;
  AxF1(2) = -(qd(3)*ORcp1_34+qdd(3)*RLcp1_34+OMcp1_14*ORcp1_35+OMcp1_15*ORcp1_36+OPcp1_14*RLcp1_35+OPcp1_15*RLcp1_36);
  AxF1(3) = qdd(1)+qd(3)*ORcp1_24+qdd(3)*RLcp1_24+OMcp1_14*ORcp1_25+OMcp1_15*ORcp1_26+OPcp1_14*RLcp1_25+OPcp1_15*RLcp1_26;
  OMPxF1(1) = OPcp1_15;
  OMPxF1(2) = -(qd(6)*OMcp1_15*S5p3p4-qdd(6)*C5p3p4);
  OMPxF1(3) = qd(6)*OMcp1_15*C5p3p4+qdd(6)*S5p3p4;
 
% Sensor Forces Computation 

  SWr1 = usrfun.fext(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
% Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc12 = ROcp1_26*SWr1(2)+ROcp1_36*SWr1(3)+SWr1(1)*C6;
  xfrc22 = SWr1(2)*C5p3p4+SWr1(3)*S5p3p4;
  xfrc32 = ROcp1_86*SWr1(2)+ROcp1_96*SWr1(3)+SWr1(1)*S6;
  frc(1,6) = s.frc(1,6)+xfrc12;
  frc(2,6) = s.frc(2,6)+xfrc22;
  frc(3,6) = s.frc(3,6)+xfrc32;
  xtrq12 = ROcp1_26*SWr1(5)+ROcp1_36*SWr1(6)+SWr1(4)*C6;
  xtrq22 = SWr1(5)*C5p3p4+SWr1(6)*S5p3p4;
  xtrq32 = ROcp1_86*SWr1(5)+ROcp1_96*SWr1(6)+SWr1(4)*S6;
  trq(1,6) = s.trq(1,6)+xtrq12-xfrc22*SWr1(9)+xfrc32*SWr1(8);
  trq(2,6) = s.trq(2,6)+xtrq22+xfrc12*SWr1(9)-xfrc32*SWr1(7);
  trq(3,6) = s.trq(3,6)+xtrq32-xfrc12*SWr1(8)+xfrc22*SWr1(7);

% = = Block_0_0_1_1_1_0 = = 
 
% Symbolic Outputs  

  frc(1,1) = s.frc(1,1);
  frc(2,1) = s.frc(2,1);
  frc(3,1) = s.frc(3,1);
  frc(1,2) = s.frc(1,2);
  frc(2,2) = s.frc(2,2);
  frc(3,2) = s.frc(3,2);
  frc(1,3) = s.frc(1,3);
  frc(2,3) = s.frc(2,3);
  frc(3,3) = s.frc(3,3);
  frc(1,4) = s.frc(1,4);
  frc(2,4) = s.frc(2,4);
  frc(3,4) = s.frc(3,4);
  frc(1,5) = s.frc(1,5);
  frc(2,5) = s.frc(2,5);
  frc(3,5) = s.frc(3,5);
  trq(1,1) = s.trq(1,1);
  trq(2,1) = s.trq(2,1);
  trq(3,1) = s.trq(3,1);
  trq(1,2) = s.trq(1,2);
  trq(2,2) = s.trq(2,2);
  trq(3,2) = s.trq(3,2);
  trq(1,3) = s.trq(1,3);
  trq(2,3) = s.trq(2,3);
  trq(3,3) = s.trq(3,3);
  trq(1,4) = s.trq(1,4);
  trq(2,4) = s.trq(2,4);
  trq(3,4) = s.trq(3,4);
  trq(1,5) = s.trq(1,5);
  trq(2,5) = s.trq(2,5);
  trq(3,5) = s.trq(3,5);

% ====== END Task 0 ====== 

  

