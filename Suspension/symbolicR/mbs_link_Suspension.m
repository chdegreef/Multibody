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
%	==> Function : F 7 : Point to point Link Forces (frc,trq,Flnk) 
%	==> Flops complexity : 29
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [frc,trq,Flnk,Z,Zd] = link(s,tsim,usrfun)

 frc = zeros(3,6);
 trq = zeros(3,6);
 Flnk = zeros(1,1);
 Z = zeros(1,1);
 Zd = zeros(1,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

% = = Block_0_0_0_0_0_3 = = 
 
% Trigonometric Variables  

  C3 = cos(q(3));
  S3 = sin(q(3));

% = = Block_0_1_0_0_1_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  RLlnk1_210 = s.dpt(2,5)*C3;
  RLlnk1_310 = s.dpt(2,5)*S3;
  POlnk1_310 = RLlnk1_310+s.dpt(3,2);

% = = Block_0_1_0_1_1_3 = = 
 
% Link Kinematics: Distance Z , Relative Velocity ZD 

  Plnk21 = RLlnk1_210-s.dpt(2,1)+s.dpt(2,2);
  Z1 = sqrt(POlnk1_310*POlnk1_310+Plnk21*Plnk21);
  e21 = Plnk21/Z1;
  e31 = POlnk1_310/Z1;
  Zd1 = qd(3)*(RLlnk1_210*e31-RLlnk1_310*e21);
 
% Link Force Computation 

  Flink1 = usrfun.flink(Z1,Zd1,s,tsim,1);

% = = Block_0_1_0_2_2_1 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fPlnk21 = Flink1*e21;
  fPlnk31 = Flink1*e31;
  frc(2,1) = s.frc(2,1)+fPlnk21;
  frc(3,1) = s.frc(3,1)+fPlnk31;
  trq(1,1) = s.trq(1,1)+fPlnk31*s.dpt(2,1);

% = = Block_0_1_0_2_2_3 = = 
 
% Link Dynamics : Forces projection on body-fixed frames 

  fSlnk21 = Flink1*(e21*C3+e31*S3);
  fSlnk31 = -Flink1*(e21*S3-e31*C3);
  frc(2,3) = s.frc(2,3)-fSlnk21;
  frc(3,3) = s.frc(3,3)-fSlnk31;
  trq(1,3) = s.trq(1,3)-fSlnk31*(s.dpt(2,5)-s.l(2,3));

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  frc(1,1) = s.frc(1,1);
  frc(1,2) = s.frc(1,2);
  frc(2,2) = s.frc(2,2);
  frc(3,2) = s.frc(3,2);
  frc(1,3) = s.frc(1,3);
  frc(1,4) = s.frc(1,4);
  frc(2,4) = s.frc(2,4);
  frc(3,4) = s.frc(3,4);
  frc(1,6) = s.frc(1,6);
  frc(2,6) = s.frc(2,6);
  frc(3,6) = s.frc(3,6);
  trq(2,1) = s.trq(2,1);
  trq(3,1) = s.trq(3,1);
  trq(1,2) = s.trq(1,2);
  trq(2,2) = s.trq(2,2);
  trq(3,2) = s.trq(3,2);
  trq(2,3) = s.trq(2,3);
  trq(3,3) = s.trq(3,3);
  trq(1,4) = s.trq(1,4);
  trq(2,4) = s.trq(2,4);
  trq(3,4) = s.trq(3,4);
  trq(1,6) = s.trq(1,6);
  trq(2,6) = s.trq(2,6);
  trq(3,6) = s.trq(3,6);
  Flnk(1) = Flink1;
  Z(1) = Z1;
  Zd(1) = Zd1;

% ====== END Task 0 ====== 

  

