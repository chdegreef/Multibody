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
%	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
%	==> Flops complexity : 157
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [M,c] = dirdyna(s,tsim,usrfun)

 M = zeros(6,6);
 c = zeros(6,1);

q = s.q; 
qd = s.qd; 
qdd = s.qdd; 
frc = s.frc; 
trq = s.trq; 

% === begin imp_aux === 

% === end imp_aux === 

% ===== BEGIN task 0 ===== 

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

% = = Block_0_1_0_1_0_3 = = 
 
% Trigonometric Variables  

  S3p4 = C3*S4+S3*C4;
 
% Forward Kinematics 

  BS53 = -qd(3)*qd(3);
  AlF23 = -s.g(3)*S3;
  AlF33 = -s.g(3)*C3;
  OM14 = qd(3)+qd(4);

% = = Block_0_2_0_1_0_2 = = 
 
% Backward Dynamics 

  FA32 = -(s.frc(3,2)+s.m(2)*s.g(3)*C2);
  CF12 = -(s.trq(1,2)-FA32*s.l(2,2));
  CM12_1 = s.m(2)*s.l(2,2)*C2;
  CM12_2 = s.In(1,2)+s.m(2)*s.l(2,2)*s.l(2,2);

% = = Block_0_2_0_1_0_3 = = 
 
% Backward Dynamics 

  FF25 = -(s.frc(2,5)+s.frc(2,6));
  FF35 = -(s.frc(3,5)-s.frc(1,6)*S6+s.frc(3,6)*C6);
  CF15 = -(s.trq(1,5)+s.trq(1,6)*C6+s.trq(3,6)*S6-s.dpt(2,8)*(s.frc(1,6)*S6-s.frc(3,6)*C6));
  FA24 = -(s.frc(2,4)-s.m(4)*(AlF33*S4+C4*(AlF23+BS53*s.dpt(2,4))));
  FF24 = FA24+FF25*C5-FF35*S5;
  FF34 = -(s.frc(3,4)-s.m(4)*(AlF33*C4-OM14*OM14*s.l(3,4)-S4*(AlF23+BS53*s.dpt(2,4)))-FF25*S5-FF35*C5);
  CF14 = -(s.trq(1,4)-CF15+FA24*s.l(3,4)+s.dpt(3,7)*(FF25*C5-FF35*S5));
  CM14_1 = -s.m(4)*s.l(3,4)*S3p4;
  FB24_3 = s.m(4)*(s.dpt(2,4)*S4-s.l(3,4));
  CM14_3 = -FB24_3*s.l(3,4);
  CM14_4 = s.m(4)*s.l(3,4)*s.l(3,4);
  FA33 = -(s.frc(3,3)-s.m(3)*AlF33);
  CF13 = -(s.trq(1,3)-CF14-FA33*s.l(2,3)-s.dpt(2,4)*(FF24*S4+FF34*C4));
  CM13_1 = s.m(3)*s.l(2,3)*C3+s.m(4)*(s.dpt(2,4)*C3-s.l(3,4)*S3p4);
  CM13_3 = s.In(1,3)+CM14_3+s.m(3)*s.l(2,3)*s.l(2,3)+s.dpt(2,4)*(s.m(4)*s.dpt(2,4)*C4*C4+FB24_3*S4);

% = = Block_0_2_0_2_0_1 = = 
 
% Backward Dynamics 

  FF31 = -(s.frc(3,1)+s.m(1)*s.g(3)-FA32*C2+S2*(s.frc(2,2)+s.m(2)*(qd(2)*qd(2)*s.l(2,2)+s.g(3)*S2))-C3*(FA33+FF24*S4+FF34*C4)+S3*(s.frc(2,3)-...
 s.m(3)*(AlF23+BS53*s.l(2,3))-FF24*C4+FF34*S4));
  FM31_1 = s.m(1)+s.m(2)+s.m(3)+s.m(4);

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  M(1,1) = FM31_1;
  M(1,2) = CM12_1;
  M(1,3) = CM13_1;
  M(1,4) = CM14_1;
  M(2,1) = CM12_1;
  M(2,2) = CM12_2;
  M(3,1) = CM13_1;
  M(3,3) = CM13_3;
  M(3,4) = CM14_3;
  M(4,1) = CM14_1;
  M(4,3) = CM14_3;
  M(4,4) = CM14_4;
  c(1) = FF31;
  c(2) = CF12;
  c(3) = CF13;
  c(4) = CF14;
  c(5) = CF15;
  c(6) = -s.trq(2,6);

% ====== END Task 0 ====== 

  

