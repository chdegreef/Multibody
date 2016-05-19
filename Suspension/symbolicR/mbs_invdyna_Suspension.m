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
%	==> Function : F 2 : Inverse Dynamics : RNEA
%	==> Flops complexity : 132
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [Qq] = invdyna(s,tsim,usrfun)

 Qq = zeros(6,1);

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

% = = Block_0_1_0_0_0_0 = = 
 
% Forward Kinematics 

  ALPHA31 = qdd(1)-s.g(3);
  BS53 = -qd(3)*qd(3);
  ALPHA23 = ALPHA31*S3;
  ALPHA33 = ALPHA31*C3;
  OM14 = qd(3)+qd(4);
 
% Backward Dynamics 

  Fq25 = -(s.frc(2,5)+s.frc(2,6));
  Fq35 = -(s.frc(3,5)-s.frc(1,6)*S6+s.frc(3,6)*C6);
  Cq15 = -(s.trq(1,5)+s.trq(1,6)*C6+s.trq(3,6)*S6-s.dpt(2,8)*(s.frc(1,6)*S6-s.frc(3,6)*C6));
  Fs24 = -(s.frc(2,4)+s.m(4)*(s.l(3,4)*(qdd(3)+qdd(4))-C4*(ALPHA23+BS53*s.dpt(2,4))-S4*(ALPHA33+qdd(3)*s.dpt(2,4))));
  Fq24 = Fs24+Fq25*C5-Fq35*S5;
  Fq34 = -(s.frc(3,4)+s.m(4)*(OM14*OM14*s.l(3,4)-C4*(ALPHA33+qdd(3)*s.dpt(2,4))+S4*(ALPHA23+BS53*s.dpt(2,4)))-Fq25*S5-Fq35*C5);
  Cq14 = -(s.trq(1,4)-Cq15+Fs24*s.l(3,4)+s.dpt(3,7)*(Fq25*C5-Fq35*S5));
  Fs33 = -(s.frc(3,3)-s.m(3)*(ALPHA33+qdd(3)*s.l(2,3)));
  Cq13 = -(s.trq(1,3)-Cq14-qdd(3)*s.In(1,3)-Fs33*s.l(2,3)-s.dpt(2,4)*(Fq24*S4+Fq34*C4));
  Fs32 = -(s.frc(3,2)-s.m(2)*(qdd(2)*s.l(2,2)+ALPHA31*C2));
  Cq12 = -(s.trq(1,2)-qdd(2)*s.In(1,2)-Fs32*s.l(2,2));
  Fq31 = -(s.frc(3,1)-s.m(1)*ALPHA31-Fs32*C2+S2*(s.frc(2,2)+s.m(2)*(qd(2)*qd(2)*s.l(2,2)-ALPHA31*S2))-C3*(Fs33+Fq24*S4+Fq34*C4)+S3*(s.frc(2,3)-...
 s.m(3)*(ALPHA23+BS53*s.l(2,3))-Fq24*C4+Fq34*S4));

% = = Block_0_2_0_0_0_0 = = 
 
% Symbolic Outputs  

  Qq(1) = Fq31;
  Qq(2) = Cq12;
  Qq(3) = Cq13;
  Qq(4) = Cq14;
  Qq(5) = Cq15;
  Qq(6) = -s.trq(2,6);

% ====== END Task 0 ====== 

  

