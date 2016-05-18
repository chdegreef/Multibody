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
%	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
%	==> Flops complexity : 29
%
%	==> Generation Time :  0.000 seconds
%	==> Post-Processing :  0.000 seconds
%
%-------------------------------------------------------------
%
function [h,Jac] = cons_hJ(s,tsim,usrfun)

 h = zeros(2,1);
 Jac = zeros(2,6);

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

% = = Block_0_1_0_0_0_2 = = 
 
% Constraints and Constraints Jacobian 

%
  RL_1_28 = s.dpt(2,3)*C2;
  RL_1_38 = s.dpt(2,3)*S2;

% = = Block_0_1_0_0_0_3 = = 
 
% Trigonometric Variables  

%
  S3p4 = C3*S4+S3*C4;
  C3p4 = C3*C4-S3*S4;
 
% Constraints and Constraints Jacobian 

  RL_0_24 = s.dpt(2,4)*C3;
  RL_0_34 = s.dpt(2,4)*S3;
  RL_0_27 = -s.dpt(3,6)*S3p4;
  RL_0_37 = s.dpt(3,6)*C3p4;
  JT_0_27_3 = -(RL_0_34+RL_0_37);
  JT_0_37_3 = RL_0_24+RL_0_27;

% = = Block_0_1_0_0_1_0 = = 
 
% Constraints and Constraints Jacobian 

%
  h_2 = RL_0_24+RL_0_27-RL_1_28-s.dpt(2,1)+s.dpt(2,2);
  h_3 = RL_0_34+RL_0_37-RL_1_38+s.dpt(3,2);

% = = Block_0_3_0_0_0_0 = = 
 
% Symbolic Outputs  

  h(1) = h_2;
  h(2) = h_3;
  Jac(1,2) = RL_1_38;
  Jac(1,3) = JT_0_27_3;
  Jac(1,4) = -RL_0_37;
  Jac(2,2) = -RL_1_28;
  Jac(2,3) = JT_0_37_3;
  Jac(2,4) = RL_0_27;

% ====== END Task 0 ====== 

  

