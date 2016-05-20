function [yp] = integrationODE45(t, y)
global Data

% affichage du temps: a commenter pour faire tourner la simu plus vite
disp(['t: ' num2str(t)])

% recuperation des indices pour ecriture plus compacte
nqu = Data.nqu; 
ind_qu = Data.qu;  
nqv = Data.nqv; 
ind_qv = Data.qv; 
nqc = Data.nqc; 
ind_qc = Data.qc;  
ind_quc = [ind_qu, ind_qc];

% Copie des variables d'etat (y) dans les variables articulaires (qu, qdu)
Data.q(ind_qu) = y(1:nqu); 
Data.qd(ind_qu)= y(nqu+1:2*nqu); 
Data.tsim = t;

du = Data.qd(ind_quc);

%% Newton-Raphson to find v = q(qv)
% h(q) = 0
vk = Data.q(ind_qv);
vkplus1 = vk;

err = 1e-9;         % autorised error fo NR
n = 100;            % number of maximal iteration
i = 0;              % number of iteration
[qnew,qdnew,qddnew] = drivenJoints(Data);

while(norm(vkplus1-vk) > err || i<1)
	vk = vkplus1;
    Data.q(ind_qc) = qnew(ind_qc);
    Data.qd(ind_qc) = qdnew(ind_qc);
    Data.qdd(ind_qc) = qddnew(ind_qc);

	Data.q(ind_qv) = vk;
	[h, J] = cons_hJ(Data);
	Jv = J(:,ind_qv);
	vkplus1 = vk - Jv\h;
	if i==n
		error('NewtonRaphson not converging')
		break;
	end
	i = i+1;
end
Data.q(ind_qv) = vkplus1; %iterate value of coordinates

[h, J] = cons_hJ(Data);
Ju = J(:,ind_quc);
Jv = J(:,ind_qv);
Bvu = - Jv\Ju;


%% Linear system to find dv = dq(qv)
dv = Bvu*du;

Data.qd(ind_quc) = du;
Data.qd(ind_qv) = dv;

%% Calculation of force and momentum to find d2u

% Ici normalement: calcul des forces
[M, C] = dirdyna();
Q = zeros(6,1);%JointForces();
JdQd = cons_jdqd(Data);

Muu = M(ind_quc,ind_quc);
Muv = M(ind_quc,ind_qv);
Mvu = M(ind_qv,ind_quc);
Mvv = M(ind_qv,ind_qv);
Cu = C(ind_quc);
Cv = C(ind_qv);
Qu = Q(ind_quc);
Qv = Q(ind_qv);
bCalculs = - Jv\JdQd;

Mr = (Muu + Muv*Bvu + Bvu.'*Mvu + Bvu.'*Mvv*Bvu); %coordinate partitioning matrix Mr(u)
Fr = (Muv + Bvu.'*Mvv)*bCalculs + (Cu + Bvu.'*Cv) - (Qu + Bvu.'*Qv); % coordinate paritioning matrix Fr(du,u)

% Final form : Mr(u)*d2u + fr(du,u) = 0

if nqc > 0 % Driven coordinates, second partitioning
    nuc = length(ind_quc);
    
    Mr_uu = Mr(1:nqu, 1:nqu);
    Fr_u = Fr(1:nqu);
    Mr_uc = Mr(1:nqu, nqu+1:nuc);
    
    [qnew,qdnew,qddnew] = drivenJoints(Data);
    Data.qdd(ind_qc) = qddnew(ind_qc);
    Data.qd(ind_qc) = qdnew(ind_qc);
    Data.q(ind_qc) = qnew(ind_qc);
    
	Data.qdd(ind_qu) = Mr_uu\(-(Fr_u + Mr_uc*Data.qdd(ind_qc)));
else % No driven coordinates
	Data.qdd(ind_qu) = Mr\(-Fr); %value of the second derivative independant coordinates
end
%% Linear system to solve d2v = Bvu*d2q(qu) + b
d2v = Bvu*Data.qdd(ind_quc) + bCalculs;
Data.qdd(ind_qv) = d2v; %value of the second derivative dependant coordinates

%                                                             %
%                     Fin contribution                        %
%                                                             %
%                                                             %
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - %



% Copie des variables articulaires dans les variables d'etat
yp(1:nqu,1) = Data.qd(ind_qu); 

yp(nqu+1:2*nqu,1) = Data.qdd(ind_qu); 

end
%------------------------------------------------