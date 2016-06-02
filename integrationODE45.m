function [yp] = integrationODE45(t, y)
% Function used in ode45.
% From qu, qud => qudd. qvdd aswell if Data.computeIndependant != 0;
%%%
% Authors
%   De Gréef Christophe,
%   Greiner Philippe,
%   Lefebvre Martin, 
%   Raucq Simon
global Data

%% Display time. To comment to make it faster !
msg = sprintf('time = %.1f \n',t);
fprintf(msg);

%% To make it shorter
nqu = Data.nqu; 
ind_qu = Data.qu;  
nqv = Data.nqv; 
ind_qv = Data.qv; 
nqc = Data.nqc; 
ind_qc = Data.qc;  
ind_quc = [ind_qu, ind_qc];

%% Copy y into memory
Data.q(ind_qu) = y(1:nqu); 
Data.qd(ind_qu)= y(nqu+1:2*nqu); 
Data.tsim = t;

du = Data.qd(ind_quc);

%% Newton-Raphson to find v = q(qv)
% h(q) = 0
vk = Data.q(ind_qv);
vkplus1 = vk;

err = 1e-9;         % autorised error fo NR
n = 100;            % maximal number of iteration
i = 0;              % current number of iteration
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
Data.q(ind_qv) = vkplus1; % copy the solution in memory 

[h, J] = cons_hJ(Data);
Ju = J(:,ind_quc);
Jv = J(:,ind_qv);
Bvu = - Jv\Ju;


%% Linear system to find dv = dq(qv)
dv = Bvu*du;

Data.qd(ind_quc) = du;
Data.qd(ind_qv) = dv;

%% Dynamics to find d2u

[M, C] = dirdyna();
Q = zeros(6,1); %JointForces(); but equals to 0 !
JdQd = cons_jdqd(Data);

% First coordinate partitionning
Muu = M(ind_quc,ind_quc);
Muv = M(ind_quc,ind_qv);
Mvu = M(ind_qv,ind_quc);
Mvv = M(ind_qv,ind_qv);
Cu = C(ind_quc);
Cv = C(ind_qv);
Qu = Q(ind_quc);
Qv = Q(ind_qv);
bCalculs = - Jv\JdQd;

Mr = (Muu + Muv*Bvu + Bvu.'*Mvu + Bvu.'*Mvv*Bvu);                       % coordinate partitioning matrix Mr(u)
Fr = (Muv + Bvu.'*Mvv)*bCalculs + (Cu + Bvu.'*Cv) - (Qu + Bvu.'*Qv);    % coordinate paritioning matrix Fr(du,u)

% Final form : Mr(u)*d2u + fr(du,u) = 0

% Second coordinate partitioning
if nqc > 0 % If driven coordinates
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

%% Linear system to find d2v = Bvu*d2q(qu) + b
if(Data.computeIndependant ~= 0)
    d2v = Bvu*Data.qdd(ind_quc) + bCalculs;
    Data.qdd(ind_qv) = d2v; %value of the second derivative dependant coordinates
end


%% End of function :)
yp(1:nqu,1) = Data.qd(ind_qu); 
yp(nqu+1:2*nqu,1) = Data.qdd(ind_qu); 

end