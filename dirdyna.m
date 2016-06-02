function [M,C] = dirdyna()
global Data;

% Computes M and C, used in equation of motion
%%%
% Authors
%   De Gréef Christophe,
%   Greiner Philippe,
%   Lefebvre Martin, 
%   Raucq Simon

Data.C = zeros(6,1);
%% Initialization
Data.alphac = zeros(3,Data.NBody);
Data.alphac(:,1) = [0; 0; 9.81];     % Gravity in the inertial frame
Data.betac = zeros(3,3,Data.NBody);

Data.omega = zeros(3,Data.NBody);
Data.omegacd = zeros(3,Data.NBody);

Data.OM = zeros(3, Data.NBody, Data.NBody);
Data.AM = zeros(3, Data.NBody, Data.NBody);
Data.Rij = zeros(3,3,Data.NBody);

Data.F = zeros(3, Data.NBody);
Data.L = zeros(3, Data.NBody);
Data.Wc = zeros(3, Data.NBody);
Data.Fc = zeros(3, Data.NBody);
Data.Lc = zeros(3, Data.NBody);
Data.WM = zeros(3, Data.NBody, Data.NBody);
Data.FM = zeros(3, Data.NBody, Data.NBody);
Data.LM = zeros(3, Data.NBody, Data.NBody);

%% Forward Kinematics
Data.z(:,2) = [0; 0; Data.q(1)];            % Update of the z-vector of the chassis

for i=2:Data.NBody
    h = Data.inBody(i-1);                   % Index of the parent body
    
    R_ih = eye(3);
    if(Data.phi(Data.axis(i-1),i-1) == 1)   % If the degree of freedom is a rotation, we calculate the rotation matrix R_ih
        R_ih = rot(Data.axis(i-1),Data.q(i-1));
    end
    
    Data.Rij(:,:,i) = R_ih;                 % Save the rotation matrix between the ith body and its parent h ==> X_i = R_ih* X_h
    Data.omega(:,i) = R_ih*Data.omega(:,h) + Data.phi(:,i-1)*Data.qd(i-1);
    Data.omegacd(:,i) = R_ih*Data.omegacd(:,h) + tilde(Data.omega(:,i))*Data.phi(:,i-1)*Data.qd(i-1); 
    Data.betac(:,:,i) = tilde(Data.omegacd(:,i)) + tilde(Data.omega(:,i))*tilde(Data.omega(:,i));
    Data.alphac(:,i) = R_ih*(Data.alphac(:,h) + Data.betac(:,:,h)*(Data.z(:,h) + Data.dij(:,h,i))) + 2*tilde(Data.omega(:,i))*Data.psi(:,i-1)*Data.qd(i-1);
    
    for k=2:i
        Data.OM(:,i,k) = R_ih*Data.OM(:,h,k) + (k==i)*Data.phi(:,i-1);
        Data.AM(:,i,k) = R_ih*(Data.AM(:,h,k) + tilde(Data.OM(:,h,k))*(Data.z(:,h) + Data.dij(:,h,i))) + (k==i)*Data.psi(:,i-1);
    end
    
end

%% External forces
Data.Fext = extForces(Data);
Data.Lext = zeros(3, Data.NBody);

%% Backward Dynamics
i = Data.NBody;
while(i > 1)
    Data.Wc(:,i) = Data.Body(i-1).m*(Data.alphac(:,i) + Data.betac(:,:,i)*(Data.z(:,i) + Data.dij(:,i,i))) - Data.Fext(:,i);
    Data.Fc(:,i) = Data.Wc(:,i);
    Data.Lc(:,i) = tilde(Data.z(:,i) + Data.dij(:,i,i))*Data.Wc(:,i) - Data.Lext(:,i) + Data.Body(i-1).I*Data.omegacd(:,i) + tilde(Data.omega(:,i))*Data.Body(i-1).I*Data.omega(:,i);
    for j=2:Data.NBody
        if(Data.inBody(j-1) == i)     % The parent body is the current body <==> The jth body is the children of the ith body
            Data.Fc(:,i) = Data.Fc(:,i) + Data.Rij(:,:,j).' * Data.Fc(:,j);
            Data.Lc(:,i) = Data.Lc(:,i) + Data.Rij(:,:,j).' * Data.Lc(:,j) + tilde(Data.z(:,i) + Data.dij(:,i,j))*Data.Rij(:,:,j).' * Data.Fc(:,j);
        end
    end
    
    for k=2:i
        Data.WM(:,i,k) = Data.Body(i-1).m*(Data.AM(:,i,k) + tilde(Data.OM(:,i,k))*(Data.z(:,i) + Data.dij(:,i,i)));
        Data.FM(:,i,k) = Data.WM(:,i,k);
        Data.LM(:,i,k) = tilde(Data.z(:,i) + Data.dij(:,i,i))*Data.WM(:,i,k) + Data.Body(i-1).I*Data.OM(:,i,k);
        for j=2:Data.NBody
            if(Data.inBody(j-1) == i)     % The parent body is the current body <==> The jth body is the children of the ith body
                Data.FM(:,i,k) = Data.FM(:,i,k) + Data.Rij(:,:,j).' * Data.FM(:,j,k);
                Data.LM(:,i,k) = Data.LM(:,i,k) + Data.Rij(:,:,j).' * Data.LM(:,j,k) + tilde(Data.z(:,i) + Data.dij(:,i,j))*Data.Rij(:,:,j).' * Data.FM(:,j,k);
            end
        end
    end
    i = i-1;
end

%% Mass matrix M, non-linear term C, independent term Q
for i=2:Data.NBody
    Data.C(i-1) = Data.psi(:,i-1)'*Data.Fc(:,i) + Data.phi(:,i-1)'*Data.Lc(:,i);
    for j=2:i
        Data.M(i-1,j-1) = Data.psi(:,i-1)'*Data.FM(:,i,j) + Data.phi(:,i-1)'*Data.LM(:,i,j);
    end
end
Data.M(:,:) = (Data.M(:,:) + Data.M(:,:).')/2;


M = Data.M;
C = Data.C;

end