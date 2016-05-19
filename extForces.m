function externalForces = extForces(Data)
%% Force between wheel and ground
externalForces = zeros(3, Data.NBody);
r = 0.375;
Kr = 50e3;

h = Data.q(1)*Data.psi(:,1) + Data.dij(:,2,4) + Data.Rij(:,:,4).' * Data.dij(:,4,5) + ((Data.Rij(:,:,4).') * Data.Rij(:,:,5).')* Data.dij(:,5,6);
Z =  h(3) - r;%*cos(Data.q(6));

F_roue = zeros(3,1);
if(Z < 0)
    F_roue(3) = - Kr*Z;
end
externalForces(:,Data.NBody) = Data.Rij(:,:,6)*Data.Rij(:,:,5)*Data.Rij(:,:,4)*F_roue;

%% Link Force due to the suspension
K = 20e3;
D = 1e3;
l = 0.5;

d = Data.dij(:,2,4) +  Data.Rij(:,:,4).' * Data.dij(:,4,4) - Data.dij(:,2,3);   % Vector corresponding to the spring
dir = d/norm(d);                                                    % Direction of elongation of the spring
dldt = -(tilde(Data.qd(3)*Data.phi(:,3))*Data.dij(:,4,4)).'*dir;    % Elongation speed of the spring

F = Kr*(norm(d)-l) + D*dldt;
end

