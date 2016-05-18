function externalForces = extForces(Data)
externalForces = zeros(3, Data.NBody);

%% Force between wheel and ground
r = 0.375;
Kr = 50e3;

h = Data.q(1)*Data.psi(:,1) + Data.dij(:,2,4) + Data.Rij(:,:,4).' * Data.dij(:,4,5) + ((Data.Rij(:,:,4).') * Data.Rij(:,:,5).')* Data.dij(:,5,6);
Z =  h(3) - r*cos(Data.q(6));

F_roue = zeros(3,1);
if(Z < 0)
    F_roue(3) = - Kr*Z;
end
externalForces(:,Data.NBody) = Data.Rij(:,:,6)*Data.Rij(:,:,5)*Data.Rij(:,:,4)*F_roue;
end
