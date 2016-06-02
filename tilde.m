function matTilde = tilde(u)
% Computes the "tilde" matrix
%%%
% Authors
%   De Gréef Christophe,
%   Greiner Philippe,
%   Lefebvre Martin, 
%   Raucq Simon

    matTilde = [0       -u(3)   u(2); ...
                u(3)    0      -u(1); ...
                -u(2)   u(1)    0];
end

