function matRot = rot(axis, theta)
% Computes the rotation matrix for a given axis and angle
%%%
% Authors
%   De Gréef Christophe,
%   Greiner Philippe,
%   Lefebvre Martin, 
%   Raucq Simon

if (axis == 1)
    matRot = [1 0 0;
        0 cos(theta) sin(theta);
        0 -sin(theta) cos(theta)];
else if (axis == 2)
        matRot = [cos(theta) 0 -sin(theta);
            0 1 0;
            sin(theta) 0 cos(theta)];
    else
        matRot = [cos(theta) sin(theta) 0;
            -sin(theta) cos(theta) 0;
            0 0 1];
    end
end


end

