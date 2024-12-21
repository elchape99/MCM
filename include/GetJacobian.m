function J = GetJacobian(bTe, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
%-> the direct geometry vector of matrix

% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

numberOfLinks = max(size(jointType));
J = zeros(6, numberOfLinks); % creating the Jacobian of the correct dimension

for i = 1:numberOfLinks
    if jointType(i) == 0

        ki = [0;0;1]; % axis of joint i (OK) -> the z axis of frame i seen in frame i
        b_irn = bTe(1:3, 4, numberOfLinks) - bTe(1:3, 4, i); % vector from i to end-effector (OK)
        
        % vector projected on base frame 
        b_ki = bTe(1:3, 1:3, i)*ki; % projection of Zi axis on the base frame
        b_ki_irn = (cross(b_ki, b_irn));

        
        J(:, i) = [b_ki; b_ki_irn];
    elseif jointType(i) == 1
        ki = [0;0;1]; % axis of joint i
        b_irn = bTe(1:3, 4, numberOfLinks) - bTe(1:3, 4, i); % vector from i to end-effector
        
        % vector projected on base frame 
        b_ki = bTe(1:3, 1:3, i)*ki; % projection of Zi axis on the base frame
        b_ki_irn = cross(b_ki, b_irn);

        J(:, i) = [b_ki; b_ki_irn];
    else
        err("not a joint")
    end
end



end