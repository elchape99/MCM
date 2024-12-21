function J = GetJacobian(bTi, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% INPUTS:
% bTi:  vector of matrix (4, 4, numberOfLinks), contaning the trasformation
%       matrix from base to i joint
% - jointType: vector identifying the joint type, 0 for R, 1 for P
%
% OUTPUT:
% - J: end-effector jacobian matrix

numberOfLinks = max(size(jointType));
J = zeros(6, numberOfLinks); % creating the Jacobian of the correct dimension
bTe = bTi(:, :, numberOfLinks);
J_a = zeros(3, numberOfLinks);
J_l = zeros(3, numberOfLinks);

for i = 1:numberOfLinks

    if jointType(i) == 0
    % case of revolute joint
        % the angular part is equal to rotation axis
        J_a(:, i) = bTi(1:3, 3, i); 
        % distance vector from i to n 
        irn = bTe(1:3, 4) - bTi (1:3, 4, i);
        J_l(:, i) = cross(J_a(:, i) , irn);
        % select the column correspond to the joint considered
        J(:,i) = [J_a(:,i); J_l(:, i)];

       
    elseif jointType(i) == 1
    % case of prismatic joint
        % linear part is equal to the axis of movment 
        J_l(:, i) = bTi(1:3, 3, i);
        % angular part is eqial to zero
        J_a(:, i) = [0, 0, 0];
        % select the column correspond to the joint considered
        J(:,i) = [J_a(:,i); J_l(:, i)];

    else
        err("not a joint")
    end
end

