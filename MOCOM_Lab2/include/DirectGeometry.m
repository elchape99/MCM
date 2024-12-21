function iTj_q = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function:
% This function calculate the rotation matrix from <i> to <j> conidering
% the actual value of q (jint vatiable)

% INPUTS: 
% qi:   current joint position;
% iTj:  is the constant transformation between the base of the link <i>
%       and its follower frame <j>; 
% jointType: 0 for R, 1 for P
%
% OUTPUT
% iTj_q:    transformation between the base of the joint <i> and its follower
%           frame considering the acutal value of 
% inizialize the matrix
iTj_q = zeros(4, 4);

if linkType == 0 % rotational -> qi is an angle
    % rotaton around z axis of qi angle
    Rz = [cos(qi), -sin(qi), 0; sin(qi), cos(qi), 0; 0,  0, 1];
    % generate the trasformation matrix for a revolut joint
    iTj_q(1:3, 1:3) = iTj(1:3, 1:3) * Rz; % product of the rotation matrix from the geom model and the rotation of qi 
                                          % around the z axis
    iTj_q(:, 4) = [iTj(1:3, 4); 1]; % the distance between the links remains the same

elseif linkType == 1 % prismatic -> qi is a linear displacement
    % since the movement is a traslation, no rotation occurs
    iTj_q(1:3, 1:3) = iTj(1:3, 1:3);

    rz = [0;0;1] * qi;
    iTj_q(:, 4) = [(iTj(:, 3)+rz); 1]; % the z axis is the conventional axis for the movement of the joint
end

end