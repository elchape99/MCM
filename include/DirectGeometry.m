function iTj_q = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function 
% inputs: 
% qi : current joint position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>; 
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_q : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/traslation of the joint

iTj_q = zeros(4, 4);

if linkType == 0 % rotational -> qi is an angle
    Rz = [cos(qi), -sin(qi), 0; sin(qi), cos(qi), 0; 0,  0, 1];
    % generate the trasformation matrix for a revolut joint
    iTj_q(1:3, 1:3) = iTj(1:3, 1:3) * Rz; % product of the rotation matrix from the geom model and the rotation of qi 
                                          % around the z axis
    iTj_q(:, 4) = [iTj(1:3, 4); 1]; % the distance between the links remains the same

elseif linkType == 1 % prismatic -> qi is a linear displacement
    iTj_q(1:3, 1:3) = iTj(1:3, 1:3); % since the movement is a traslation, no rotation occurs

    rz = [0;0;1] * qi;
    iTj_q(:, 4) = [(iTj(:, 3)+rz); 1]; % the z axis is the conventional axis for the movement of the joint
end

end