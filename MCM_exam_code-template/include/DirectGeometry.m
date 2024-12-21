function iTj_q = DirectGeometry(qi, geomModel, linkType)
% DirectGeometry Function:
% This function calculate the trasformation matrix from <i> to <j> conidering
% the actual value of q (joint vatiable)

% INPUTS: 
% qi:           current joint variaable
% geomModel:    is the geometric model of the robot
% jointType:    0 for R, 1 for P
%
% OUTPUT
% iTj_q:    transformation between the joint <i> and its following frame 
%           <j> considering the acutal value of q 

% inizialize the matrix
    iTj_q = zeros(4, 4);
    
    if linkType == 0 % rotational -> qi is an angle
        % rotaton around z axis of qi angle
        Rz = [cos(qi), -sin(qi), 0; sin(qi), cos(qi), 0; 0,  0, 1];
        % generate the trasformation matrix for a revolut joint
        iTj_q(1:3, 1:3) = geomModel(1:3, 1:3) * Rz;   % product of the rotation matrix from the geom model and the rotation of qi 
                                                    % around the z axis
        iTj_q(:, 4) = [geomModel(1:3, 4); 1]; % the distance between the links remains the same
    
    
    elseif linkType == 1 % prismatic -> qi is a linear displacement
        % since the movement is a traslation, no rotation occurs
        iTj_q(1:3, 1:3) = geomModel(1:3, 1:3);
    
        rz = [0;0;1] * qi;
        iTj_q(:, 4) = [(geomModel(:, 3)+rz); 1]; % the z axis is the conventional axis for the movement of the joint
    end

end