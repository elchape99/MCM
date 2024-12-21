function [rot_matrix] = quatToRot(q0,q1,q2,q3)
% quatToRot convert a quaternion into a rotation matrix
    %Covert a quaternion into a full three-dimensional rotation matrix.
 
    %Input
    %:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    %Output
    %return: A 3x3 element matrix representing the full 3D rotation matrix. 

    %3x3 rotation matrix     
    % Extract quaternion components
    w = q0;
    x = q1;
    y = q2;
    z = q3;

    % Calculate the elements of the rotation matrix
    rot_matrix = [1 - 2*y^2 - 2*z^2, 2*x*y - 2*w*z, 2*x*z + 2*w*y;
                  2*x*y + 2*w*z, 1 - 2*x^2 - 2*z^2, 2*y*z - 2*w*x;
                  2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x^2 - 2*y^2];
    % mu = q0;
    % epsilon = [q1, q2, q3]
    % skew_epsilon = [0 -q3 q2; q3, 0, -q1; -q2, q1, 0]
    % rot_matrix = eye(3) - 2*mu*skew_epsilon + 2 * (skew_epsilon^2);


end