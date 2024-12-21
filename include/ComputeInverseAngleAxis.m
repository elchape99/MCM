function [theta,v] = ComputeInverseAngleAxis(R)
%EULER REPRESENTATION: Given a tensor rotation matrices this function
% should output the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'v' (axis) 
% SUGGESTED FUNCTIONS
    % size()
    % eye()
    % eig()
    % find()
    % abs()
    % det()
    % NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
    % and axis of rotation. Error messages must be displayed if the matrix
    % does not satisfy the rotation matrix criteria.
     I = eye(3,3);
     D = det(R);
     size_R = size(R);
    
    if size_R == 3   % Check matrix R to see if its size is 3x3
        
        % Check matrix R to see if it is orthogonal
        R_t = R';
        product = R * R_t;
        tollerance = 10^(-4);    
    
        if  norm(product - eye(size_R)) <= tollerance
            % I need to puta tollerance because the value is not prefect 1

            % Check matrix R to see if it is proper: det(R) = 1
            if det(R) >= 1-(4*10^(-4)) && det(R) <= 1+(4*10^(-4)) 
              % Compute the angle of rotation
                theta = acos((trace(R)-1)/2);
                
                % Compute the sin(theta)*V with vex
                tmp = (R - transpose(R))/2;
                vex = [tmp(3,2);tmp(1,3);tmp(2,1)];
                % compute v
                v = vex / sin(theta);                
            else
                error('DETERMINANT OF THE INPUT MATRIX IS NOT 1')
            end
        else
             err('NOT ORTHOGONAL INPUT MATRIX')
        end
    else
       error('WRONG SIZE OF THE INPUT MATRIX')
    end
end

