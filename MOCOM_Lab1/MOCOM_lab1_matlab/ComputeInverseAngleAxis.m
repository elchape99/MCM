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
                theta = acos((trace(R)-1)/2)
                
                % Calculate eigenvalues and eigenvectors of R %check if the correct eigenvalue is v or -v
                v1 = -v;
                A = ComputeAngleAxis(theta, v);
                A1 = ComputeAngleAxis(theta, v1);
                if(norm(A-R) == tollerance * ones(3))
                    R = A;
                else
                    R = A1;
                [V,D] = eig(R) %V = eigenvectors, D = eigenvalues
                % Now I need to find the eigenvalue = 1, and this
                % eigenvalues is associated to the h eigenvector. For
                % definitions h(axes of rotation) is the eighenvectors with
                % eigenvalue = 1
                [rr,cc ] = size(D);
                for i=1:rr
                    for j =1:cc
                        if(abs(D(i,j)) >= 1-(4*10^-4) && (abs(D(i,j)) <= 1+(4*10^-4))) 
                            col_j=j;
                        end
                     end
                end
                v = V(:,col_j);
                v = v';
                % Compute the axis of rotation

                %check if the correct eigenvalue is v or -v
                v1 = -v;
                A = ComputeAngleAxis(theta, v);
                A1 = ComputeAngleAxis(theta, v1);
                if(norm(A-R) == tollerance * ones(3))
                    R = A;
                    theta = theta;
                    v = v;
                else
                    R = A1;
                    theta = theta;
                    v = -v;
                end                
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

