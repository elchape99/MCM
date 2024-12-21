 %% This funtion implement the rodriguez formula for calculate the rotation of the frame against a vector v of an angle theta
function R = ComputeAngleAxis(theta,v)
%Implement here the Rodrigues formula
   % if norm(v) == 1
        sk_matrix = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        I = eye(3);
        % for i = 1:3
        %     I(i,i) = 1
        R = I + sin(theta)*sk_matrix + (1-cos(theta))*(sk_matrix)^2;
    %else
     %   fprintf(text,"error, the vector is NOT unix vector")

   % end
end