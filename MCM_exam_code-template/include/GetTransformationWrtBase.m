function bTi = GetTransformationWrtBase(biTei, numberOfLinks)
%%% GetTransformatioWrtBase function
% 
% INPUT:
% biTei: vector of matrices containing the transformation matrices from link i to link i +1 for the current joint position q.
% The size of biTei is equal to (4,4,numberOfLinks)
% numberOfLinks:    total number of links 
% 
% OUTPUTS:
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by biTei.
%bTi = eye(4, 4, numberOfLinks);

for i = 1:numberOfLinks
    if i == 1
        % only for the first iteration
        bTi(:, :, i) = biTei(:, :, i);
    else
        % moltiply the trasfromation from the base to the precedent frame
        % with the trasformation matrix of this frame. In this way I obtain
        % the trasformation matrix from the base to the actual frame
        bTi(:,: ,i) = bTi(:, : ,i-1) * biTei(:, :, i);
    end
end

end