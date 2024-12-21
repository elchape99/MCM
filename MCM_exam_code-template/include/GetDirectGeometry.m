function [biTei] = GetDirectGeometry(q, geomModel, linkType)
%%% GetDirectGeometryFunction

% INPUTS: 
% q:            actual joint variable 
% geomModel:    contain all the geometry Model, is the tree of the robot,
%               don't consider the joint varible
% jointType:    0 for R, 1 for P his size is (1, numberOfLinks)

% OUTPUTS:
% iTj:  vector of matrices containing the transformation matrices from 
%       previous joint <i> to next joint <j> considering the actual joint
%       vairable q 
%       The size of iTj is equal to (4,4,numberOfLinks)

    numberOfLinks = size(linkType, 1);
    biTei = zeros(4, 4, numberOfLinks);
    % Put all the matrices computed by DirectGeometry inside a vector of matrix 
    for i = 1:1:numberOfLinks
        biTei(:, :, i) = DirectGeometry(q(i), geomModel(:, :, i), linkType(i));
    end

end