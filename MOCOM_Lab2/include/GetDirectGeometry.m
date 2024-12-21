function [iTj_q] = GetDirectGeometry(q, iTj, linkTypoe, numberOfLinks)
%%% GetDirectGeometryFunction

% INPUTS: 
% q:    actual joint variable 
% iTj:  vector of matrices containing the transformation matrices from
%       previous joint <i> to next joint <j>
% jointType: 0 for R, 1 for P his size is (1, numberOfLinks)

% OUTPUTS:
% iTj_q:    vector of matrices containing the transformation matrices from 
%           previous joint <i> to next joint <j> considering the actual 
%           jOint vairable q 
%           The size of iTj_q is equal to (4,4,numberOfLinks)

    iTj_q = zeros(4, 4, numberOfLinks);
    
    for i = 1:1:numberOfLinks
        iTj_q(:, :, i) = DirectGeometry(q(i), iTj(:, :, i), linkTypoe(i));
    end

end