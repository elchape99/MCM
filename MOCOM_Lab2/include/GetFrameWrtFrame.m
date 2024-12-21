function [iTj]=GetFrameWrtFrame(linkNumber_i, linkNumber_j,biTei)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices from link i to link i +1 for the current q.
% The size of biTei is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformationMatrix in between link i and link j for the
% configuration described in biTei

% save the input frame to control for errors
    cycleInd = [int32(linkNumber_i), int32(linkNumber_j)];

    if max(cycleInd) <= 7 && min(cycleInd) >= 1 % the frame number must be between 1 and 7
        for i = min(cycleInd):max(cycleInd) % compute the tr matrix from the lower to the higher frame
            if i == min(cycleInd)
                iTj = biTei(:, :, i);
            else
                iTj = iTj * biTei(:, :, i);
            end
        end
        
        if linkNumber_i > linkNumber_j % if the desired matrix is the inverse 
            iTj = inv(iTj);
        elseif linkNumber_i == linkNumber_j % if the 2 frame are the same 
            iTj = [1,0,0,0;
                   0,1,0,0;
                   0,0,1,0;
                   0,0,0,1];
        end
    else
        err("ERROR: frame number must be between 1 and 7")
    end
end

