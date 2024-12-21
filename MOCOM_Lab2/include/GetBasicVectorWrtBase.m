function [r]=GetBasicVectorWrtBase(biTei, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% biTei: vector of matrices containing the trasformation the trasf between
% the base and each link
% linkNumber: link number 
% output
% r : basic vector from frame i to the robot base frame <0>

r = biTei(1:3, 4, linkNumber);

end