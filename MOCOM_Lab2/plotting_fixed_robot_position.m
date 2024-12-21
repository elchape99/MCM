function [bri] = plotting_fixed_robot_position(Direct_geometry, numberOfLinks, figureNum)
% function used to plot single configurtion from the DirectGeometry matrix
bri= zeros(3,numberOfLinks);
bTi = zeros(4,4,numberOfLinks);  

% plotting starting position
for i =1:numberOfLinks
    % projection on base frame
    bTi(:,:,i)= GetTransformationWrtBase(Direct_geometry, i); 
end

for i = 2:numberOfLinks
    % extraction of the link
    bri(:,i) = GetBasicVectorWrtBase(bTi, i);
end

figure(figureNum)
plot3(bri(1, :), bri(2, :), bri(3, :), '-o', 'Color','r', 'MarkerSize',10);
grid on

end