%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
clear
clc

addpath('include');

% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q1 = [1.8, 1.8, 1.8, 1.8, 1.8, 1.8, 1.8];
q2 = [0.3, 1.4, 0.1, 2.0, 0, 1.3, 0];
q3 = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0];
q4 = [1, 1, 1, 1, 1, 1, 1];

[J_1, biTei_1, bTe_1]   = compute_Jacobian(geom_model, q1, linkType, numberOfLinks);
[J_2, biTei_2, bTe_2]   = compute_Jacobian(geom_model, q2, linkType, numberOfLinks);
[J_3, biTei_3, bTe_3]   = compute_Jacobian(geom_model, q3, linkType, numberOfLinks);
[J_4, biTei_4, bTe_4]   = compute_Jacobian(geom_model, q4, linkType, numberOfLinks);

