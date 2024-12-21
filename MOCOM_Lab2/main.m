%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');



% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7;                    % number of manipulator's links.
linkType = [0,0,0,0,0,0,0];                         % boolean that specifies two possible link types: Rotational, Prismatic.
bri= zeros(3,numberOfLinks);        % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);     % Trasformation matrix i-th link w.r.t. base


% Initial joint configuration 
q = [0,0,0,0,0,0,0]; % plotting the position for the "rest" configuration of the robot

q_plot = plotting_fixed_robot_position(geom_model, numberOfLinks, 1);


% Q1.1 and Q1.2
% to be able to plot the following configuration I have to compute the
% Direct geometry first
q1= [0, 0, 0, 0, 0, pi/2, 0];
q2= [0, pi/2, 0,-pi/2, 0, 0, 0];
q3= [pi/4, pi/2,-pi/8,-pi/2, pi/4, 2/(3*pi), 0];

iTj_q1 = GetDirectGeometry(q1, geom_model, linkType, numberOfLinks);
q1_plot = plotting_fixed_robot_position(iTj_q1, numberOfLinks, 2);

iTj_q2 = GetDirectGeometry(q2, geom_model, linkType, numberOfLinks);
q2_plot = plotting_fixed_robot_position(iTj_q2, numberOfLinks, 3);

iTj_q3 = GetDirectGeometry(q3, geom_model, linkType, numberOfLinks);
q3_plot = plotting_fixed_robot_position(iTj_q3, numberOfLinks, 4);

% Q1.4
%initial and final configuration to be animated
qi_1 = [0, 0, 0, 0, 0, 0, 0];
qf_1 = [pi/4, pi/2,-pi/8,-pi/2, pi/4, 2/(3*pi), 0];

qi_2 = [0, pi/2, 0,-pi/2, 0, 0, 0]; 
qf_2 = [0, 0, 0, 0, 0, 0, 0];

qi_3 = [1.3, 0.1, 0.1, 1, 0.2, 0.3, 1.3]; 
qf_3 = [2, 2, 2, 2, 2, 2, 2];

STR_ND(:, :, 1) = [qi_1; qf_1]; % temporary matrix used to store initial e final 
STR_ND(:, :, 2) = [qi_2; qf_2]; % joint variable configuration
STR_ND(:, :, 3) = [qi_3; qf_3];

% the algoritm will go as follow: for each couple of config we need to
% create intermediate configuration, compute the direct geometry, project
% the new configuration on the base frame an then plot the motion

numberOfStep = 100; % "frame" of the motion

qif1_plot = zeros(3, numberOfLinks, numberOfStep);
qif2_plot = zeros(3, numberOfLinks, numberOfStep);
qif3_plot = zeros(3, numberOfLinks, numberOfStep);

config_matrix = zeros(3, numberOfLinks, numberOfStep);
limitmat = zeros(3, 6);
 
for i = 1:3 % one cycle for each couple of joint var vector
    q = STR_ND(1,:,i); % starting joint variable vector 
    deltaq = (STR_ND(2,:,i)-STR_ND(1,:,i))/numberOfStep; % variation of the configuration
    q_init = zeros(1, numberOfLinks);
    % useful initialization
    biTei = zeros(4, 4, numberOfLinks);
    bri = zeros(3, numberOfLinks);

    for j = 1:numberOfStep % one repetition for each frame

        iTj = GetDirectGeometry(q_init , geom_model, linkType, numberOfLinks); % direct geometry computation
        bTe = GetTransformationWrtBase(iTj, numberOfLinks)

        for k = 1:numberOfLinks
            biTei(:, :, k) = GetTransformationWrtBase(iTj, k); % projection on base frame
        end

        for k = 1:numberOfLinks
            bri(:, k) = GetBasicVectorWrtBase(biTei, k); % retriving the link position
        end

        q = q+deltaq;
        
        if i == 1
            qif1_plot(:, :, j) = bri;
            config_matrix(:, :, j) = bri;
        elseif i == 2
            qif2_plot(:, :, j) = bri;
            config_matrix(:, :, j) = bri;
        else
            qif3_plot(:, :, j) = bri;
            config_matrix(:, :, j) = bri;
        end

    end

    figure(10*i); % plotting
    [xlim, ylim, zlim] = get_axis_limits(config_matrix);
    axislimits = [xlim, ylim, zlim];
    limitmat(i, :) = axislimits;

    for k =  1:numberOfStep
        plot3(config_matrix(1, :, k), config_matrix(2, :, k), config_matrix(3, :, k),'-o','Color','b', 'MarkerSize',7, 'LineWidth',5, MarkerEdgeColor='r');
        axis(axislimits);
        grid on; drawnow; pause(1/60)
    end
    
    pause(2);
end



% Q1.5
% Plot the same transformation moving one joint at a time 
% joint_vector_matrix_1 = one_joint_at_a_time(STR_ND(:, :, 1), numberOfLinks);
% joint_vector_matrix_2 = one_joint_at_a_time(STR_ND(:, :, 2), numberOfLinks);
% joint_vector_matrix_3 = one_joint_at_a_time(STR_ND(:, :, 3), numberOfLinks);

for i = 1:3
    joint_vector_matrix = one_joint_at_a_time(STR_ND(:, :, i), numberOfLinks);

    for j = 1:numberOfLinks
        q = joint_vector_matrix(1, :, j);

        for k = 1:numberOfStep
            deltaq = (joint_vector_matrix(2, :, j)-joint_vector_matrix(1, :, j))/numberOfStep;

            iTj = GetDirectGeometry(q, geom_model, linkType, numberOfLinks);

            
            for h = 1:numberOfLinks
                biTei(:, :, h) = GetTransformationWrtBase(iTj, h);
            end

            for h = 1:numberOfLinks
                bri(:, h) = GetBasicVectorWrtBase(biTei, h);
            end

            q = q+deltaq;

            config_matrix(:, :, k) = bri;
        end

        figure(100*i);
    
        for h =  1:numberOfStep
            plot3(config_matrix(1, :, h), config_matrix(2, :, h), config_matrix(3, :, h),'-o','Color','b', 'MarkerSize',7, 'LineWidth',3, MarkerEdgeColor='r');
            axis(limitmat(i, :));
            grid on; drawnow; pause(1/60);
        end
    
        pause(2);
    end
end




