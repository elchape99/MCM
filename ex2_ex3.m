%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about eventual warnings!
%model.franka.inverseDynamics(q, )
%% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 60.0;
t = t_start:ts:t_end;

%% Initial manipulator configuration
% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits (saturation)
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Function that gives the transformation from <base> to <e-e>, given a
% configuration of the manipulator
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7');%DO NOT EDIT 
bTe_init = bTe;% usefull for plot the initial position of the manipulatorc
%% Tool frame definition
% distnce between tool and ee
eOt = [0, 0, 0.2104]';  
% rotation between 
phi = deg2rad(-44.98);
eRt = [ cos(phi),   -sin(phi),  0
        sin(phi),   cos(phi),   0  
        0,          0,          1];

% trasformation matrix from <ee> to <tool>
eTt = [ eRt,        eOt;
        zeros(1,3), 1];

% trasformation matrix from <base> to <tool>
bTt = bTe * eTt;
% save a new variable forplot the initial configuration of manipulator
bTt_init = bTt;

%% Goal definition 
% the goal distance of the goal-frame respect to base
bOg = [0.55, -0.3, 0.2]';

% Switch between the two cases (with and without the tool frame)
tool = false;
% change to true for using the tool
if tool == true
    % frame is rotated of theta around y-axis of the robot tool initial
    % configuration
    theta = pi/6;
    % rotation around y axis
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
    % rotation mtrix from <base> to <goal>
    bRg = bTt(1:3, 1:3) * Ry;
    % Trasformation matrix from <base> to <goal>
    bTg = [ bRg(1,1), bRg(1,2), bRg(1,3), bOg(1);
            bRg(2,1), bRg(2,2), bRg(2,3), bOg(2);
            bRg(3,1), bRg(3,2), bRg(3,3), bOg(3);
            0,        0,        0,        1];


else % no tool, only e-e
    theta = pi/6;
    % Rotation matrix around y-axis of angle theta
    Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0, cos(theta)];
   % extract the Rotation matrix from <base> to <goal>
    bRg = bTe(1:3, 1:3) * Ry;
    % Transformation matrix from <base> to <goal>
    bTg = [ bRg(1,1), bRg(1,2), bRg(1,3), bOg(1);
            bRg(2,1), bRg(2,2), bRg(2,3), bOg(2);
            bRg(3,1), bRg(3,2), bRg(3,3), bOg(3);
            0,        0,        0,        1];
end   

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
lin_err = zeros(3,1);
ang_err = zeros(3,1); 
% Start the inverse kinematic control  
q = q_init;

%% Simulation Loop
for i = t
   disp(i);
    if tool == true % compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka, [q',0,0], 'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka, [q',0,0], 'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6, 1:7); %DO NOT EDIT
        % Trasformation matrix from <base> to <tool>
        bTt = bTe * eTt;
        % I need to write the rigid body jcobian of tool in end effector
        
        beOt = bTe * [eOt; 0];
        % define the skew simmetric matrix of ert:
        skew_ert = [ 0,          -beOt(3),   0;
                     beOt(3),    0,          0;
                     0,          0,          0;];
        % rigud body jacobian of poit t in body n
        eSt = [eye(3,3),  zeros(3,3);
               skew_ert,       eye(3,3) ];
        % jacobian between t and e, 6X1        
        bJt = eSt * bJe;  

        % linear error: grb(distance goal-base) - trb(distance tool-base)
        lin_err = bTg(1:3, 4) - bTt(1:3, 4);
        % angular error 
        % rotational matrix from tool to goal
        tRg = bTt(1:3,1:3)' * bTg(1:3,1:3);
        % find the angle-axis rappresentation of the rotational matix
        [theta, v] = ComputeInverseAngleAxis(tRg);
        rho = theta * v;
        % project the vector rho in the frame <b>
        ang_err = bTt(1:3, 1:3) * rho;

    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka, [q',0,0], 'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
% linear error
        lin_err = bTg(1:3, 4) - bTe(1:3, 4);       
        % angular error 
        % rotational matrix from tool to goal
        eRg = bTe(1:3,1:3)' * bTg(1:3,1:3);
        % find the angle-axis rappresentation of the rotational matix
        [theta, v] = ComputeInverseAngleAxis(eRg);
        rho = theta * v;
        % project the vector rho in the frame <b>
        ang_err = bTe(1:3, 1:3) * rho;

    end
    
       
    %% Compute the reference velocities linear and angular
    
    v_ref = linear_gain * lin_err;
    angular_ref = angular_gain * ang_err;
    %linear gain per linear error
    % angular gain per agular error

   
    %% Compute desired joint velocities 
     % q dot = pseudoinvers(j) * x dot dot velocityes of reference point
     % before
     % x_dot: actual velocity of joint
     % q_dot desired velocity of joint
    if tool ==  true 
        x_dot = [angular_ref; v_ref];
        q_dot = pinv(bJt) * x_dot;
    else     
        x_dot = [angular_ref; v_ref];
        q_dot = pinv(bJe) * x_dot;
    end
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);    
    % DO NOT EDIT - plot the robot moving
    %switch visuals to off for seeing only the frames
    model_franka = show(model.franka, [q',0,0], 'visuals', 'on');
    hold on
    if tool == true
        %set the window size of the figure to "full-screen" for a better visualization
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
        quiver3([bOg(1);bOg(1);bOg(1)],[bOg(2);bOg(2);bOg(2)],[bOg(3);bOg(3);bOg(3)], bRg(:, 1), bRg(:, 2), bRg(:, 3), "LineWidth", 3,"Color",[1,0,0]);
        quiver3([bTt_init(1,4);bTt_init(1,4);bTt_init(1,4)],[bTt_init(2,4);bTt_init(2,4);bTt_init(2,4)],[bTt_init(3,4);bTt_init(3,4);bTt_init(3,4)], bTt_init(1:3, 1), bTt_init(1:3, 2),bTt_init(1:3, 3), "LineWidth", 3);
        quiver3([bTt(1,4);bTt(1,4);bTt(1,4)],[bTt(2,4);bTt(2,4);bTt(2,4)],[bTt(3,4);bTt(3,4);bTt(3,4)], bTt(1:3, 1), bTt(1:3, 2),bTt(1:3, 3), "LineWidth", 3,"Color",[0,1,0]);
      
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
        quiver3([bOg(1);bOg(1);bOg(1)],[bOg(2);bOg(2);bOg(2)],[bOg(3);bOg(3);bOg(3)], bRg(:, 1), bRg(:, 2), bRg(:, 3), "LineWidth", 3,"Color",[1,0,0]);
        quiver3([bTe_init(1,4);bTe_init(1,4);bTe_init(1,4)],[bTe_init(2,4);bTe_init(2,4);bTe_init(2,4)],[bTe_init(3,4);bTe_init(3,4);bTe_init(3,4)], bTe_init(1:3, 1), bTe_init(1:3, 2),bTe_init(1:3, 3), "LineWidth", 3);
        quiver3([bTe(1,4);bTe(1,4);bTe(1,4)],[bTe(2,4);bTe(2,4);bTe(2,4)],[bTe(3,4);bTe(3,4);bTe(3,4)], bTe(1:3, 1), bTe(1:3, 2),bTe(1:3, 3), "LineWidth", 3,"Color",[0,1,0]);
    end
    drawnow
    if(norm(x_dot) < 0.001)
        disp(t)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    hold off;
end % end of the for loop


