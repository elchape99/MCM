%% Exercises Modelling Part 1
% Rotation matrices, Equivalent angle-axis representations, Quaternions
addpath('include') %%DO NOT CHANGE STUFF INSIDE THIS PATH

% Exercise 1
% Write a function called ComputeAngleAxis() implementing the Rodrigues Formula, 
% taking in input the geometric unit vector v and the rotation angle theta
% and returning the orientation matrix.
% and test it for the following cases:
% 1.2.
v2 = [1 0 0];
theta2 = pi/4;
aRb2 = ComputeAngleAxis(theta2, v2);

disp('aRb ex 1.1:');disp(aRb2);
plotRotation(theta2,v2,aRb2);
disp('theta ex 1.2:');disp(theta2);
disp('v ex 1.2:');disp(v2);


% 1.3.
v3 = [0 1 0];
theta3 = pi/6;
aRb3 = ComputeAngleAxis(theta3, v3);

disp('aRb ex 1.1:');disp(aRb3); 
plotRotation(theta3,v3,aRb3);
disp('theta ex 1.3:');disp(theta3);
disp('v ex 1.3:');disp(v3);


% 1.4.
v4 = [0 0 1];
theta4 = 3*(pi/4);
aRb4 = ComputeAngleAxis(theta4, v4);

disp('aRb ex 1.1:');disp(aRb4);
plotRotation(theta4,v4,aRb4);
disp('theta ex 1.4:');disp(theta4);
disp('v ex 1.4:');disp(v4);


% 1.5.
v5 = [0.3202, 0.5337, 0.7827];
theta5 = 2.8;

aRb5 = ComputeAngleAxis(theta5, v5);
disp('aRb ex 1.1:');disp(aRb5);
plotRotation(theta5,v5,aRb5);
disp('theta ex 1.5:');disp(theta5);
disp('v ex 1.5:');disp(v5);


% 1.6.
v6 = [0, 1, 0]
theta6 = 2*pi/3;

aRb6 = ComputeAngleAxis(theta6, v6);
disp('aRb ex 1.1:');disp(aRb6);
plotRotation(theta6,v6,aRb6);
disp('theta ex 1.2:');disp(theta6);
disp('v ex 1.2:');disp(v6);


% 1.7.
ro7 = [0.25, -1.3, 0.15];
theta7 = norm(ro7);
v7 = ro7 / theta7;

aRb7 = ComputeAngleAxis(theta7, v7);
disp('aRb ex 1.1:');disp(aRb7);
plotRotation(theta7,v7,aRb7);
disp('theta ex 1.7:');disp(theta7);
disp('v ex 1.7:');disp(v7);



% 1.8.
ro8 = [-pi/4, -pi/3, pi/6];
theta8 = norm(ro8);
v8 = ro8 / theta8;

aRb8 = ComputeAngleAxis(theta8, v8);
disp('aRb ex 1.1:');disp(aRb8);
plotRotation(theta8,v8,aRb8);
disp('theta ex 1.8:');disp(theta7);
disp('v ex 1.8:');disp(v8);



%% Exercise 2
% 2.1. Write the relative rotation matrix aRb
% 2.2. Solve the Inverse Equivalent Angle-Axis Problem for the orientation matrix aRb. 
% 2.3. Repeat the exercises using the wRc instead of wRa (more general example)
% NB: check the notation used !

    % 2.1 
    % Initialize the rotation matrices, using the suggested notation
        %rotation matrix from <w> to frame <a>
        wRa = [1, 0, 0; 0, 1, 0; 0, 0, 1];
        %rotation matrix from <w> to <b> (represent 90° around z)
        wRb = [0, -1, 0; 1, 0, 0; 0, 0, 1];
    % Compute the rotation matrix between frame <a> and <b>
    aRb = wRa' * wRb;
    % 2.2
    % Compute the inverse equivalent angle-axis repr. of aRb 
    [theta, v] = ComputeInverseAngleAxis(aRb);
    %Plot Results
    disp('aRb ex 2.1:');disp(aRb);
    plotRotation(theta,v,aRb);
    disp('theta ex 2.2:');disp(theta);
    disp('v ex 2.2:');disp(v);                             

   
    % 2.3

    % Compute the rotation matrix between frame <c> and <b>
    wRc = [0.835959, -0.283542, -0.46986; 0.271321, 0.957764, -0.0952472; 0.47703, -0.0478627 0.877583]
    cRb = wRc' * wRb;
    % Compute inverse equivalent angle-axis repr. of cRb
    [theta, v] = ComputeInverseAngleAxis(cRb);
    % Plot Results
    plotRotation(theta,v,cRb);
    disp('theta ex 2.3:');disp(theta);
    disp('v ex 2.3:');disp(v); 
    imagefilename="Images/es2.3_end.png";
    print(imagefilename,"-dpng");


%% Exercise 3
% 3.1 Given two generic frames < w > and < b >, define the elementary 
% orientation matrices for frame < b > with respect to frame < w >, knowing
% that:
%     a. < b > is rotated of 45◦ around the z-axis of < w >
%     b. < b > is rotated of 60◦ around the y-axis of < w >
%     c. < b > is rotated of -30◦ around the x-axis of < w >
% 
% 3.2 Compute the equivalent angle-axis representation for each elementary rotation
% 3.3 Compute the z-y-x (yaw,pitch,roll) representation using the already
% computed matrices and solve the Inverse Equivalent Angle-Axis Problem
% 
% 3.4 Compute the z-x-z representation using the already computed matrices 
% and solve the Inverse Equivalent Angle-Axis Problem 
% 
% 3.1
% hint: define angle of rotation in the initialization

    % a
        %rotation matrix from <w> to frame <b> by rotating around z-axes
        yaw = pi/4;
        wRb_z = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1 ];

    % b
        %rotation matrix from <w> to frame <b> by rotating around y-axes
        pitch = pi/3;
        wRb_y = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)];

    % c
        %rotation matrix from <w> to frame <b> by rotating around x-axes

        roll = -pi/6;
        wRb_x = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)];

        disp('es 3.1:');disp(wRb_z);disp(wRb_y);disp(wRb_x);
% 

%3.2
    % a       
        [theta, v] = ComputeInverseAngleAxis(wRb_z);
        % Plot Results
        plotRotation(theta,v,wRb_z);
        imagefilename="Images/es3.2_end.png";
        print(imagefilename,"-dpng");

        disp('theta ex 3.2.a:');disp(theta);
        disp('v ex 3.2.a:');disp(v); 
    % b

        [theta, v] = ComputeInverseAngleAxis(wRb_y);
        % Plot Results
        plotRotation(theta,v,wRb_y);
         imagefilename="Images/es3.2.y_end.png";
        print(imagefilename,"-dpng");
        disp('theta ex 3.2.b:');disp(theta);
        disp('v ex 3.2.b:');disp(v); 
    % c

        [theta, v] = ComputeInverseAngleAxis(wRb_x);
        % Plot Results
        plotRotation(theta,v,wRb_x);
         imagefilename="Images/es3.2.x_end.png";
        print(imagefilename,"-dpng");
        disp('theta ex 3.2.c:');disp(theta);
        disp('v ex 3.2.c:');disp(v); 
 % 3.3 
    % Compute the rotation matrix corresponding to the z-y-x representation;
    Rxyz = wRb_z * wRb_y * wRb_x
    % Compute equivalent angle-axis repr.
    [theta, v] = ComputeInverseAngleAxis(Rxyz);
    % Plot Results
    plotRotation(theta,v,Rxyz);
    imagefilename="Images/es3.3_end.png";
    print(imagefilename,"-dpng");
    disp('theta ex 3.3:');disp(theta);
    disp('v ex 3.3:');disp(v);


% 3.4
    % Compute the rotation matrix corresponding to the z-x-z representation;
    Rzxz = wRb_z * wRb_y * wRb_z
	% Compute equivalent angle-axis repr.
    [theta, v] = ComputeInverseAngleAxis(Rzxz);
    % Plot Results
    plotRotation(theta,v,Rzxz);
    imagefilename="Images/es3.4_end.png";
    print(imagefilename,"-dpng");
    disp('theta ex 3.4:');disp(theta);
    disp('v ex 3.4:');disp(v);


%% Exercise 4
% 4.1 Represent the following quaternion with the equivalent angle-axis
% representation. q = 0.1647 + 0.31583i + 0.52639j + 0.77204k
% 4.2 Solve the Inverse Equivalent Angle-Axis Problem for the obtained orientation matrix
% 4.3 Repeat the exercise using the built-in matlab functions see:
% https://it.mathworks.com/help/nav/referencelist.html?type=function&category=coordinate-system-transformations&s_tid=CRUX_topnav
% CHECK IF THE RESULT IS THE SAME 
    %%%%%%%%%%%%

    %%%%%%%%%%%%%
    % Compute the rotation matrix associated with the given quaternion
    q0 = 0.1647;
    q1 = 0.31583;
    q2 = 0.52639;
    q3 = 0.77204;
    rotMatrix0 = quatToRot(q0,q1,q2,q3);
    disp('rot matrix es 4.1');disp(rotMatrix0)



    % solve using matlab functions quaternion(), rotmat(),
    quat = quaternion(q0,q1,q2, q3)
    rotMatrix = rotmat(quat,"frame")

    % Evaluate angle-axis representation and display rotations - check if the
    % same results as before
    [theta, v] = ComputeInverseAngleAxis(rotMatrix);
    % Plot Results
    plotRotation(theta,v,rotMatrix);
    disp('rot matrix es 4.3');disp(rotMatrix)