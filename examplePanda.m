%% NOTES

% this test script follows the example in Lynch and Park's "Modern
% Robotics - Mechanics, Planning and Control"
% In Ch. 6, Example 6.1 demonstrates the application of numerical inverse
% kinematics on an RR robot in the XY-plane.

% Remarks:
% - computation of screws is explained in Ch. 3.3.3, see Ex. 3.26
% - computation of the space Jacobian is explained in Ch. 5.1.1 + examples
% - Ch. 6, p. 231 mentions to transform th error twist to the global frame
% using the adjoint matrix of the TCP pose

%% clear workspace and console
close all;
clear;
clc;

%% create robot

dhparams = [0       0       0.333   0;
            0       -pi/2   0       0;
            0       pi/2    0.316   0;
            0.0825  pi/2    0       0;
            -0.0825 -pi/2   0.384   0;
            0       pi/2    0       0;
            0.088   pi/2    0       0]; % DH_Params Panda


robot = getRigidBodyTree(dhparams);

tcpName = char(robot.BodyNames(robot.NumBodies));

%% create trajectory

% create a single target waypoint (as in the book example)
targetPose = [0.5   -0.5    0;
              0.5   1.0     -0.5;
              0.5   1.0     0.5];
targetOrientation = [0 0 0;
                    0 0 0; 
                    0 pi -pi/2];

trajectory = zeros([4 4 size(targetPose,2)]);

for k = 1:size(targetPose,2)

    trans = trvec2tform(targetPose(:,k).');                                    
    rot = eul2tform(targetOrientation(:,k).',"XYZ");                        
    trajectory(:,:,k) = [rot(1,1:3) trans(1,4);rot(2,1:3) trans(2,4);rot(3,1:3) trans(3,4);rot(4,1:3) trans(4,4)];

end

%T_sd = getTransform(robot, targetArticulation, tcpName); % T(30°, 60°)
%targetWaypoints = T_sd; % just a single target waypoint

%weights = [1 1 1 1 1 1];

%% run IK

[outTrajectory,outJointStates] = traceTrajectory(robot, tcpName, trajectory, 10, 1e-5);
[~, ~, numWaypoints] = size(outTrajectory);
disp("Generated " + numWaypoints + " waypoints");

%% Visualization

for i = 1:size(outTrajectory,3)
    xyz(i,:) = tform2trvec(outTrajectory(:,:,i));
end

figure('Visible','on')
show(robot,outJointStates(:,end));

% Visualisiere Target-Path und Ist-Path
hold on
plot3(xyz(:,1),xyz(:,2),xyz(:,3),'-k','LineWidth',2);
plot3(targetPose(1,:),targetPose(2,:),targetPose(3,:),'--r','LineWidth',2)
hold off