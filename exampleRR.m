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

l1 = 1.0;
l2 = 1.0;
dhparams = [l1   0   0  0;
            l2   0   0  0];

jTypes = ["revolute" "revolute"];

robot = getRigidBodyTree(dhparams, jTypes);

tcpName = char(robot.BodyNames(robot.NumBodies));

%% setup IK

targetPositions = [ 1.5     0.0     -0.5
                    0.5     1.5     1.0
                    0.0    0.0     0.0];

[~, numWaypoints] = size(targetPositions);
waypoints = zeros(4, 4, numWaypoints);
for idxWaypoint = 1:numWaypoints
    waypoints(:,:, idxWaypoint) = trvec2tform(targetPositions(:,idxWaypoint).');
end
weights = [0 0 0 1 1 1];
% initialGuess = monteCarloInitialGuess(robot, tcpName, waypoints(:,:,1));
initialGuess = homeConfiguration(robot);
minDistance = 1e-5;
maxIterations = 150;

%% run IK
[outTrajectory, outJointStates] = traceTrajectory(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess);

%% plotting

figure('Visible','on')
show(robot,outJointStates(:,end));

xyz = zeros(numWaypoints, 3);
for i = 1:size(outTrajectory,3)
    xyz(i,:) = tform2trvec(outTrajectory(:,:,i));
end

hold on
plot3(xyz(:,1),xyz(:,2),xyz(:,3),'-k','LineWidth',2);
plot3(targetPositions(1,:),targetPositions(2,:),targetPositions(3,:),'--r','LineWidth',2)
hold off