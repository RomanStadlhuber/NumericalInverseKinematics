clc
clear
close all;

%% import the robot

% Denavit-Hartenberg parameters:
%           A           alpha   d       theta
dhparams = [0.025       0       0.4         0
            0.015       pi      0           0
            0           0       0           0
            0           0       0.15        0];

jTypes = ["revolute" "revolute" "prismatic" "revolute"];

robot = getRigidBodyTree(dhparams, jTypes);

tcpName = char(robot.BodyNames(robot.NumBodies));

%% setup IK

targetPositions = [ 0.08    0.0    -0.1
                    0.0     0.075   0.0
                    0.0     0.25    0.0];

[~, numWaypoints] = size(targetPositions);
waypoints = zeros(4, 4, numWaypoints);
for idxWaypoint = 1:numWaypoints
    waypoints(:,:, idxWaypoint) = trvec2tform(targetPositions(:,idxWaypoint).');
end
weights = [0 0 0 1 1 1];
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