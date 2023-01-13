clc
clear
close all;

%% import the robot

% Denavit-Hartenberg parameters:
%           A           alpha   d       theta
dhparams = [0           pi/2    0.1273      0
            -0.312      0       0           0
            -0.5723     0       0           0
            0           pi/2    0.163941    0
            0           -pi/2   0.1157      0
            0           0       0.0922      0];
% NOTE: parameters obtained from
% https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

robot = getRigidBodyTree(dhparams);

tcpName = char(robot.BodyNames(robot.NumBodies));

%% setup IK

targetPositions = [ 0.75    0.0    -0.75
                    0.0     0.75   0.0
                    0.6     0.8    0.6];

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
% plot3(waypoints(1, 4, :),waypoints(2, 4, :),waypoints(3, 4, :),'--r','LineWidth',2)
hold off