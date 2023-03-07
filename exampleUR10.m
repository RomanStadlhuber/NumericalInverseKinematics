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
jTypes = ["revolute" "revolute" "revolute" "revolute" "revolute" "revolute"];

robot = getRigidBodyTree(dhparams, jTypes);

tcpName = char(robot.BodyNames(robot.NumBodies));

%% setup IK

targetPositions = [ 0.17   0.12   0.05  -0.03  -0.12  -0.10  -0.14  -0.18  -0.15
                    0.09   0.15   0.20   0.20   0.15   0.15   0.08  -0.10  -0.07
                    0.05   0.10   0.08   0.14   0.20   0.16   0.07   0.04   0.20];

targetPositions = 2.5 .* targetPositions;

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
% set to true to obtain diagnostic information in the workspace
diagnosticMode = false;
%% run IK
[outTrajectory, outJointStates] = traceTrajectory(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess, diagnosticMode);

%% plot trajectory
viz(robot, outTrajectory, targetPositions, outJointStates);