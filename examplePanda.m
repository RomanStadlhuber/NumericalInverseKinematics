%% clear workspace and console
close all;
clear;
clc;

%% create robot


% Denavit-Hartenberg parameters:
%           A       alpha   d   theta
dhparams = [0       0       0.333   0
            0       -pi/2   0       0
            0       pi/2    0.316   0
            0.0825  pi/2    0       0
            -0.0825 -pi/2   0.384   0
            0       pi/2    0       0
            0.088   pi/2    0       0];
% NOTE: parameters obtained from
% https://frankaemika.github.io/docs/control_parameters.html#denavithartenberg-parameters
jTypes = ["revolute" "revolute" "revolute" "revolute" "revolute" "revolute" "revolute"];

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