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

%% run IK
[outTrajectory, outJointStates] = traceTrajectory(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess);

%% plot trajectory
viz(robot, outTrajectory, targetPositions, outJointStates);