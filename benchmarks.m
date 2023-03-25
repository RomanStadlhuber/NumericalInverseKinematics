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
diagnosticMode = true;
%% compute inverse kinematics for entire trajectory

% GN
[outTrajectoryGN, outJointStatesGN] = traceTrajectoryGN(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess, diagnosticMode);
% LM
[outTrajectoryLM, outJointStatesLM] = traceTrajectory(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess, diagnosticMode);
% BGFS
[outTrajectoryBFGS, outJointStatesBFGS] = traceTrajectoryBFGS(robot, tcpName, waypoints, maxIterations, minDistance, weights, initialGuess, diagnosticMode);

%% print diagnostics
disp("min. iterations");
disp("GN: " + min(iterationsPerWaypointGN) + "  LM: " + min(iterationsPerWaypoint) + "  BFGS: " + min(iterationsPerWaypointBFGS));
disp("avg. iterations");
disp("GN: " + mean(iterationsPerWaypointGN) + "  LM: " + mean(iterationsPerWaypoint) + "  BFGS: " + mean(iterationsPerWaypointBFGS));
disp("max. iterations");
disp("GN: " + max(iterationsPerWaypointGN) + "  LM: " + max(iterationsPerWaypoint) + "  BFGS: " + max(iterationsPerWaypointBFGS));

[minErrorGN, avgErrorGN, maxErrorGN] = evaluateTrajectory(waypoints, outTrajectoryGN, weights);
[minErrorLM, avgErrorLM, maxErrorLM] = evaluateTrajectory(waypoints, outTrajectoryLM, weights);
[minErrorBFGS, avgErrorBFGS, maxErrorBFGS] = evaluateTrajectory(waypoints, outTrajectoryBFGS, weights);


%% plot trajectory
viz(robot, outTrajectoryLM, targetPositions, outJointStatesLM);

%% additional function definitions

% evaluate the an output trajectory w.r.t. a target trajectory
function [minError, avgError, maxError] = evaluateTrajectory(targetTrajectory, outputTrajectory, weights)

    % a diagonal weighting factor matrix defaulting to the identity
    W = eye(6);
    % set an optional weighting matrix
    if exist('weights', 'var')
        W = diag([weights(4:6), weights(1:3)]);
    end
    [~, ~, numWaypoints] = size(targetTrajectory);
    localErrors = zeros([1 numWaypoints]);
    for idxWaypoint = 1:numWaypoints
        T_sd = targetTrajectory(:, :, idxWaypoint);
        T_sb = outputTrajectory(:, :, idxWaypoint);
        error = W * errorTwist(T_sb, T_sd);
        localErrors(idxWaypoint) = norm(error);
    end

    minError = min(localErrors);
    avgError = mean(localErrors);
    maxError = max(localErrors);
end

% compute error twist in body (=TCP frame)
function err = errorTwist(Ta, Tb)
  % pose delta as seen from tangent at point a
  deltaA =  logm(Ta \ Tb);
  err_tvec = deltaA(1:3, 4); % tangent translation vector
  w_x = deltaA(1:3, 1:3); % tangent rotation vector
  err_rvec = [w_x(3,2); w_x(1,3); w_x(2,1)];
  % weighted error vector in twist coordinates at the local
  % tangent-space
  err = [err_tvec; err_rvec];
end