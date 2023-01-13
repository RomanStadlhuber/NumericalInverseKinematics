% trace a trajectory with the endeffector, outputs the computed trajectory
% --- parameters ---
% "robot" - a RigidBodyTree object representing the serial open chain
% "tcpName" - the name of the endeffector body used to compute transforms
% "inTrajectory" - a (4, 4, numWaypoints) array of homogeneous transforms
% "maxIterations" - the max. number of optimization iterations per waypoint
% "minDistance" - the acceptance threshold for optimized pose error
% "weights" - (optional) a row-vector of orientation and position weights
% in the order: [rx ry rz x y z]
% --- returns ---
% a tuple of the form [outTrajectory, articulations]
% "outTrajectory" - the endeffector poses computed by the optimizer
% "articulations" - the articulations used to generate outTrajectory
function [outTrajectory, articulations] = traceTrajectory(robot, tcpName, inTrajectory, maxIterations, minDistance, weights, initialGuess, numOffsetJoints)
    % the links that need to be offset before computing the jacobian
    jointOffset = 0;
    if exist('numOffsetJoints', 'var')
        jointOffset = numOffsetJoints;
    end
    % a diagonal weighting factor matrix defaulting to the identity
    W = eye(6);
    % set an optional weighting matrix
    if exist('weights', 'var')
        W = diag([weights(4:6), weights(1:3)]);
    end
    % NOTE: assume the shape of the trajectory to be (4, 4, numWaypoints)
    [~, ~, numWaypoints] = size(inTrajectory);
    % number of joints (required to fill the articulation space)
    numJoints = size(homeConfiguration(robot), 1);
    % set initial articulation to be the home joint state of the robot
    articulation = homeConfiguration(robot);
    % set articulation to initial guess if provided
    if exist('initialGuess', 'var')
        articulation = initialGuess;
    end
    % initialize the output trajectory storage
    outTrajectory = zeros([4 4 numWaypoints]);
    articulations = zeros(numJoints, numWaypoints);
    for idxWaypoint = 1:numWaypoints
       % load the current target waypoint
       T_sd = inTrajectory(:, :, idxWaypoint); 
       % initialize the system state
       T_sb = getTransform(robot, articulation, tcpName);
       % initialize the cost term
       currdistance = norm(W * adjointSE3(T_sb) * errorTwist(T_sb, T_sd));
       % initialize the iteration counter
       numIterations = 0;
       while currdistance > minDistance && numIterations < maxIterations
            % compute iterative change in articulation
            deltaArticulation = iterateIK(robot, articulation, tcpName, T_sd, jointOffset);
            % update the joint state by adding the change
            articulation = articulation + deltaArticulation;
            % update the system state
            T_sb = getTransform(robot, articulation, tcpName);
            % update the cost term
            currdistance = norm(W * adjointSE3(T_sb) * errorTwist(T_sb, T_sd));
            % increase iteration counter
            numIterations = numIterations + 1;
       end
        % add the new pose to the output trajectory
        outTrajectory(:, :, idxWaypoint) = T_sb;
        % add the generated articulation to the output
        articulations(:, idxWaypoint) = articulation;
    end
end

%% additional function definitions

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

% iteration step of damped least squares IK algorithm
function deltaArticulation = iterateIK(robot, articulation, tcpName, targetPose, linkOffset)
    l = 0.125;
    tcpPose = getTransform(robot, articulation, tcpName);
    localError = errorTwist(tcpPose, targetPose);
    globalError = adjointSE3(tcpPose) * localError;
    J = spaceJacobian(robot, articulation, linkOffset);
    deltaArticulation = J.' / (J * J.' + l^2 * eye(6)) * globalError;
end