% trace a trajectory with the endeffector, outputs the computed trajectory
% "robot" - a RigidBodyTree object representing the serial open chain
% "tcpName" - the name of the endeffector body used to compute transforms
% "inTrajectory" - a (4, 4, numWaypoints) array of homogeneous transforms
% "maxIterations" - the max. number of optimization iterations per waypoint
% "minDistance" - the acceptance threshold for optimized pose error
function outTrajectory = traceTrajectory(robot, tcpName, inTrajectory, maxIterations, minDistance)
    % NOTE: assume the shape of the trajectory to be (4, 4, numWaypoints)
    [~, ~, numWaypoints] = size(inTrajectory);
    % set initial articulation to be the home joint state of the robot
    articulation = homeConfiguration(robot);
    % initialize the output trajectory storage
    outTrajectory = zeros([4 4 numWaypoints]);
    for idxWaypoint = 1:numWaypoints
       % load the current target waypoint
       T_sd = inTrajectory(:, :, idxWaypoint); 
       % initialize the system state
       T_sb = getTransform(robot, articulation, tcpName);
       % initialize the cost term
       currdistance = norm(adjointSE3(T_sb) * errorTwist(T_sb, T_sd));
       % initialize the iteration counter
       numIterations = 0;
       while currdistance > minDistance || numIterations < maxIterations
            % compute iterative change in articulation
            deltaArticulation = iterateIK(robot, articulation, tcpName, T_sd);
            % update the joint state by adding the change
            articulation = articulation + deltaArticulation;
            % update the system state
            T_sb = getTransform(robot, articulation, tcpName);
            % add the new pose to the output trajectory
            outTrajectory(:, :, idxWaypoint) = T_sb;
            % update the cost term
            currdistance = norm(adjointSE3(T_sb) * errorTwist(T_sb, T_sd));
            % increase iteration counter
            numIterations = numIterations + 1;
       end
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
function deltaArticulation = iterateIK(robot, articulation, tcpName, targetPose)
    l = 0.125;
    tcpPose = getTransform(robot, articulation, tcpName);
    localError = errorTwist(tcpPose, targetPose);
    globalError = adjointSE3(tcpPose) * localError;
    J = spaceJacobian(robot, articulation);
    deltaArticulation = J.' / (J * J.' + l^2 * eye(6)) * globalError;
end