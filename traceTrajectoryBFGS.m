function [outTrajectory, articulations] = traceTrajectoryBFGS(robot, tcpName, inTrajectory, maxIterations, minDistance, weights, initialGuess, diagnostic)
    % set a flag prompting the export of diagnostic information upon
    % exiting the function
    diagnosticMode = 0;
    if exist("diagnostic", "var")
        diagnosticMode = logical(diagnostic);
    end
    % set the weighting vector if not provided (defaults to all weights in full)
    if ~exist('weights', 'var')
        weights = [1 1 1 1 1 1];
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
    % weighting matrix
    W = diag([weights(4:6), weights(1:3)]);
    % initialize the output trajectory storage
    outTrajectory = zeros([4 4 numWaypoints]);
    articulations = zeros(numJoints, numWaypoints);
    iterationsPerWaypoint = zeros([1 numWaypoints], "int32");
    
    for idxWaypoint = 1:numWaypoints
    
        % --- BFGS-IK ---
        % setup the IK solver for BFGS Gradient Projection
        % for "inverseKinematics" object, see: https://de.mathworks.com/help/robotics/ref/inversekinematics-system-object.html
        % for function parameters, see: https://de.mathworks.com/help/robotics/ug/inverse-kinematics-algorithms.html#bve7ae8

        solverParameters = struct( ...
            "MaxIterations", maxIterations, ...
            "SolutionTolerance", minDistance, ...
            "AllowRandomRestarts", true);
        bfgs_ik = inverseKinematics("RigidBodyTree", robot, ...
                                    "SolverAlgorithm", "BFGSGradientProjection", ...
                                    "SolverParameters", solverParameters);
                                    
        % desired transformation in space frame
        T_sd = inTrajectory(:, :, idxWaypoint);
        [articulation, solutionInfo] = bfgs_ik(tcpName, T_sd, weights, articulation);
        numIterations = solutionInfo.Iterations;
        % endeffector transformation in space frame
        T_sb = getTransform(robot, articulation, tcpName);
        currDistance = norm(W * adjointSE3(T_sb) * errorTwist(T_sb, T_sd));
        % append current pose to output trajectory
        outTrajectory(:, :, idxWaypoint) = T_sb;
        % append articulation vector to output
        articulations(:, idxWaypoint) = articulation;
        if diagnosticMode
            iterationsPerWaypoint(idxWaypoint) = numIterations;
        end
    end
    if diagnosticMode
        % store the number of solver iterations that were required to
        % approach a waypoint in the base workspace
        assignin("base", "iterationsPerWaypointBFGS", iterationsPerWaypoint);
        % store the final error in the base workspace
        assignin("base", "finalErrorBFGS", currDistance);
    end
end


%% aditional function definitions
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
