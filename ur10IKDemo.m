clc
clear
close all;

%% import the robot

% import robot from simulink
% [robot, ~] = importrobot("ur10_model");

% otherwise: import robot from URDF
robot = importrobot("ur10_files/ur10/ur10.urdf");

% set format and obtain name of endeffector frame
robot.DataFormat = "column";
tcpName = char(robot.BodyNames(robot.NumBodies));
jointIdxOffset = 1;

%% configure parameters for inverse kinematics

targetArticulation = [0; pi/2; 0; pi/2; 0; 0];
% desired endeffector pose in space frame
T_sd = getTransform(robot, targetArticulation, tcpName);

% initialGuess = monteCarloInitialGuess(robot, tcpName, T_sd); 

% this is a slightly offset articulation than that which produces the
% target pose, so it "should" converge
initialGuess = monteCarloInitialGuess(robot, tcpName, T_sd);
T_sb0 = getTransform(robot, initialGuess, tcpName);
% satisfaction constraint and breaking condition
minDistance = 1e-3; % consider the result converged if this distance is met
maxIterations = 5e3; % other wise break the loop after that many iterations

%% numerical IK
Ad_T_sb0 = adjointSE3(T_sb0);
currDistance = norm(Ad_T_sb0 * errorTwist(T_sb0, T_sd));
numIterations = 0;
articulation = initialGuess;

while currDistance > minDistance && numIterations < maxIterations
    deltaArticulation = iterateIK(robot, articulation, tcpName, T_sd);
    articulation = articulation + deltaArticulation;
    tcpPose = getTransform(robot, articulation, tcpName);
    currDistance = norm(adjointSE3(tcpPose) * errorTwist(tcpPose, T_sd));
    numIterations = numIterations + 1;
end

disp("Exiting after " + numIterations + " iterations");
disp("Final error: " ...
    + norm(adjointSE3(tcpPose) * errorTwist(tcpPose, T_sd)));


%% function definitions

% compute error twist in body (=TCP frame)
% see "Modern Robotics", Ch. 6.2.2 p. 230, where the numerical inverse
% kinematics algorithm is explained step by step
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
% for more information on "damped least squares", see
% http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf
% this particular algorithm uses Eq. (11) which is found on p. 10
function deltaArticulation = iterateIK(robot, articulation, tcpName, targetPose)
    l = 0.125;
    tcpPose = getTransform(robot, articulation, tcpName);
    localError = errorTwist(tcpPose, targetPose);
    globalError = adjointSE3(tcpPose) * localError;
    J = spaceJacobian(robot, articulation, 1);
    deltaArticulation = J.' / (J * J.' + l^2 * eye(6)) * globalError;
end