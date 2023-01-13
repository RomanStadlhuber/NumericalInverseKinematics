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

l1 = 1.0;
l2 = 1.0;
dhparams = [l1   0   0  0;
            l2   0   0  0];


robot = rigidBodyTree;

body1 = rigidBody("body1");
joint1 = rigidBodyJoint("R1", "revolute");
setFixedTransform(joint1, dhparams(1,:), "dh");
body1.Joint = joint1;
addBody(robot, body1, "base");


body2 = rigidBody("body2");
joint2 = rigidBodyJoint("R2", "revolute");
setFixedTransform(joint2, dhparams(2, :), "dh");
body2.Joint = joint2;
addBody(robot, body2, "body1");

robot.DataFormat = "column";

%% inverse kinematics setup

targetArticulation = [pi/6; pi/2];
T_sd = getTransform(robot, targetArticulation, "body2"); % T(30°, 60°)
initialGuess = [0; pi/3];
T_sb0 = getTransform(robot, initialGuess, "body2");


fprintf('Desired artiuclation :[');
fprintf('%g, ', targetArticulation(1:end-1));
fprintf('%g]\n', targetArticulation(end));

% NOTE: Space Jacobian should look like this in its general form:

% [   0   l1 * sin(theta1);
%     0   l1 * -cos(theta1);
%     0   0
%     0   0
%     0   0
%     1   1];

%% numerical inverse kinematics

minDistance = 10e-7; % acceptance threshold for valid solution
Ad_T_sb0 = adjointSE3(T_sb0); % adjoint matrix of the initial guess
initialError = errorTwist(T_sb0, T_sd);
currDistance = norm(Ad_T_sb0 * initialError);
numIterations = 0; % increment counter to track no. of required iterations
articulation = initialGuess;

disp("Solution acceptance threshold is " + minDistance);

while currDistance > minDistance
    deltaArticulation = iterateIK(robot, articulation, "body2", T_sd);
    articulation = articulation + deltaArticulation;
    tcpPose = getTransform(robot, articulation, "body2");
    currDistance = norm(adjointSE3(tcpPose) * errorTwist(tcpPose, T_sd));
    numIterations = numIterations + 1;
end

disp("Converged after " + numIterations + " iterations");

fprintf('Converged towards artiuclation :[');
fprintf('%g, ', targetArticulation(1:end-1));
fprintf('%g]\n', targetArticulation(end));


%% function definitions

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
