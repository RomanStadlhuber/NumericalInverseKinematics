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

%% create trajectory

% create a single target waypoint (as in the book example)
targetArticulation = [pi/6; pi/2];
T_sd = getTransform(robot, targetArticulation, "body2"); % T(30°, 60°)
targetWaypoints = T_sd; % just a single target waypoint

%% run IK

outTrajectory = traceTrajectory(robot, "body2", targetWaypoints, 10, 1e-5);
[~, ~, numWaypoints] = size(outTrajectory);
disp("Generated " + numWaypoints + " waypoints");

