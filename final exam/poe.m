% RBE 501 - Robot Dynamics - Spring 2023
% Midterm Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/21/2023
clear, clc, close all
addpath('utils');

n = 4;   % degrees of freedom

%% Create the manipulator and display it in its home configuration
[robot, jointLimits] = make_robot();

q = zeros(1,n);
robot.teach(zeros(1,n));

%% Question 1 - Calculate the screw axes
S = [0 0 1 0 0 0;
     0 0 1 0 -0.3 0;
     0 0 0 0 0 -1;
     0 0 -1 -cross([0 0 -1], [0.55 0 0])]';

%% Question 2 - Calculate the home configuration
M = [1 0 0 0.55;
     0 -1 0 0;
     0 0 -1 0;
     0 0 0 1];

q = [1 1 2 1];
robot.teach(q)
T = fkine(S, M, q)