%% RBE/ME 501 - Robot Dynamics - Spring 2023
%  Mini-Project #1
%  Instructor: L. Fichera, loris@wpi.edu
close all; clear, clc
addpath('utils');


%% Initialize the model of the robot
% Screw Axes:
S = [0 0 1 0 0 0;
    0 1 0 -0.352 0 0.07;
    0 1 0 -0.352 0 0.43;
    0 0 1 0 -0.43 0;
    0 1 0 -0.732 0 0.43;
    0 0 1 0 -0.43 0]';

% Home configuration:
R = [1 0 0; 0 0 -1; 0 1 0];
p = [0.43 0 0.797]';
M = [R p; 0 0 0 1];

%% Load the test configurations
load target_poses.mat
nPts = length(V);

%% Calculate the IK
% Initialize a matrix to store the IK solutions
q = zeros(nPts,6);
success_num = 0;

for ii = 1 : nPts
    q0 = zeros(1,6);
    joint_angle = ikin(S,M,q0,V(:,ii));
    
    q(ii,:) = ikin(S,M,q0,V(:,ii));


    T = fkine(S, M, q(ii,:));
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';

    if norm(V(:,ii) - currentPose) < 1e-10
        success_num = success_num + 1;
    end
end

success_num
q
