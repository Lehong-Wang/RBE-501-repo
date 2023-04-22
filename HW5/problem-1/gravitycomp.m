% RBE 501 - Robot Dynamics - Spring 2023
% Homework 5, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 04/04/2023
clear, clc, close all
addpath('../lib');

plotOn = false;

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot and display it in the home configuration
robot = make_robot();
robot.plot(zeros(1,3));

% Create a kinematic model of the robot
[S,M] = make_kinematics_model();
n = size(S,2); % read the number of joints

% Create a dynamical model of the robot
[Mlist,Glist] = make_dynamics_model();


%% Gravity Compensation
fprintf('-----------------------Gravity Compensation-----------------------\n');
% We are now going to solve the inverse dynamics and calculate the torques
% required to keep the robot where it is.

% Initialize the parameters for the RNE algorithm
clear params

params.g = g; % gravity vector
params.S = S; % screw axes
params.M = Mlist; % link frames 
params.G = Glist; % inertial properties
params.jointPos = zeros(3,1); % desired joint positions
params.jointVel = zeros(3,1); % desired joint velocities
params.jointAcc = zeros(3,1); % desired joint accelerations
params.Ftip = zeros(6,1);     % desired wrench at the end effector

% Invoke the RNE algorithm to calculate the joint torques needed for
% gravity compensation
tau = rne(params);

fprintf('Joint Torques: ');
fprintf('[%f %f %f] Nm\n', tau(1), tau(2), tau(3));

% To make sure that the solution is correct, let us simulate the robot
fprintf('\nWe are now going to simulate the robot to see if it moves.\n');
fprintf('Calculating the Forward Dynamics: ');
nbytes = fprintf('0%%');

dt = 1e-4;        % simulation time step [s]
t = 0 : dt : 0.5; % total simulation time [s]

qt = zeros(n,size(t,2));  qt(:,1) = params.jointPos;
qdt = zeros(n,size(t,2)); qdt(:,1) = params.jointVel;

% Display the robot
robot.plot(params.jointPos');

for ii = 1 : size(t,2) - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(ii/(size(t,2)-1)));

    % Set torques
    params.tau = tau; % supply the torques needed for gravity compensation

    % Calculate the joint accelerations
    jointAcc = fdyn(params);

    % Integrate the joint accelerations to get velocity and
    % position
    params.jointPos = params.jointPos + dt * params.jointVel;
    params.jointVel = params.jointVel + dt * jointAcc;

    % Accumulate results
    qt(:,ii+1) = params.jointPos;
    qdt(:,ii+1) = params.jointVel;
end

fprintf('\nDone. Simulating the robot...\n');
title('Gravity Compensation');
robot.plot(qt(:,1:100:end)');

input('Simulation complete. Press Enter to continue.');