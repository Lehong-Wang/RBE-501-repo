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


%% Forward Dynamics -- robot falls under gravity
fprintf('---------Simulation of the Robot Falling Under Gravity--------\n');
fprintf('Calculating the Forward Dynamics: ');
nbytes = fprintf('0%%');

% Initialize the parameters for the simulation
dt = 1e-4;        % simulation time step [s]
t = 0 : dt : 1; % total simulation time [s]
tau = zeros(n,size(t,2));  % joint torques
Ftip = zeros(6,size(t,2)); % end effector wrench

params.g = g; % gravity
params.S = S; % screw axes
params.M = Mlist; % link frames
params.G = Glist; % inertial properties
params.jointPos = zeros(3,1);    % initial configuration
params.jointVel = zeros(3,1);   % initial velocities

qt = zeros(n,size(t,2));
qdt = zeros(n,size(t,2));

for ii = 1 : size(t,2) - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(ii/(size(t,2)-1)));

    % Set torques and forces
    params.tau = tau(:,ii);
    params.Ftip = Ftip(:,ii);

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

fprintf('\nDone. Simulating the robot.\n');
title('Robot Falling Under Gravity');
robot.plot(qt(:,1:100:end)');

input('Simulation complete. Press Enter to continue.');