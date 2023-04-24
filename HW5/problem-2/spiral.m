% RBE 501 - Robot Dynamics - Spring 2023
% Homework 5, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 04/07/2023
clear, clc, close all
addpath('../lib');

plotOn = false;

% Create the environment
g = [0 0 -9.81]; % Gravity Vector [m/s^2]

% Create the robot and display it in the home configuration
robot = make_robot();
robot.plot(zeros(1,6));

% Create a kinematic model of the robot
[S,M] = make_kinematics_model(robot);
n = size(S,2); % read the number of joints

% Create a dynamical model of the robot
[Mlist,Glist] = make_dynamics_model(robot);


%% Control the motion of the robot between 2 set points
fprintf('----------------------Dynamic Control of a 3-DoF Arm--------------------\n');

% nPts = 100;
nPts = 20;
fprintf('Generating task space path... ');
phi = linspace(0, 4*pi, nPts);
% phi = linspace(0, .5*pi, nPts);
r = linspace(0, 0.3, nPts) ;
x = r .* cos(phi) + 0.4;
y = r  .* sin(phi);
z = 0.2 * ones(1,nPts);
path = [x; y; z];
fprintf('Done.\n');

fprintf('Calculating the Inverse Kinematics... ');
robot.plot(zeros(1,6)); hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
title('Inverse Dynamics Control');

%% YOUR CODE HERE











% Calculate the inverse kinematics
waypoints = zeros(n,nPts);
% waypoints = ...

currentXYZ = M(1:3,4);
currentQ = zeros(1,6);

for ii = 1 : nPts
    % fprintf(repmat('\b',1,nbytes));
    % nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
        
    while norm(path(:,ii) - currentXYZ) > 1e-3
        Ja = jacoba(S,M,currentQ);  

        % deltaQ = pinv(Ja) * (path(:,ii) - currentXYZ);
        deltaQ = Ja' * (path(:,ii) - currentXYZ);

        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentXYZ = T(1:3,4);
    end
    % for i = 1:length(currentQ)
    %     currentQ(i) = mod((currentQ(i) + pi), 2*pi) - pi;
    % end
    waypoints(:,ii) = currentQ;

end
waypoints

fprintf('Done.\n');

% Now, for each pair of consecutive waypoints, we will first calculate a
% trajectory between these two points, and then calculate the torque
% profile necessary to move from one point to the next.
fprintf('Generating the Trajectory and Torque Profiles... ');
nbytes = fprintf('0%%');

% Inititalize the variables where we will store the torque profiles, joint
% positions, and time, so that we can display them later
tau_acc = [];
jointPos_acc = [];
t_acc = [];

for jj = 1 : nPts - 1
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%3.0f%%', 100*(jj/(nPts - 1)));
   
    % Initialize the time vector
    dt = 1e-3;       % time step [s]
    % dt = 1e-2;       % time step [s]
    % t  = 0 : dt : 5; % total time [s]
    t  = 0 : dt : 1; % total time [s]

    % Initialize the arrays where we will accumulate the output of the robot
    % dynamics
    jointPos_prescribed = zeros(n,size(t,2)); % Joint Variables (Prescribed)
    jointVel_prescribed = zeros(n,size(t,2)); % Joint Velocities (Prescribed)
    jointAcc_prescribed = zeros(n,size(t,2)); % Joint Accelerations (Prescribed)
    tau_prescribed      = zeros(n,size(t,2)); % Joint Torques

    jointPos_actual = zeros(n,size(t,2)); % Joint Variables (Actual)
    jointVel_actual = zeros(n,size(t,2)); % Joint Velocities (Actual)

    % For each joint
    for ii = 1 : n
        % Calculate a trajectory using a quintic polynomial
        params_traj.t = [0 t(end)]; % start and end time of each movement step
        params_traj.time_step = dt;
        params_traj.q = [waypoints(ii,jj) waypoints(ii,jj+1)];
        params_traj.v = [0 0];
        params_traj.a = [0 0];
        params_traj.dt = dt;

        traj = make_trajectory('quintic', params_traj);

        % Generate the joint profiles (position, velocity, and
        % acceleration)
        jointPos_prescribed(ii,:) = traj.q;
        jointVel_prescribed(ii,:) = traj.v;
        jointAcc_prescribed(ii,:) = traj.a;
    end

    % Initialize the parameters for both inverse and forward dynamics
    params_rne.g = g; % gravity
    params_rne.S = S; % screw axes
    params_rne.M = Mlist; % link frames
    params_rne.G = Glist; % inertial properties
    params_fdyn.g = g; % gravity
    params_fdyn.S = S; % screw axes
    params_fdyn.M = Mlist; % link frames
    params_fdyn.G = Glist; % inertial properties


    % Initialize the (actual) joint variables
    jointPos_actual(:,1) = jointPos_prescribed(:,1);
    jointVel_actual(:,1) = jointVel_actual(:,1);

    for ii = 1 : size(t,2) - 1
        % Calculate the joint torques using the RNE algorithm
        params_rne.jointPos = jointPos_prescribed(:,ii);
        params_rne.jointVel = jointVel_prescribed(:,ii);
        params_rne.jointAcc = jointAcc_prescribed(:,ii);
        params_rne.Ftip = zeros(6,1); % end effector wrench
        params_rne.Ftip = [0 0 0 -9.8 0 0]'; % end effector wrench

        tau_prescribed(:,ii) = rne(params_rne);

        % Feed the torques to the forward dynamics model and perform one
        % simulation step
        params_fdyn.jointPos = jointPos_actual(:,ii);
        params_fdyn.jointVel = jointVel_actual(:,ii);
        params_fdyn.tau = tau_prescribed(:,ii);
        params_fdyn.Ftip = zeros(6,1); % end effector wrench
        params_fdyn.Ftip = [0 0 0 -9.8 0 0]';
        jointAcc = fdyn(params_fdyn);

        % Integrate the joint accelerations to get velocity and
        % position
        jointVel_actual(:,ii+1) = dt * jointAcc + jointVel_actual(:,ii);
        jointPos_actual(:,ii+1) = dt * jointVel_actual(:,ii) + jointPos_actual(:,ii);
    end

    tau_prescribed(:,end) = tau_prescribed(:,end-1);
    
    tau_acc = [tau_acc tau_prescribed];
    jointPos_acc = [jointPos_acc jointPos_actual];
    t_acc = [t_acc t+t(end)*(jj-1)];
end

fprintf('\nDone. Simulating the robot...');

%% Animate the robot
title('Inverse Dynamics Control');
% robot.plot(jointPos_acc(:,1:100:end)','trail',{'r', 'LineWidth', 2}, 'movie', 'RBE-501-2023-HW5-point2point.mp4');
robot.plot(jointPos_acc(:,1:50:end)','trail',{'r', 'LineWidth', 2}, 'movie', 'RBE-501-2023-HW5-point2point.mp4');
fprintf('Done.\n');


%% Display the Joint Torques
figure, hold on, grid on
plot(t_acc, tau_acc(1,:), 'Linewidth', 2);
plot(t_acc, tau_acc(2,:), 'Linewidth', 2);
plot(t_acc, tau_acc(3,:), 'Linewidth', 2);
plot(t_acc, tau_acc(4,:), 'Linewidth', 2);
plot(t_acc, tau_acc(5,:), 'Linewidth', 2);
plot(t_acc, tau_acc(6,:), 'Linewidth', 2);
title('Torque Profiles');
xlabel('Time [s]'), ylabel('Torque [Nm]');
legend({'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6'});
set(gca, 'FontSize', 14);

fprintf('Program completed successfully.\n');