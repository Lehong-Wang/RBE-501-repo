% RBE 501 - Robot Dynamics - Spring 2023
% Homework 4, Problem 2
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 03/20/2023
clear, clc, close all
addpath('utils');

plotOn = true; 
nTests = 10;

%% Create the manipulator
mdl_stanford
stanf

if plotOn
   stanf.teach(zeros(1,6)); 
end

L1 = 0.412;
L2 = 0.154;

%% Part A - Forward Kinematics via PoE in the body frame
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S_space = [0 0 1 0 0 0;
           0 1 0 -cross([0 1 0], [0 0 L1]);
           0 0 0 0 0 1;
           0 0 1 -cross([0 0 1], [0 L2 0]);
           1 0 0 -cross([1 0 0], [0 L2 L1]);
           0 0 1 -cross([0 0 1], [0 L2 0])]';


% Let us calculate the homogeneous transformation matrix M for the
% home configuration
% R_home = [0 0 -1; 1 0 0; 0 -1 0]';
% t_home = [0 L2 L1-L3]';
% M = [R_home t_home; 0 0 0 1];
% M = stanf.fkine(zeros(1,6));
M =  [0  1 0 0;
      -1 0 0 0.154;
      0  0 1 0.675;
      0  0 0 1];


T_body = M;
S_body = [];
for i = 1:width(S_space)
    V_s = S_space(:,i);
    V_b = twistspace2body(V_s, T_body);
    S_body(:,i) = V_b;
end
S_space
S_body



fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Test the forward kinematics for 10 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = rand(1,6)*2*pi - pi
    q(3) = (q(3) + pi)/5;
    
    % Calculate the forward kinematics
   %  T = fkine(S_body, M, q, 'body')
    T = fkine(S_space, M, q)
    
    if plotOn
        stanf.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(stanf.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part B - Calculate the Body Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 10 random sets of joiny
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = rand(1,6)*2*pi - pi
    q(3) = (q(3) + pi)/5;
    
    % Calculate the Jacobian in the body frame
    J_b = jacobe(S_space, M, q)
    
    if plotOn
        stanf.teach(q);
        title('Differential Kinematics Test');
    end
    J_r = stanf.jacobe(q)
    % Test the correctness of the Jacobian
    J_test = [J_b(4:6,:); J_b(1:3,:)]; % swap the rotation and translation components
    assert(all(all(abs(double(stanf.jacobe(q)) - J_test) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');


%% Part C - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Set the current joint variables
currentQ = zeros(1,6);

% Calculate the Analytical Jacobian at the current configuration
J_a = jacoba(S_space,M,currentQ);

% Generate path to follow
t = linspace(0, 2*pi, nTests);
x = 0.5 * cos(t);
y = 0.5 * sin(t);
z = 0.4 * ones(1,nTests);
path = [x; y; z];

if plotOn
    stanf.teach(currentQ);
    h = plot_ellipse(J_a*J_a');
    title('Inverse Kinematics Test');
    hold on
    scatter3(path(1,:), path(2,:), path(3,:), 'filled');
end
     
% Iterate over the target points
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Select the next target point
    targetPose = path(:,ii);
    T = fkine(S_body, M, currentQ, 'body');
    currentPose = T(1:3,4);
    
    while norm(targetPose - currentPose) > 1e-7

        J_a = jacoba(S_space,M,currentQ);
        deltaQ = pinv(J_a) * (targetPose - currentPose);
        currentQ = currentQ + deltaQ';
        currentT = fkine(S_body,M,currentQ,'body');
        currentPose = currentT(1:3,4);
        % YOUR INVERSE KINEMATICS CODE HERE
        % Necessary variables:
        % Current Robot Pose -> currentPose
        % Target Robot Pose ->  targetPose
        % Current Joint Variables -> currentQ
    end
    stanf.teach(currentQ)
end

fprintf('\nIK Test passed successfully.\n');
