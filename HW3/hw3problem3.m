% RBE 501 - Robot Dynamics - Spring 2023
% Homework 3
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/17/2023
clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 20; % number of random test configurations
n      = 6;  % degrees of freedom

%% Create the manipulator
% Link length values (meters)
H1 = 0.320;
H2 = 0.225;
H3 = 0.225;
H4 = 0.065;
W  = 0.035;

robot = SerialLink([Revolute('d', H1, 'a', 0,  'alpha', -pi/2, 'offset', 0), ...
    Revolute('d', 0,  'a', H2, 'alpha', 0,     'offset', -pi/2), ...
    Revolute('d', W,  'a', 0,  'alpha', pi/2,  'offset', pi/2), ...
    Revolute('d', H3, 'a', 0,  'alpha', -pi/2, 'offset', 0), ...
    Revolute('d', 0,  'a', 0,  'alpha', pi/2,  'offset', 0), ...
    Revolute('d', H4, 'a', 0,  'alpha', 0,     'offset', 0)], ...
    'name', 'Staubli TX-40');

% Joint limits
qlim = [-180  180;  % q(1)
    -125  125;  % q(2)
    -138  138;  % q(3)
    -270  270;  % q(4)
    -120  133.5;% q(5)
    -270  270]; % q(6)


% Display the manipulator in the home configuration
q = zeros(1,n);
robot.teach(q);
hold on;


%% Calculate the screw axes and the home configuration
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0 0 1 0 0 0;
        0 1 0 -0.32 0 0;
        0 1 0 -0.545 0 0;
        0 0 1 0.035 0 0;
        0 1 0 -0.77 0 0;
        0 0 1 0.035 0 0]';

R = eye(3);
p = [0 0.035 0.835]';
M = [R p; 0 0 0 1];



% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,6);

% Generate the circle that the robot has to trace
t = linspace(0, 2*pi, 36);
x = 0.3  * ones(1,36);
y = 0.25 * sin(t);
z = 0.25 * cos(t) + 0.4;
path = [x; y; z];

% Display the circle in the task space
scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Desired robot orientation
for ii = 1 : length(path)
   RTarget(:,:,ii) = roty(pi/2 * ii/length(path));
end


    
for ii = 1 : length(path)
    robot.teach(currentQ);
    
    % Generate the desired pose
    T = eye(4);
    T(1:3,1:3) = M(1:3,1:3);
    T(1:3,4) = path(:,ii);
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
        
    % Inverse Kinematics
    while norm(targetPose - currentPose) > 1e-3
        J = jacob0(S,currentQ);        
        
        deltaQ = pinv(J) * (targetPose - currentPose);

        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';       
    end
end


