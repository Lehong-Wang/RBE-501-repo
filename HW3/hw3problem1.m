% RBE 501 - Robot Dynamics - Spring 2023
% Homework 3, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/17/2023
clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 20; % number of random test configurations

%% Create the manipulator
% Link length values (meters)
L1 = 0.3;
L2 = 0.3;
L3 = 0.3;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', pi/2, 'offset', pi/2), ...
                    Revolute('a', L2, 'd', 0, 'alpha', 0), ...
                    Revolute('a', L3, 'd', 0, 'alpha', pi/2, 'offset', -pi/2)], ...
                    'name', 'RRR Manipulator');

% Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        -pi/12 pi/3]; % q(3)

% Display the manipulator in the home configuration
q = zeros(1,3);
robot.plot(q);


%% Calculate the screw axes and home configuration
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0 0 1 0 0 0;
     1 0 0 -cross([1 0 0], [0 0 L1]);
     1 0 0 -cross([1 0 0], [0 L2 L1])]';

R_home = [0 0 -1; 1 0 0; 0 -1 0]';
t_home = [0 L2 L1-L3]';
M = [R_home t_home; 0 0 0 1];


%% Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,3);

% Generate the target configurations     
q = [linspace(0,pi/2,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests)];

% Initialize the vector of target twists
targetTwist = zeros(6,nTests);
targetPose  = zeros(4,4,nTests); % save the same information as HT matrices

% For each of the target configurations, calculate the forward kinematics
for ii = 1 : nTests
    targetPose(:,:,ii) = fkine(S,M,q(:,ii)');
    t = MatrixLog6(targetPose(:,:,ii));
    targetTwist(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
end

% Display the robot and the points
if plotOn
    robot.plot(currentQ); hold on;
    h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
    scatter3(reshape(targetPose(1,4,:), 1, nTests), ...
             reshape(targetPose(2,4,:), 1, nTests), ...
             reshape(targetPose(3,4,:), 1, nTests), ...
        'filled');
end

tic
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    
    if plotOn
        set(h, 'matrix', targetPose(:,:,ii));
        title('Inverse Kinematics Test');
        drawnow;
    end
    
    while norm(targetTwist(:,ii) - currentPose) > 1e-3
        J_s = jacob0(S,currentQ);
% 
%         V_d = targetTwist(:,ii)
%         current_V = fkine()
        
%         deltaQ = 0.2*transpose(J_s) * (targetTwist(:,ii) - currentPose)
        deltaQ = pinv(J_s) * (targetTwist(:,ii) - currentPose);
        
        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
                   
        %robot.maniplty(currentQ);
        
        if plotOn
            try
                robot.plot(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end

toc

fprintf('\nTest passed successfully.\n');
