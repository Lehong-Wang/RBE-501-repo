% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 3
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 01/03/2023

clear, clc, close all

plotOn = true;
nTests = 20; % number of random test configurations

%% Create the manipulator
n = 6; % degrees of freedom

% Robot dimensions (meters)
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
qlim = qlim * pi/180;
% Display the manipulator in the home configuration
q = zeros(1,n);
robot.teach(q);



%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0 0 1 0 0 0;
     0 1 0 -0.32 0 0;
     0 1 0 -0.545 0 0;
     0 0 1 0.035 0 0;
     0 1 0 -0.770 0 0;
     0 0 1 0.035 0 0;]';

%% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration

M = double(robot.fkine([0 0 0 0 0 0]));

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the forward kinematics for 100 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(), ...
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(), ...
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(), ...
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
%     T = eye(4);
%     for i = 1: width(S)
%         Si = S(:,i);
%         qi = q(i);
%         TF = twist2ht(Si, qi);
%         T = T * TF;
% 
%         disp(i)
%         disp(TF)
%         disp(T)
%     end
%     T = T * M;
    T = fkine(S,M,q);
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end

    disp(double(robot.fkine([0 0 0 0 0 0])));
    disp(M)
    disp(double(robot.fkine(q)));
    disp(T);
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');











