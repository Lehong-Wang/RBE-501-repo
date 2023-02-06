% RBE 501 - Robot Dynamics - Spring 2023
% Homework 2, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 01/26/2023

clear, clc, close all

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
robot.teach(q);


%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [0 0 1 0 0 0;
     1 0 0 0 L1 0;
     1 0 0 0 L1 -L2]';

%% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration

M = [0 1 0 0;
     0 0 -1 L2;
     -1 0 0 0;
     0 0 0 1];

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
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
    
    % Calculate the forward kinematics
    T = eye(4);
    for i = 1: width(S)
        Si = S(:,i);
        qi = q(i);
        TF = twist2ht(Si, qi);
        T = T * TF;
    end
    T = T * M;
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');





function R = axisangle2rot(omega,theta)
    % Rodrigoes Formula
    I = eye(3);
    st = sin(theta);
    ct = cos(theta);
    w = [0          -omega(3)   omega(2);
         omega(3)   0           -omega(1);
         -omega(2)  omega(1)    0;];

    R = I + st * w + (1-ct) * w * w;
end



function T = twist2ht(S,theta)

    I = eye(3);
    st = sin(theta);
    ct = cos(theta);
    omega = [S(1); S(2); S(3)];
    v = [S(4); S(5); S(6)];
    w = [0          -omega(3)   omega(2);
         omega(3)   0           -omega(1);
         -omega(2)  omega(1)    0;];
    R = axisangle2rot(omega,theta);
    coner = (I*theta + (1-ct)*w + (theta-st)*w*w) * v;
    
    T = [R coner;
        0 0 0 1];
end




