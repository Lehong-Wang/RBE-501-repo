function [Mlist,Glist] = make_dynamics_model()
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link


%% Create the manipulator
L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.3; % Lenght of Link 2 [m]
L3 = 0.15; % Lenght of Link 3 [m]
w  = 0.04; % Link Width [m]
l  = 0.04; % Link Depth [m]

% Link poses when the robot is in the home configuration
R1 = [1 0 0; 0 1 0; 0 0 1]; P1 = [0 0 L1/2]';
M01 = [R1 P1; 0 0 0 1];
R2 = [1 0 0; 0 0 1; 0 -1 0]; P2 = [0 L2/2 L1/2]';
M12 = [R2 P2; 0 0 0 1];
R3 = [1 0 0; 0 0 1; 0 -1 0]; P3 = [0 L3/2 L2/2]';
M23 = [R3 P3; 0 0 0 1];
R4 = [0 1 0; 0 0 1; 1 0 0];     P4 = [0 0 L3/2]';
M34 = [R4 P4; 0 0 0 1];

Mlist = cat(3, M01, M12, M23, M34);

%% Spatial Inertia Matrices
% *** Define the link inertial properties ***
m1 = 5;   % Mass of Link 1 [kg]
m2 = 1;   % Mass of Link 2 [kg]
m3 = 1;   % Mass of Link 3 [kg]

% m1 = 0;
% m2 = 0;
% m3 = 0;

% Spatial Inertia Matrices
o3 = zeros(3);
I1 = o3; I2 = o3; I3 = o3;
% I1 = m1 / 12 * [(w^2 + L1^2) 0 0; 0 (l^2 + L1^2) 0; 0 0 (w^2 + l^2)];
% I2 = m2 / 12 * [(w^2 + L2^2) 0 0; 0 (l^2 + L2^2) 0; 0 0 (w^2 + l^2)];
% I3 = m3 / 12 * [(w^2 + L3^2) 0 0; 0 (l^2 + L3^2) 0; 0 0 (w^2 + l^2)];
G1 = [I1 o3; o3 m1*eye(3)];
G2 = [I2 o3; o3 m2*eye(3)];
G3 = [I3 o3; o3 m3*eye(3)];

G1
Glist = cat(3, G1, G2, G3);

end