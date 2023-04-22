function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% a robot.
%
% Inputs: None
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

L1 = 0.3; % Lenght of Link 1 [m]
L2 = 0.3; % Lenght of Link 2 [m]
L3 = 0.15; % Lenght of Link 3 [m]

% Screw Axes
S = [0 0 1 0 0 0;
     1 0 0 -cross([1 0 0], [0 0 L1]);
     1 0 0 -cross([1 0 0], [0 L2 L1])]';

% Home configuration
R_home = [0 0 -1; 1 0 0; 0 -1 0]';
t_home = [0 L2 L1-L3]';
M = [R_home t_home; 0 0 0 1];

end