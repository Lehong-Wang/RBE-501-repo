function [S,M] = make_kinematics_model()
% MAKE_KINEMATICS_MODEL Calculates the Screw Axes and Home Configuration of
% the IRB 910SC- 3/0.55 SCARA robot.
%
% Inputs: robot - the robot object created by the robotics toolbox
%
% Output: S - 6xn matrix whose columns are the screw axes of the robot
%         M - homogeneous transformation representing the home configuration

% Screw Axes
L1 = 300e-3;
L2 = 250e-3;

S = [0 0 1 0 0 0;
     0 0 1 0 -L1 0;
     0 0 0 0 0 -1]';

% Home Configuration
R = [1 0 0; 0 -1 0; 0 0 -1];
p = [L1+L2 0 0]';
M = [R p; zeros(1,3) 1];

end