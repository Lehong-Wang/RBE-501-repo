function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates an approximate inertial model of the ABB
% IRB 910SC- 3/0.55 robot.
%
% Inputs: None
%
% Output: Mlist - 4x4x5 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x4 matrix containing the spatial inertia matrices of each link

%% Link poses when the robot is in the home configuration
M01 = eye(4);
M01(1,4) = robot.a(1);

M12 = eye(4);
M12(2,2) = -1;
M12(3,3) = -1;
M12(1,4) = robot.a(2);

M23 = eye(4);
M34 = eye(4);

Mlist = cat(3, M01, M12, M23, M34);

%% Spatial Inertia Matrices
m1 = 3;  % [kg] Mass of Link 1
m2 = 2;  % [kg] Mass of Link 2
m3 = 1; % [kg] Mass of Link 3

G1 = zeros(6);
G1(4:6,4:6) = m1 * eye(3);

G2 = zeros(6);
G2(4:6,4:6) = m2 * eye(3);

G3 = zeros(6);
G3(4:6,4:6) = m3 * eye(3);


Glist = cat(3, G1, G2, G3);

end
