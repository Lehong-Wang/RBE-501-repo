function [Mlist,Glist] = make_dynamics_model(robot)
% MAKE_KINEMATICS_MODEL Creates the dynamics model of the robot
%
% Inputs: None
%
% Output: Mlist - 4x4x7 matrix containing all the transformation matrices between consecutive link frames
%         Glist - 6x6x6 matrix containing the spatial inertia matrices of each link

    %% Link poses when the robot is in the home configuration
    [M01, M12, M23, M34, M45, M56, M67] = calculatelinkframes(robot);
    Mlist = cat(3, M01, M12, M23, M34, M45, M56, M67);

    %% Spatial Inertia Matrices
    % *** Define the link inertial properties ***
    o3 = zeros(3);
    I1 = [0.010267495893 0 0; 0 0.010267495893 0; 0 0 0.00666];
    m1 = 3.7 * eye(3);
    G1 = [I1 o3; o3 m1];
    I2 = [0.22689067591 0 0; 0 0.22689067591 0; 0 0 0.0151074];
    m2 = 8.393 * eye(3);
    G2 = [I2 o3; o3 m2];
    I3 = [0.049443313556 0 0; 0 0.049443313556 0; 0 0 0.004095];
    m3 = 2.275 * eye(3);
    G3 = [I3 o3; o3 m3];
    I4 = [0.111172755531 0 0; 0 0.111172755531 0; 0 0 0.21942];
    m4 = 1.219 * eye(3);
    G4 = [I4 o3; o3 m4];
    I5 = [0.111172755531 0 0; 0 0.111172755531 0; 0 0 0.21942];
    m5 = 1.219 * eye(3);
    G5 = [I5 o3; o3 m5];
    I6 = [0.0171364731454 0 0; 0 0.0171364731454 0; 0 0 0.033822];
    m6 = 0.1879 * eye(3);
    G6 = [I6 o3; o3 m6];
    Glist = cat(3, G1, G2, G3, G4, G5, G6);

end