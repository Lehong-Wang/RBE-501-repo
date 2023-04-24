

addpath('utils')
addpath('lib')


% %% Sample Usage of the RNE function for a three-dof robot (this is the same 3-DoF robot you have seen in HW 2, 3 and 4)

% %% *** LET US FIRST DEFINE THE KINEMATICS OF THIS ROBOT ***
% n = 3; % degrees of freedom
% L1 = 0.3; % Lenght of Link 1 [m]
% L2 = 0.3; % Lenght of Link 2 [m]
% L3 = 0.15; % Lenght of Link 3 [m]

% % Screw Axes
% S = [0 0 1 0 0 0;
%      1 0 0 -cross([1 0 0], [0 0 L1]);
%      1 0 0 -cross([1 0 0], [0 L2 L1])]';

% % Home configuration matrix
% R_home = [0 0 -1; 1 0 0; 0 -1 0]';
% t_home = [0 L2 L1-L3]';
% M = [R_home t_home; 0 0 0 1];

% % Link frames in the home configuration
% M01 = [eye(3) [0 0 L1/2]'; 0 0 0 1];
% M12 = [[1 0 0; 0 0 1; 0 -1 0], [0 L2/2 L1/2]'; 0 0 0 1];
% M23 = [[1 0 0; 0 0 1; 0 -1 0], [0 L3/2 L2/2]'; 0 0 0 1];
% M34 = [[0 1 0; 0 0 1; 1 0 0], [0 0 L3/2]'; 0 0 0 1];

% M1 = M01;
% M2 = M1 * M12;
% M3 = M2 * M23;
% M4 = M3 * M34;

% M = cat(3, M01, M12, M23, M34);

% %% *** NOW LET US DEFINE THE INERTIAL PROPERTIES ***
% m1 = 5;   % Mass of Link 1 [kg]
% m2 = 1;   % Mass of Link 2 [kg]
% m3 = 1;   % Mass of Link 3 [kg]
% w  = 0.04; % Link Width [m]
% l  = 0.04; % Link Depth [m]

% % Spatial Inertia Matrices
% G1 = zeros(6,6); 
% Ixx1 = m1*(w^2+L1^2)/12;
% Iyy1 = m1*(l^2+L1^2)/12;
% Izz1 = m1*(l^2+w^2)/12;
% G1(1:3,1:3) = diag([Ixx1 Iyy1 Izz1]);
% G1(4:6,4:6) = m1 * eye(3);

% G2 = zeros(6,6);
% Ixx2 = m2*(w^2+L2^2)/12;
% Iyy2 = m2*(l^2+L2^2)/12;
% Izz2 = m2*(l^2+w^2)/12;
% G2(1:3,1:3) = diag([Ixx2 Iyy2 Izz2]);
% G2(4:6,4:6) = m2 * eye(3);

% G3 = zeros(6,6);
% Ixx3 = m2*(w^2+L3^2)/12;
% Iyy3 = m2*(l^2+L3^2)/12;
% Izz3 = m2*(l^2+w^2)/12;
% G3(1:3,1:3) = diag([Ixx3 Iyy3 Izz3]);
% G3(4:6,4:6) = m3 * eye(3);

% G = cat(3, G1, G2, G3);

% %% *** FINALLY, LET US ASSEMBLE THE STRUCTURE WE NEED TO PASS TO THE RNE FUNCTION ***
% params.g =[0 0 -9.81]; % Gravity Vector [m/s^2]
% params.S = S;       
% params.M = M;
% params.G = G;
% params.jointPos = zeros(3,1); % Current Joint Variables (arbitrarily set to zero)
% params.jointVel = zeros(3,1); % Current Joint Velocities (arbitrarily set to zero)
% params.jointAcc = zeros(3,1); % Current Joint Accelerations (arbitrarily set to zero)
% params.Ftip = zeros(6,1);     % Wrench applied at the tip

% % INVOKE THE RNE FUNCTION TO CALCULATE THE INVERSE DYNAMICS
% [tau,V,Vdot] = rne(params)






% Create a cubic polynomial trajectory
params.t = [0 5]; % seconds
params.dt = 0.01; % seconds
params.q = [0 pi/4]; % radians
params.v = [0 0]; % radians/s
params.a = [0 0];

% params.t = [0       5.407]
% params.dt = 0.042586
% params.q = [0       3.398]
% params.v = [0     0.42351]
% params.a = [0   0.29]
traj = make_trajectory('quintic', params);

% params.t = [0      2.4659]
% params.dt = 0.031653
% params.q = [0      5.0638]
% params.v = [0     0.73382]


% traj = make_trajectory('cubic', params);

% Uncomment the following code to create a quintic polynomial trajectory
%params.a = [0 0];
%traj = make_trajectory('quintic', params);

% Plot the results
figure, hold on;
plot(traj.t, traj.q, 'LineWidth', 2.5);
plot(traj.t, traj.v, 'LineWidth', 2.5);
plot(traj.t, traj.a, 'LineWidth', 2.5);

legend({'coordinate', 'velocity', 'acceleration'});

set(gca, 'FontSize', 16);

grid on;
xlabel('Time [s]');







% function [tau,V,Vdot] = rne(params)
%     %% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
%     %
%     % Inputs: params - a structure containing the following fields:
%     %           params.g - 3-dimensional column vector describing the acceleration of gravity
%     %           params.S - 6xn matrix of screw axes (each column is an axis)
%     %           params.M - 4x4xn home configuration matrix for each link
%     %           params.G - 6x6xn spatial inertia matrix for each link
%     %           params.jointPos - n-dimensional column vector of joint coordinates
%     %           params.jointVel - n-dimensional column vector of joint velocities
%     %           params.jointAcc - n-dimensional column vector of joint accelerations
%     %           params.Ftip - 6-dimensional column vector representing the
%     %           wrench applied at the tip
%     %
%     % Output: tau  - n-dimensional column vector of generalized joint forces
%     %         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
%     %         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
%     %
%     % Forward iterations
%     n = width(params.S);

%     A = zeros(6,n);
%     V = zeros(6,n+1);
%     Vd = zeros(6,n+1);
%     F = zeros(6,n+1);
%     tau = zeros(n,1);

    
%     V(:,1) = zeros(6,1);
%     Vd(:,1) = [0 0 0 -params.g]';
%     F(:,end) = params.Ftip

%     for i = 1:n
%         Mi = params.M(:,:,i)

%         if i == 1
%             Mi_1 = eye(4)
%         else
%             Mi_1 = params.M(:,:,i-1)
%         end
%         Mi_1_i = pinv(Mi_1) * Mi
%         Si = params.S(:,i)
%         qi = params.jointPos(i)
%         qdi = params.jointVel(i)
%         qddi = params.jointAcc(i)

%         Ai = Adjoint(pinv(Mi)) * Si
%         A(:,i) = Ai

%         Ti_i_1 = twist2ht(Si, -qi) * pinv(Mi_1_i)
%         % index of V and Vd are larger by 1
%         V(:,i+1) = Ai * qdi + Adjoint(Ti_i_1) * V(:,i)
%         Vd(:,i+1) = Ai * qddi + Adjoint(Ti_i_1) * Vd(:,i) + ad(V(:,i+1)) * Ai * qdi
%     end
    
%     % Backward iterations

%     for ii = 1:n
%         i = n-i+1
%         Gi = params.G(:,:,i)
%         Mi = params.M(:,:,i)

%         if i == n
%             Mi1 = eye(4)
%         else
%             Mi1 = params.M(:,:,i+1)
%         end
%         Mi1_i = pinv(Mi1) * Mi
%         Si = params.S(:,i)
%         qi = params.jointPos(i)
%         qdi = params.jointVel(i)
%         qddi = params.jointAcc(i)

%         Vi = V(:,i+1)
%         Vdi = Vd(:,i+1)

%         Ti1_i = twist2ht(Si, -qi) * Mi1_i
%         F(:,i) = Gi * Vdi - ad(Vi) * Gi * Vi + Adjoint(Ti1_i)' * F(:,i+1);
%         tau(i) = F(:,i)' * Ai


%     end
    

%     tau = tau
%     V = V
%     Vdot = Vd
    
% end




