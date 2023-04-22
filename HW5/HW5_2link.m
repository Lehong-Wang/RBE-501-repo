

clear
clc

addpath("utils")

% % Feel free to change the code below to try different transformation matrices
% T = eye(4);
% Adjoint(T)


% % Feel free to change the code below to try different twists
% V1 = [0 0 1 0 0 0]';
% ad(V1)












%% Robot Definition:
n = 2;    % Number of links in the kinematic chain
L1 = 0.3; % [m] Length of the first link
L2 = 0.4; % [m] Length of the second link
m1 = 1;   % [kg] Mass of the first link
m2 = 1;   % [kg] Mass of the second link
g = -9.81;  % [m/s2] Gravity acceleration (aligned with the Y axis)

%% *** STEP 1 ***
% Calculate the home configurations of each link, expressed in the space frame      
R1 = [1 0 0; 0 1 0; 0 0 1]; P1 = [L1 0 0]';          
M1 = [R1 P1; 0 0 0 1]; % pose of frame {1} expressed in the {0} (space) reference frame
R2 = [1 0 0; 0 1 0; 0 0 1]; P2 = [L1+L2 0 0]'; 
M2 = [R2 P2; 0 0 0 1]; % pose of frame {2} expressed in the {0} (space) reference frame
R3 = [1 0 0; 0 1 0; 0 0 1]; P3 = [L1+L2 0 0]'; 
M3 = [R3 P3; 0 0 0 1]; % pose of frame {3} expressed in the {0} (space) reference frame

% Calculate the home configurations of each link, expressed w.r.t. the previous link frame
M01 = M1; % pose of frame {1} expressed in the {0} (space) reference frame
M12 = inv(M1)*M2; % pose of frame {2} expressed in the {1} reference frame
M23 = inv(M2)*M3; % pose of frame {3} expressed in the {2} reference frame

% Define the screw axes of each joint, expressed in the space frame
S = zeros(6,n);
S(:,1) = [0 0 1 -cross([0 0 1], [0 0 0])]';
S(:,2) = [0 0 1 -cross([0 0 1], [L1 0 0])]';

% Calculate the screw axes of each joint, expressed in the local link frame
A = zeros(6,n);
A(:,1) = Adjoint(pinv(M1)) * S(:,1);
A(:,2) = Adjoint(pinv(M2)) * S(:,2);


% Initialize the twists and accelerations of each link
V1 = zeros(6,1);
V2 = zeros(6,1);
Vd1 = zeros(6,1);
Vd2 = zeros(6,1);
 
% Initialize the joint positions and velocities
q = zeros(2,1);
qd = ones(2,1);
qdd = ones(2,1);
% q = [1; 2];
% qd = [3; 2];
% qdd = [4; 1];

%% *** STEP 2 ***

V0 = zeros(6,1);
% Vd0 = [0 0 0 0 -g 0]';
Vd0 = [0 0 0 0 0 -g]';

% Forward Iteration - First Link
T10 = twist2ht(S(:,1), -q(1)) * pinv(M1);
V1 =  A(:,1) * qd(1); % Link Velocity
Vd1 = A(:,1) * qdd(1) + Adjoint(T10) * Vd0 + ad(V1) * A(:,1) * qd(1); % Link Acceleration
     
% Forward Iteration - Second Link
% T21 = twist2ht(S(:,2), q(2)) * M12;
% inv(T21)
T21 = twist2ht(S(:,2), -q(2)) * pinv(M12);

V2 =  A(:,2) * qd(2) + Adjoint(T21) * V1; % Link Velocity
Vd2 = A(:,2) * qdd(2) + Adjoint(T21) * Vd1 + ad(V2) * A(:,2) * qd(2); % Link Acceleration


% V1
% Vd1
% V2
% Vd2



% %% Step 1: COPY-PASTE YOUR SOLUTION FOR THE JOINT VELOCITIES AND ACCELERATIONS BELOW
% V1 =  [0 0 1 0 .3 0]'; % Link Velocity
% Vd1 = [0 0 1 0 10.1 0]'; % Link Acceleration
% V2 =  [0 0 2 0 1.1 0]'; % Link Velocity
% Vd2 = [0 0 2 .3 10.9 0]'; % Link Acceleration

%% Step 2: Initialize the Spatial Inertia Matrices
G1 = [zeros(3) zeros(3); zeros(3) m1*eye(3)]; % Spatial Inertia Matrix for Link 1
G2 = [zeros(3) zeros(3); zeros(3) m2*eye(3)]; % Spatial Inertia Matrix for Link 2

%% Step 3: Calculate the Joint Torques
F3 = ones(6,1); % Wrench applied at the end effector

% Second joint
T32 = eye(4)
F2 = G2 * Vd2 - ad(V2)*G2*V2 + Adjoint(T32)' * F3;
tau2 = F2' * A(:,2);

% First joint
T21 = twist2ht(S(:,2), -q(2)) * pinv(M12)
F1 = G1 * Vd1 - ad(V1)*G1*V1 + Adjoint(T21)' * F2;
tau1 = F1' * A(:,1);










params.g =[0 0 -9.81]; % Gravity Vector [m/s^2]
params.S = S;  
parmas.M = zeros(4,4,n);
params.M(:,:,1) = M1;
params.M(:,:,2) = M2;
params.G = zeros(6,6,n)
params.G(:,:,1) = G1;
params.G(:,:,2) = G2;
params.jointPos = q;
params.jointVel = qd;
params.jointAcc = qdd;
params.Ftip = F3;


[tau_, V_, Vd_] = rne(params);

% [tau1; tau2]
% [V0 V1 V2]
% [Vd0 Vd1 Vd2]



% function AdT = Adjoint(T)
%     P = T(1:3, 4);
%     R = T(1:3, 1:3);
%     AdT = [  R           zeros(3);
%             skew(P)*R    R   ];

% end




% function adV = ad(V)
%     w = V(1:3);
%     v = V(4:6);
%     adV = [skew(w) zeros(3);
%            skew(v) skew(w)];
% end




