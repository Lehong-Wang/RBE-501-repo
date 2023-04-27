function jointAcc = fdyn(params)
%% FDYN Implements the Forward Dynamics of a Serial Kinematic Chain
%
 % Inputs: params - a structure containing the following fields:
%           params.g - 3-dimensional column vector describing the acceleration of gravity
%           params.S - 6xn matrix of screw axes (each column is an axis)
%           params.M - 4x4xn home configuration matrix for each link
%           params.G - 6x6xn spatial inertia matrix for each link
%           params.jointPos - n-dimensional column vector of joint coordinates
%           params.jointVel - n-dimensional column vector of joint velocities
%           params.tau - n-dimensional column vector of joint torques/forces
%           params.Ftip - 6-dimensional column vector representing the wrench applied at the tip
%
% Output:   jointAcc - n-dimensional column vector of joint accelerations
%
% Author: L. Fichera, loris@wpi.edu
% Last Updated: 4/02/2023

% Step 1: Calculate the Mass Matrix column-by-column using the RNE
% algorithm
n = size(params.jointPos,1);
MM = zeros(n,n);

params_rne.g = zeros(3,1);
params_rne.S = params.S;
params_rne.M = params.M;
params_rne.G = params.G;
params_rne.jointPos = params.jointPos;
params_rne.jointVel = zeros(n,1);
params_rne.Ftip = zeros(6,1);

for ii = 1 : n
    % Set all accelerations to zero, except for the ith joint
    params_rne.jointAcc = zeros(n,1);
    params_rne.jointAcc(ii) = 1;

    % Calculate the ith column of the mass matrix
    MM(:,ii) = rne(params_rne);
end


% Step 2: Calculate the Centripetal/Coriolis terms using the RNE algorithm
params_rne.jointVel = params.jointVel;
params_rne.jointAcc = zeros(n,1);
h1 = rne(params_rne);

% Step 3: Calculate the Gravity terms using the RNE algorithm
params_rne.jointVel = zeros(n,1);
params_rne.g = params.g;
h2 = rne(params_rne);

% Step 4: Calculate the end effector wrench using the RNE algorithm
params_rne.g = zeros(3,1);
params_rne.Ftip = params.Ftip;
JtFtip = rne(params_rne);

% Step 5: Solve the forward dynamics equation
jointAcc = MM \ (params.tau - h1 - h2 - JtFtip);

if isnan(jointAcc)
    display('NAN')
end


end

