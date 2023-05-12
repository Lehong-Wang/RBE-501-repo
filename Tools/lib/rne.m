
function [tau,V,Vdot] = rne(params)

    %% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
    %
    % Inputs: params - a structure containing the following fields:
    %           params.g - 3-dimensional column vector describing the acceleration of gravity
    %           params.S - 6xn matrix of screw axes (each column is an axis)
    %           params.M - 4x4xn home configuration matrix for each link w.r.t. previous links
    %           params.G - 6x6xn spatial inertia matrix for each link
    %           params.jointPos - n-dimensional column vector of joint coordinates
    %           params.jointVel - n-dimensional column vector of joint velocities
    %           params.jointAcc - n-dimensional column vector of joint accelerations
    %           params.Ftip - 6-dimensional column vector representing the
    %           wrench applied at the tip
    %
    % Output: tau  - n-dimensional column vector of generalized joint forces
    %         V    - 6x(n+1) matrix - each column represents the twist of one of the robot's links
    %         Vdot - 6x(n+1) matrix - each column represents the acceleration of one of the robot's links
    %
    % Forward iterations

    g = params.g;
    S = params.S;
    M = params.M;
    G = params.G;
    q = params.jointPos;
    qd = params.jointVel;
    qdd = params.jointAcc;
    Ftip = params.Ftip;
    % num of joints
    n = width(S);
    % frame velocity & acceleration
    % first column is base frame
    V = zeros(6, n+1);
    Vdot = zeros(6, n+1);
    % -g for base fram acceleration
    Vdot(:,1) = transpose([0 0 0 -g(1) -g(2) -g(3)]);

    % Transformation matrix between frames
    T = zeros(4,4,n+1);
    % Skew axis in local (joint) frame
    A = zeros(6,n);


    % Forward Iterations
    for ii = 1 : n
        % home config w.r.t. space frame
        Mi = eye(4);
        for jj = 1 : ii
            Mi = Mi * M(:,:,jj);
        end
        % Transform Skew axis from space frame to local frame
        Ai = Adjoint(pinv(Mi)) * S(:,ii);
        A(:,ii) = Ai;
        % Transformation Matrix from last frame to this frame (ii-1 -> ii)
        Ti = twist2ht(Ai, -q(ii)) * pinv(M(:,:,ii));
        T(:,:,ii) = Ti;

        % local rotation + velocity from previous joint
        V(:,ii+1) = Ai * qd(ii) + Adjoint(Ti) * V(:,ii);
        % local acceleration + centripetal acceleration + acceleration from previous joint
        Vdot(:,ii+1) = Ai * qdd(ii) + ad(V(:,ii+1)) * Ai * qd(ii) + Adjoint(Ti) * Vdot(:,ii);
    end

    % Transformation from last link frame to end effector frame is home configration
    T(:,:,end) = pinv(M(:,:,end));

    % last column is Wrench on end effector
    F = zeros(6,n+1);
    F(:,end) = Ftip;

    % torque to be applyed to each joint
    tau = zeros(n,1);


    % Backward Iterations
    for ii = n:-1:1

        Ti1 = T(:,:,ii+1);
        % Wrench from local accelration + wrench from next frame
        F(:,ii) = G(:,:,ii) * Vdot(:,ii+1) - ...
                    ad(V(:,ii+1))' * (G(:,:,ii) * V(:,ii+1)) + ...
                    Adjoint(Ti1)' * F(:,ii+1);
        % Torque = Wrench.T * local skew
        tau(ii) = F(:,ii)' * A(:,ii);
    end


end
