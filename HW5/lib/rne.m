
function [tau,V,Vdot] = rne(params)

    %% RNE Implements the Recursive Newton-Euler Inverse Dynamics Algorithm
    %
    % Inputs: params - a structure containing the following fields:
    %           params.g - 3-dimensional column vector describing the acceleration of gravity
    %           params.S - 6xn matrix of screw axes (each column is an axis)
    %           params.M - 4x4xn home configuration matrix for each link
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


    n = width(S);

    V = zeros(6, n+1);
    Vdot = zeros(6, n+1);
    % g   
    % transpose([0 0 0 -g(1) -g(2) -g(3)])
    Vdot(:,1) = transpose([0 0 0 -g(1) -g(2) -g(3)]);

    % home config transformation matrix of each joint to base frame
    T_m = zeros(4,4,n+1);
    T = zeros(4,4,n+1);
    A = zeros(6,n);



    % Forward Iterations
    for ii = 1 : n
        Mi = eye(4);

        for jj = 1 : ii
            Mi = Mi * M(:,:,jj);
        end
        % T_m(:,:,ii) = Mi;

        Ai = Adjoint(pinv(Mi)) * S(:,ii);
        A(:,ii) = Ai;
        Ti = twist2ht(Ai, -q(ii)) * pinv(M(:,:,ii));
        T(:,:,ii) = Ti;

        V(:,ii+1) = Ai * qd(ii) + Adjoint(Ti) * V(:,ii);
        Vdot(:,ii+1) = Ai * qdd(ii) + ad(V(:,ii+1)) * Ai * qd(ii) + Adjoint(Ti) * Vdot(:,ii);
    end

    T_m(:,:,end) = T_m(:,:,end-1);
    T(:,:,end) = T(:,:,end-1);
    % M
    % T_m
    % T
        
    F = zeros(6,n+1);
    tau = zeros(n,1);

    F(:,end) = Ftip;

    % Backward Iterations
    % for i = 1 : n
    %     ii = n - i + 1;

    for ii = n:-1:1

        
        % Mi1 = T_m(:,:,i+1);
        % Ai1 = Adjoint(pinv(Mi1) * S(:,ii+1));
        % Ti1 = twist2ht(Ai1, -q(ii+1))
        Ti1 = T(:,:,ii+1);
        F(:,ii) = G(:,:,ii) * Vdot(:,ii+1) - ...
                    ad(V(:,ii+1))' * (G(:,:,ii) * V(:,ii+1)) + ...
                    Adjoint(Ti1)' * F(:,ii+1);
        tau(ii) = F(:,ii)' * A(:,ii);
    end


    % for ii = n:-1:1

    % for i = number_of_links:-1:1
    %     if i == number_of_links
    %         f(:,:,i+1) = f_external(1,:)';
    %         n(:,:,i+1) = f_external(2,:)';
    %     end
    %     f(:,:,i) = R(:,:,i+1)\f(:,:,i+1) + F(:,:,i);
    %     n(:,:,i) = N(:,:,i) + R(:,:,i+1)\n(:,:,i+1) + cross(mass_center_list(i,:)',F(:,:,i))...
    %                 + cross(P(:,:,i+1),R(:,:,i+1)\f(:,:,i+1));
    %     tau_list (k,i) = dot(n(:,:,i),z);
    % end

end
