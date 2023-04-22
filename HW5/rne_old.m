

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
    n = width(params.S);

    A = zeros(6,n);
    % first index is base frame
    V = zeros(6,n+1);
    Vd = zeros(6,n+1);
    % last index is F tip
    F = zeros(6,n+1);
    tau = zeros(n,1);

    
    V(:,1) = zeros(6,1);
    Vd(:,1) = [0 0 0 -params.g]';
    F(:,end) = params.Ftip;

    for i = 1:n
        Mi = eye(4)
        for j = 1:i-1
            Mi = Mi * M(:,:,j)
        end
        % Mi = params.M(:,:,i);

        % if i == 1
        %     Mi_1 = eye(4);
        % else
        %     Mi_1 = params.M(:,:,i-1);
        % end
        % Mi_1_i = pinv(Mi_1) * Mi;
        Si = params.S(:,i);
        qi = params.jointPos(i);
        qdi = params.jointVel(i);
        qddi = params.jointAcc(i);

        Ai = Adjoint(pinv(Mi)) * Si;
        A(:,i) = Ai;

        % Ti_i_1 = twist2ht(Si, -qi) * pinv(Mi_1_i);
        Ti_i_1 = twist2ht(Ai, -qi) * pinv(Mi(:,:,i-1));
        % index of V and Vd are larger by 1
        V(:,i+1) = Ai * qdi + Adjoint(Ti_i_1) * V(:,i);
        Vd(:,i+1) = Ai * qddi + Adjoint(Ti_i_1) * Vd(:,i) + ad(V(:,i+1)) * Ai * qdi;
    end
    
    % Backward iterations

    params.M(:,:,end+1) = params.M(:,:,end)
    for ii = 1:n
        % ii
        i = n-ii+1
        Gi = params.G(:,:,i);

        Mi = params.M(:,:,i);

        % if i == n
        %     Mi_i1 = eye(4);

        % else
            Mi1 = params.M(:,:,i+1);
            Mi_i1 = pinv(Mi) * Mi1

        % end

        % Mi1_i = pinv(Mi1) * Mi;
        % Mi = params.M(:,:,i)

        % Mi1 = params.M(:,:,i);

        % if i == 1
        %     Mi_1 = eye(4);
        % else
        %     Mi_1 = params.M(:,:,i-1);
        % end
        % Mi_i1 = pinv(Mi) * Mi1


        Si = params.S(:,i);
        qi = params.jointPos(i);
        qdi = params.jointVel(i);
        qddi = params.jointAcc(i);

        Vi = V(:,i+1);
        Vdi = Vd(:,i+1);

        % Ti1_i = twist2ht(Si, -qi) * Mi1_i

        if i == n
            Ti1_i = eye(4)
        else
            Si1 = params.S(:,i+1);
            Ai1 = A(:,i+1);
            qi1 = params.jointPos(i+1);
            % Ti1_i = twist2ht(Si1, -qi1) * pinv(Mi_i1)
            Ti1_i = twist2ht(Ai1, -qi1) * pinv(Mi_i1)
        end
        % if i == n
        %     Ti1_i = eye(4);
        % end
        % Ti1_i
        F(:,i) = Gi * Vdi - ad(Vi) * Gi * Vi + Adjoint(Ti1_i)' * F(:,i+1);
        tau(i) = F(:,i)' * Ai;


    end
    

    tau = tau;
    V = V;
    Vdot = Vd;
    
end

