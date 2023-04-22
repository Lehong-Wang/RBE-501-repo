
function [tau,V,Vdot] = rne(params)


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

    Vdot(:,1) = [0 0 0 -g]';

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
        T_m(:,:,ii) = Mi;

        Ai = Adjoint(pinv(Mi)) * S(:,ii);
        A(:,ii) = Ai;
        Ti = twist2ht(Ai, -q(ii)) * pinv(M(:,:,ii));
        T(:,:,ii) = Ti;

        V(:,ii+1) = Ai * qd(ii) + Adjoint(Ti) * V(:,ii);
        Vdot(:,ii+1) = Ai * qdd(ii) + ad(V(:,ii+1)) * Ai * qd(ii) + Adjoint(Ti) * Vdot(:,ii);
    end

    T_m(:,:,end) = T_m(:,:,end-1);
    T(:,:,end) = T(:,:,end-1);
    M
    T_m
    T
        
    F = zeros(6,n+1);
    tau = zeros(n,1);

    F(:,end) = Ftip

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
                    Adjoint(Ti1)' * F(:,ii+1)
        tau(ii) = F(:,ii)' * A(:,ii)
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
