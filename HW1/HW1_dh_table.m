

%% Question 5 - Rotation Matrices in Matlab

function T = trot(theta, axis)
    if axis == 'x'
        T = [1 0 0; 0 cos(theta) -sin(theta); 0 sin(theta) cos(theta)];
    elseif axis == 'y'
        T = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    elseif axis == 'z'
        T = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0; 0 0 1];
    end
end



%% Question 6 (DH Parameters in Matlab)

function T = tdh(theta, d, a, alpha)

    st = sin(theta);
    ct = cos(theta);
    sa = sin(alpha);
    ca = cos(alpha);

    T = [   ct  -st*ca  st*sa   a*ct;
            st  ct*ca   -ct*sa  a*st;
            0   sa      ca      d;
            0   0       0       1;];
end



%% Question 7 (forward kinematics of a SCARA robot)

function T = fwkinscara(q)
    % q is a 4x1 vector containing the generalized joint variables
    L1 = 0.5; % [m]
    L2 = 0.5; % [m]
    L4 = 0.1; % [m]
    
    T1 = tdh(q(1), 0, L1, 0);
    T2 = tdh(q(2), 0, L2, pi);
    T3 = tdh(0, q(3), 0, 0);
    T4 = tdh(-q(4), L4, 0, 0);

    T = T1*T2*T3*T4;
end





%% Question 8 (forward kinematics of a RPP robot)

function T = fwkinrpp(q)
    % q is a 4x1 vector containing the generalized joint variables
    L1 = 0.5; % [m]
    T1 = tdh(q(1), L1, 0, 0);
    T2 = tdh(-pi/2, q(2), 0, -pi/2);
    T3 = tdh(pi/2, q(3), 0, 0);

    T = T1*T2*T3;
end






