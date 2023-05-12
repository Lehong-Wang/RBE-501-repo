

function T = twist2ht(S,theta)

    % Product of Exponential
    % T = e^([S]?)
    %   = [e^([w]?) ]
    % T_0_n = Product(T) * M
    % 
    I = eye(3);
    st = sin(theta);
    ct = cos(theta);
    omega = [S(1); S(2); S(3)];
    v = [S(4); S(5); S(6)];
    w = [0          -omega(3)   omega(2);
         omega(3)   0           -omega(1);
         -omega(2)  omega(1)    0;];
    R = axisangle2rot(omega,theta);
    coner = (I*theta + (1-ct)*w + (theta-st)*w*w) * v;
    
    T = [R coner;
        0 0 0 1];
end



