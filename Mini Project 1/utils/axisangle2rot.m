function R = axisangle2rot(omega,theta)
    % Rodrigoes Formula
    I = eye(3);
    st = sin(theta);
    ct = cos(theta);
    w = [0          -omega(3)   omega(2);
         omega(3)   0           -omega(1);
         -omega(2)  omega(1)    0;];

    R = I + st * w + (1-ct) * w * w;
end