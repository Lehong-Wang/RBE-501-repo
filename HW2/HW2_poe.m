
% % Feel free to replace the values below with arbitrary values
% omega = [0 0 1]; % Make sure that omega is a unit vector
% theta = pi/2;
% axisangle2rot(omega,theta)



% % Feel free to replace the inputs below with arbitrary values
% % (Make sure that S is a valid screw axis)
% theta = pi/2;
% S     = [0 0 1 0 0 0]';
% 
% T = twist2ht(S,theta)



% The code below defines the screw axes of a 3DOF robot, its configuration q, and the home pose M
S = [0 0 1 0 0 0;
     1 0 0 0 0.3 0;
     0 0 0 0 0 -1]';

q = [rand() * 2 * pi, rand() * 2 * pi, rand() * 0.3];

R_home = [0 1 0; 1 0 0; 0 0 -1]';
t_home = [0.3 0 0.3]';
M = [R_home t_home; 0 0 0 1];

% Calculate the forward kinematics
fkine(S,M,q)





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



function T = twist2ht(S,theta)

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





function T = fkine(S,M,q)

    T = eye(4);
    for i = 1: width(S)
        Si = S(:,i);
        qi = q(i);
        TF = twist2ht(Si, qi);
        T = T * TF;
    end
    T = T * M;
end






