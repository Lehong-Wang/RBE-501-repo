
addpath('util')

% % Feel free to replace the values below with arbitrary values
% omega_s = [0 0 1]';

% R       = [cos(pi/4) -sin(pi/4) 0;
%            sin(pi/4)  cos(pi/4) 0;
%            0 0 1];

% angvelocityspace2body(omega_s,R)


% % Feel free to replace the values below with arbitrary values
% V_s = [0 0 1 0 0 0]'; % Twist in the space frame

% R       = [cos(pi/4) -sin(pi/4) 0;
%            sin(pi/4)  cos(pi/4) 0;
%            0 0 1];

% p = [0 0 1]';

% T = [R p; 0 0 0 1]; % Transformation matrix between {s} and {b}

% twistspace2body(V_s,T)






% The code below defines the screw axes of a 3DOF robot, its configuration q, and the home pose M
S = [0 0 1 0 0 0;
     1 0 0 0 0.3 0;
     0 0 0 0 0 -1]';

q = [rand() * 2 * pi, rand() * 2 * pi, rand() * 0.3];

R_home = [0 1 0; 1 0 0; 0 0 -1]';
t_home = [0.3 0 0.3]';
M = [R_home t_home; 0 0 0 1];

% Calculate the forward kinematics using the PoE in the space frame
%fkine(S,M,q,'space')

% Calculate the forward kinematics using the PoE in the body frame
fkine(S,M,q,'body')



% The code below defines the screw axes of a 3DOF robot and its configuration q
S = [0 0 1 0 0 0;
     1 0 0 0 0.3 0;
     0 0 0 0 0 -1]';

R_home = [0 1 0; 1 0 0; 0 0 -1]';
t_home = [0.3 0 0.3]';
M = [R_home t_home; 0 0 0 1];

q = [rand() * 2 * pi, rand() * 2 * pi, rand() * 0.3];

% Calculate the forward kinematics
jacobe(S,M,q)



function omega_b = angvelocityspace2body(omega_s,R)
    % Ws = Rdt * R-1
    % Wd = R-1 * Rdt
    omega_s_skew = skew(omega_s);
    Rdt = omega_s_skew * R
    omega_b_skew = inv(R) * Rdt
    omega_b = [omega_b_skew(3,2) omega_b_skew(1,3) omega_b_skew(2,1)]'
end


function V_b = twistspace2body(V_s,T)
    V_s
    T
    V_b = adjoint(V_s, inv(T))
    % V_b = Ad_T * V_s
end


function T = fkine(S,M,q,frame)
    % your code here
    T = eye(4);
    for i = 1: width(S)
        Si = S(:,i);
        qi = q(i);
        TF = twist2ht(Si, qi);
        T = T * TF;
    end

    if strcomp(frame, 'space')
        T = T * M;
    elseif strcomp(frame, 'body')

        T = M * T;
    else
        disp("Not Valid Frame Name")
    end



end


function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q)
    T = fkine(S,M,q)
    J_b = adjoint(J_s, inv(T))
end


