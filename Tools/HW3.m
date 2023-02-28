

% 
% % Feel free to change the code below to try different twists and transformation matrices
% V = [0 0 1 0 0 0]';
% T = eye(4);
% T = [1 2 3 4; 5 6 7 8; 9 10 11 12; 13 14 15 16];
% adjoint(V,T)
% 

% 
% % The code below defines the screw axes of a 3DOF robot and its configuration q
% S = [0 0 1 0 0 0;
%      1 0 0 0 0.3 0;
%      0 0 0 0 0 -1]';
% 
% q = [rand() * 2 * pi, rand() * 2 * pi, rand() * 0.3];
% 
% % Calculate the forward kinematics
% jacob0(S,q)
% 



% The code below defines the screw axes of a 3DOF robot and its configuration q
S = [0 0 1 0 0 0;
     1 0 0 0 0.3 0;
     0 0 0 0 0 -1]';

R_home = [0 1 0; 1 0 0; 0 0 -1]';
t_home = [0.3 0 0.3]';
M = [R_home t_home; 0 0 0 1];

q = [rand() * 2 * pi, rand() * 2 * pi, rand() * 0.3];

% Calculate the forward kinematics
jacoba(S,M,q)





function Vtrans = adjoint(V,T)
    P = T(1:3, 4);
    R = T(1:3, 1:3);
    Ad = [  R           zeros(3);
            skew(P)*R    R   ];
    Vtrans = Ad * V;
end





function J = jacob0(S,q)
    J = [];
    T = eye(4);
    for i = 1:width(S)
        skew_axis = S(:,i)
        theta = q(i)
        % Add to jacobian for this link with Trans from privious link
        J = [J adjoint(skew_axis, T)]
        % calculate T for next link
        T = T * twist2ht(skew_axis, theta)
    end
end



function J_a = jacoba(S,M,q)    
    J_s = jacob0(S,q);
    J_w = J_s(1:3,:)
    J_v = J_s(4:6,:)
    % forward kinamatic to get p
    T = fkine(S, M, q);
    p = T(1:3, 4)

    J_a = J_v - skew(p)*J_w

end



