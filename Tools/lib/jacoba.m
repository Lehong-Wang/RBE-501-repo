function J_a = jacoba(S,M,q)
    % Analytical Jacobian
    % Jacobian in different frame system (ie. XYZ coordinate instead of Skew coorfinate)
    % Output:
    %   J_a : 3xn Jacobian of end effector in XYZ coordinate

    J_s = jacob0(S,q);
    J_w = J_s(1:3,:);
    J_v = J_s(4:6,:);
    % forward kinamatic to get p
    T = fkine(S, M, q);
    p = T(1:3, 4);

    J_a = J_v - skew(p)*J_w;

end