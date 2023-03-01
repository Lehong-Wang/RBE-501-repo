function J_a = jacoba(S,M,q)    
    J_s = jacob0(S,q);
    J_w = J_s(1:3,:);
    J_v = J_s(4:6,:);
    % forward kinamatic to get p
    T = fkine(S, M, q);
    p = T(1:3, 4);

    J_a = J_v - skew(p)*J_w;

end