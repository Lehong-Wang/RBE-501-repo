function J_b = jacobe(S,M,q)
    % Body Jacobian
    % (Space Jacobian in body frame)
    J_s = jacob0(S,q);
    T = fkine(S,M,q);
    J_b = Adjoint(J_s, inv(T));
end