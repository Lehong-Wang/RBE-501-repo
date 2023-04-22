function J_b = jacobe(S,M,q)    
    J_s = jacob0(S,q);
    T = fkine(S,M,q);
    J_b = adjoint(J_s, inv(T));
end