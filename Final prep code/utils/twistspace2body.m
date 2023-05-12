function V_b = twistspace2body(V_s,T)
    V_b = Adjoint(V_s, inv(T));
    % V_b = Ad_T * V_s
end