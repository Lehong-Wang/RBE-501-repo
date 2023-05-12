function V_b = twistspace2body(V_s,T)
    % Skew Axis in body frame
    % Input:
    %   V_s : Skew axis in space frame
    %   T : transformation from space to body frame
    % Output:
    %   V_b : Skew axis in body frame

    V_b = Adjoint(V_s, inv(T));

end