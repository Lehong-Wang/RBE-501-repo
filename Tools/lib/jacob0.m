function J = jacob0(S,q)
    % Space Jacobian
    % Skew Axis of each joint in base frame
    % J_i = Ad(T_i) * S_i
    % Input:
    %   S : 6xn Skew Axis
    %   q : 6x1 joint variables
    % Output:
    %   J : 6xn Space Jacobian

    J = [];
    T = eye(4);
    for i = 1:width(S)
        skew_axis = S(:,i);
        theta = q(i);
        % Add to jacobian for this link with Trans from privious link
        J = [J Adjoint(skew_axis, T)];
        % calculate T for next link
        T = T * twist2ht(skew_axis, theta);
    end
end