function J = jacob0(S,q)
    J = [];
    T = eye(4);
    for i = 1:width(S)
        skew_axis = S(:,i);
        theta = q(i);
        % Add to jacobian for this link with Trans from privious link
        J = [J adjoint(skew_axis, T)];
        % calculate T for next link
        T = T * twist2ht(skew_axis, theta);
    end
end