function T = tdh(theta, d, a, alpha)

    % DH parameter to transformation matrix

    st = sin(theta);
    ct = cos(theta);
    sa = sin(alpha);
    ca = cos(alpha);

    T = [   ct  -st*ca  st*sa   a*ct;
            st  ct*ca   -ct*sa  a*st;
            0   sa      ca      d;
            0   0       0       1;];
end