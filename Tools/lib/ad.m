function adV = ad(V)

% 

    w = V(1:3);
    v = V(4:6);
    adV = [skew(w) zeros(3);
           skew(v) skew(w)];
end