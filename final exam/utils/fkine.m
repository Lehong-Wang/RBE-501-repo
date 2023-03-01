

function T = fkine(S,M,q)

    T = eye(4);
    for i = 1: width(S)
        Si = S(:,i);
        qi = q(i);
        TF = twist2ht(Si, qi);
        T = T * TF;
    end
    T = T * M;
end