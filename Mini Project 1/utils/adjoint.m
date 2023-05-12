function Vtrans = adjoint(V,T)
    P = T(1:3, 4);
    R = T(1:3, 1:3);
    Ad = [  R           zeros(3);
            skew(P)*R    R   ];
    Vtrans = Ad * V;
end