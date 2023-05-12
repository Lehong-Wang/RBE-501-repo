function Vtrans = Adjoint(V,T)
    switch nargin
        case 1
            T = V;
            P = T(1:3, 4);
            R = T(1:3, 1:3);
            Ad = [  R           zeros(3);
                    skew(P)*R    R   ];

            Vtrans = Ad;
        case 2
            P = T(1:3, 4);
            R = T(1:3, 1:3);
            Ad = [  R           zeros(3);
                    skew(P)*R    R   ];

            Vtrans = Ad * V;
        otherwise
            disp("Please input Skew axis and starting transformation matrix(optional)")
            Vtrans = 0;
    end
end