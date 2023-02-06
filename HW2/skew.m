function S = skew(v)
%SKEW Returns the skew-symmetric matrix associated with the 3D vector
%passed as input

if length(v) == 3
    S = [  0   -v(3)  v(2)
        v(3)  0    -v(1)
        -v(2) v(1)   0];
else
    error('argument must be a 3-vector');
end

end

