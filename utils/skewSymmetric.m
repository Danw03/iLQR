function S = skewSymmetric(s)
% fuction to transfer vector to skew-symmetric matrix
S = [0 -s(3) s(2);
    s(3) 0 -s(1);
    -s(2) s(1) 0];
end