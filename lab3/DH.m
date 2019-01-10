% computes H_sub_i_super_i-1, the homogenous transformation matrix
% that moves a point from frame i to i-1, given an entire DH table
function H=DH(a, alpha, d, theta)
    H = [eye(3) [0 0 0]'; 0 0 0 1];
    n = length(a);
    for i = 1:n
        NextH = [rotz(theta(i)) [0 0 0]'; [0 0 0 1]]*...
                [eye(3) [0 0 d(i)]'; [0 0 0 1]]*...
                [eye(3) [a(i) 0 0]'; [0 0 0 1]]*...
                [rotx(alpha(i)) [0 0 0]'; [0 0 0 1]];
        H = H*NextH;
    end
end