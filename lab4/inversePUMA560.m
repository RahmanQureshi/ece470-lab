function q = inverse(H,myrobot)
    o_ee = H(1:3,4);
    o_c = o_ee - H(1:3,1:3)*[0;0;20];
    xc = o_c(1);
    yc = o_c(2);
    zc = o_c(3);
    
    %Calculate q1
    alpha = atan2(yc,xc);
    beta = atan2(23.65, real(sqrt(xc*xc + yc*yc - 23.65*23.65)));
    
    q1 = alpha - beta;
    
    D = ((xc*xc + yc*yc + (zc-76)*(zc-76) - 23.65*23.65 - 43.228*43.228 - 43.18*43.18)/(2*43.228*43.18));
    q3 = atan2(D,(real(sqrt(1-D*D))));
    
    delta = atan2(-43.18*cos(q3), 43.228 + 43.18*sin(q3));
    q2 = atan2(zc-76, real(sqrt(xc*xc+yc*yc-23.65*23.65))) - delta;
    
    H_10 = [cos(q1) 0 sin(q1)  0;
            sin(q1) 0 -cos(q1) 0;
            0       1 0        76;
            0       0 0        1];
    
    H_21 = [cos(q2) -sin(q2) 0   43.18*cos(q2);
            sin(q2) cos(q2)  0   43.18*sin(q2);
            0       0        1   -23.65;
            0       0        0   1];
        
    H_32 = [cos(q3) 0 sin(q3)  0;
            sin(q3) 0 -cos(q3) 0;
            0       1 0        0;
            0       0 0        1];
        
    H_30 = H_10*H_21*H_32;
    
    R_30 = H_30(1:3,1:3);
    
    R_63 = R_30'*H(1:3,1:3);
    
    q5 = atan2(real(sqrt(1-R_63(3,3)*R_63(3,3))),R_63(3,3));
    q4 = atan2(R_63(2,3), R_63(1,3));
    q6 = atan2(R_63(3,2), -R_63(3,1));
    
    
    q = [q1 q2 q3 q4 q5 q6];
end