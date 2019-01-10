function q = inverseKUKA(Hd, kuka)
    q = zeros(6, 1);
    od = Hd(1:3, 4);
    Rd = Hd(1:3, 1:3);
    oc = od - Rd*[-abs(kuka.a(6)); 0; kuka.d(6)];
    xc = oc(1); yc = oc(2); zc = oc(3);
    q(1) = atan2(yc, xc);
    r = sqrt(xc^2 + yc^2);
    D = ( (r-kuka.a(1))^2 + (zc-kuka.d(1))^2 - kuka.a(2)^2 -kuka.d(4)^2 - kuka.a(3)^2 ) / ...
        ( 2*kuka.a(2)*sqrt(kuka.d(4)^2 + kuka.a(3)^2) );
    q(3) = atan2(D, real(sqrt(1-D^2))) - atan2(kuka.a(3), kuka.d(4));
    tempTheta = q(3) - pi/2 + atan2(kuka.a(3), kuka.d(4));
    tempH = sqrt(kuka.d(4)^2 + kuka.a(3)^2);
    gamma = atan2( tempH*sin(tempTheta), kuka.a(2) + tempH*cos(tempTheta));
    q(2) = atan2(zc-kuka.d(1), r-kuka.a(1)) - gamma;
    H30 = DH(kuka.a(1:3), kuka.alpha(1:3), kuka.d(1:3), q(1:3));
    R30 = H30(1:3, 1:3);
    R63 = (R30')*Rd;
    q(4) = atan2(R63(2,3),R63(1,3));
    q(5) = atan2(sqrt(1-R63(3,3)^2), R63(3,3));
    q(6) = atan2(R63(3,2), -R63(3,1));
end