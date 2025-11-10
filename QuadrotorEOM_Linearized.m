function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
    Ix = I(1);
    Iy = I(2);
    Iz = I(3);

    xE = var(1);
    yE = var(2);
    zE = var(3);
    phi = var(4);
    theta = var(5);
    psi = var(6);
    uE = var(7);
    vE = var(8);
    wE = var(9);
    p = var(10);
    q = var(11);
    r = var(12);

    dZc = deltaFc(3);
    dLc = deltaGc(1);
    dMc = deltaGc(2);
    dNc = deltaGc(3);

    xEDot = uE;
    yEDot = vE;
    zEDot = wE;

    phiDot = p;
    thetaDot = q;
    psiDot = r;

    uEDot = -g*theta;
    vEDot = g*phi;
    wEDot = dZc/m;

    pDot = dLc/Ix;
    qDot = dMc/Iy;
    rDot = dNc/Iz;

    var_dot = [xEDot; yEDot; zEDot; phiDot; thetaDot; psiDot; uEDot; vEDot; wEDot; pDot; qDot; rDot];
end
