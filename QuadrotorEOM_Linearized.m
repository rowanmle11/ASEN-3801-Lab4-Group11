function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
    Ix = I(1);
    Iy = I(2);
    Iz = I(3);

    xE = var(1);
    yE = var(2);
    zE = var(3);
    phi = var(4);
    %phi = deg2rad(5);
    theta = var(5);
    %theta = deg2rad(5);
    psi = var(6);
    %psi = deg2rad(5);
    uE = var(7);
    vE = var(8);
    wE = var(9);
    p = var(10);
    %p = 0.1;
    q = var(11);
    %q = 0.1;
    %r = var(12);
    r = 0.1;

    dZc = deltaFc(3);
    dLc = deltaGc(1);
    dMc = deltaGc(2);
    dNc = deltaGc(3);

    uEDot = r*vE - q*wE - g*theta + deltaFc(1)/m;
    vEDot = -r*uE + p*wE + g*phi + deltaFc(2)/m;
    wEDot = q*uE - p*vE + dZc/m;

    pDot = dLc/Ix;
    qDot = dMc/Iy;
    rDot = dNc/Iz;

    phiDot = p;
    thetaDot = q;
    psiDot = r;

    xEDot = uE;
    yEDot = vE;
    zEDot = wE;

    var_dot = [xEDot; yEDot; zEDot; phiDot; thetaDot; psiDot; uEDot; vEDot; wEDot; pDot; qDot; rDot];
end