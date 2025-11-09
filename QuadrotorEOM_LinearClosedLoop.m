function var_dot = QuadrotorEOM_LinearClosedLoop(t, var, g, m, I, d, km, nu, mu)
    
% Unpack state vector

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

    Ix = I(1);
    Iy = I(2);
    Iz = I(3);

    L = -mu*sqrt(p^2 + q^2 + r^2)*p;
    M = -mu*sqrt(p^2 + q^2 + r^2)*q;
    N = -mu*sqrt(p^2 + q^2 + r^2)*r;

    [Fc] = InnerLoopFeedback(var);

    xEDot = uE;
    yEDot = vE;
    zEDot = wE;

    phiDot = p;
    thetaDot = q;
    psiDot = r;

    uEDot = -g*theta;
    vEDot = g*phi;
    wEDot = (Fc(3))/m;

    pDot = L/Ix;
    qDot = M/Iy;
    rDot = N/Iz;

% Pack and output state vector
    var_dot = [xEDot yEDot zEDot phiDot thetaDot psiDot uEDot vEDot wEDot pDot qDot rDot]';

end