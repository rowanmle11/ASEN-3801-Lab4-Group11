function [Fc, Gc] = InnerLoopFeedback(var)

    xE = var(1);
    yE = var(2);
    zE = var(3);
    phi = var(4);
    theta = var(5);
    psi = var(6);
    uEB = var(7);
    vEB = var(8);
    wEB = var(9);
    p = var(10);
    q = var(11);
    r = var(12);

    m = .068;
    g = 9.81;
    Lc = -0.00116*phi - 0.000696*p;
    Mc = -0.00144*theta - 0.000864*q;
    Nc = -0.004*r;
    Fc = [0 0 -m*g/(cos(phi)*cos(theta))]';
    Gc = [Lc Mc Nc]';



end

