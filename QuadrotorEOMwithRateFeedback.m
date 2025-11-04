function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, mu, nu)
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
    %p = 0.1;
    q = var(11);
    %q = 0.1;
    %r = var(12);
    r = 0.1;

    d = 0.06;
    km = 0.0024;

    [Fc, Gc] = RotationDerivativeFeedback(var, m, g);

    motor_forces = ComputeMotorForces(Fc, Gc, d, km);

    f1 = motor_forces(1);
    f2 = motor_forces(2);
    f3 = motor_forces(3);
    f4 = motor_forces(4);

    Zc = -f1 - f2 - f3 - f4;
    Lc = (d/sqrt(2))*(-f1 - f2 + f3 + f4);
    Mc = (d/sqrt(2))*(f1 - f2 - f3 + f4);
    Nc = km*(f1 - f2 + f3 - f4);

    Fa = -nu*[uE*abs(uE); vE*abs(vE); wE*abs(wE)];
    Ma = -mu*[p*abs(p); q*abs(q); r*abs(r)];

    R_be = angle2dcm(psi, theta, phi, 'ZYX');
    F_body = [0;0;m*g] + [0;0;Zc] + Fa;
    acc_body = (1/m) * F_body - cross([p;q;r], [uE;vE;wE]);

    u_dot = acc_body(1);
    v_dot = acc_body(2);
    w_dot = acc_body(3);

    moment_body = [Lc;Mc;Nc] + Ma;
    p_dot = (moment_body(1) - (Iz - Iy)*q*r)/Ix;
    q_dot = (moment_body(2) - (Ix - Iz)*p*r)/Iy;
    r_dot = (moment_body(3) - (Iy - Ix)*p*q)/Iz;

    phi_dot = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
    theta_dot = q*cos(phi) - r*sin(phi);
    psi_dot = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

    v_inertial = R_be' * [uE;vE;wE];
    xE_dot = v_inertial(1);
    yE_dot = v_inertial(2);
    zE_dot = v_inertial(3);

    var_dot = [xE_dot; yE_dot; zE_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end