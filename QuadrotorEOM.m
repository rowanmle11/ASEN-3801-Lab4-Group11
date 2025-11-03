% Author: Naomi Ellis, Antonio Jimeno-Fernandez
% Date: 10/14/2025
% Lab 4 - Quadrotor Simulation and Control

% ODE45 Function
function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
    
% Unpack state vector

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

    Ix = I(1);
    Iy = I(2);
    Iz = I(3);

    Zc = -motor_forces(1) - motor_forces(2) - motor_forces(3) - motor_forces(4);
    Lc = motor_forces(1)*-d/sqrt(2) + motor_forces(2)*-d/sqrt(2) + motor_forces(3)*d/sqrt(2) + motor_forces(4)*d/sqrt(2);
    Mc = motor_forces(1)*d/sqrt(2) + motor_forces(2)*-d/sqrt(2) + motor_forces(3)*-d/sqrt(2) + motor_forces(4)*d/sqrt(2);
    Nc = km*motor_forces(1) - km*motor_forces(2) + km*motor_forces(3) - km*motor_forces(4);

% Calculate derivatives

    xEDot = (cos(theta)*cos(psi))*uE + (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*vE + (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*wE;
    yEDot = (cos(theta)*sin(psi))*uE + (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*vE + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*wE;
    zEDot = (-sin(theta))*uE + (sin(phi)*cos(theta))*vE + (cos(phi)*cos(theta))*wE; 

    phiDot = (1)*p + (sin(phi)*tan(theta))*q + (cos(phi)*tan(theta))*r;
    thetaDot = (0)*p + (cos(phi))*q + (-sin(phi))*r;
    psiDot = (0)*p + (sin(phi)*sec(theta))*q + (cos(phi)*sec(theta))*r;

    uEDot = (r*vE - q*wE) + g*(-sin(theta)) + (0)/m;
    vEDot = (p*wE - r*uE) + g*(cos(theta)*sin(phi)) + (0)/m;
    wEDot = (q*uE - p*vE) + g*(cos(theta)*cos(phi)) + (Zc)/m;

    pDot = (q*r*(Iy - Iz)/Ix) + (Lc/Ix);
    qDot = (p*r*(Iz - Ix)/Iy) + (Mc/Iy);
    wDot = (q*q*(Ix - Iy)/Iz) + (Nc/Iz);

% Pack and output state vector
    var_dot = [xEDot yEDot zEDot phiDot thetaDot psiDot uEDot vEDot wEDot qDot pDot wDot]';

end