function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% Create a function to calculate the control vectors Fc and Gc. The function takes as input the 12x1
% aircraft state var, aircraft mass m, and gravitational acceleration g. The control force in the body 𝑧𝑧-
% direction should still equal the weight of the quadrotor. Set the control moments about each body
% axis proportional to the rotational rates about their respective axes, but in the opposite sign of the
% angular velocity with a gain of 0.004 Nm/(rad/sec)
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

    Weight = m*g;
    gain = .004;
    Fc = [0 0 Weight/(cos(phi)*cos(theta))]';
    Gc = [-gain*p -gain*q -gain*r]';
end