function [Fc, Gc] = VelocityReferenceFeedback(t, var)

m = 1.5;
g = 9.80665;
T = 2.0; % time
s_target = 1.0; % distance to target

k1x = 6.96e-4;
k2x = 1.16e-3;
k1y = 8.64e-4;
k2y = 1.44e-3;
k_yaw = 4e-3;

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

Vp = (pi * s_target) / (2 * T); % peak velocity
if t < 0 || t > T
    u_ref = 0;
    v_ref = 0;
else
    u_ref = Vp * sin(pi * t / T);   % inertial x velocity reference
    v_ref = Vp * sin(pi * t / T);   % inertial y velocity reference
end

e_u = u_ref - u;
e_v = v_ref - v;

theta_ref = k1x * e_u;   % desired pitch
phi_ref   = k1y * e_v;   % desired roll

%% Moments

Mx = k1y*(phi_ref - phi) + k2y*(-p);
My = k1x*(theta_ref - theta) + k2x*(-q);
Mz = -k_yaw*r;

Gc = [Mx; My; Mz];

%% Force command

Fc = [0; 0; m*g];

end
