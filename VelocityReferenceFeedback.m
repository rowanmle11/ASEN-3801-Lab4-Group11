function [Fc, Gc] = VelocityReferenceFeedback(t, var)

g = 9.81;
m = 0.068;
T = 2.0; % time
distance = 1;

kp = 0.000696;
kphi = 0.00116;
kq = 0.000864;
ktheta = 0.00144;
kr = .004;
ku = .001;
kv = .001;

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

if t < T
    uRef = .5;
    vRef = .5;
else
    uRef = 0;   % inertial x velocity reference
    vRef = 0;   % inertial y velocity reference
end

%% Moments

Lc = -kphi*phi - kp*p + kv*(vRef - vE);
Mc = -ktheta*theta - kq*q - ku*(uRef - uE);
Nc = -kr*r;

Gc = [Lc; Mc; Nc];

%% Force command

Fc = [0; 0; -m*g/(cos(phi)*cos(theta))];

end
