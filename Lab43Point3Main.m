clear; clc; close all

g = 9.81;
m = 0.068;
I = [5.8e-5; 7.2e-5; 1.0e-4];
d = 0.06;
km = 0.0024;
nu = 1e-3;
mu = 2e-6;

t0 = 0;
tf = 10;
tspan = [t0 tf];
%tols = odeset('RelTol', 1e-12, 'Abstol', 1e-12);
uEE0 = 0;
vEE0 = 0;
wEE0 = 0;
VEE0 = [uEE0 vEE0 wEE0]';

xE0 = 0;
yE0 = 0;
zE0 = 0;
phi0 = deg2rad(5);
theta0 = 0;
psi0 = 0;

DCM = DCM_x(phi0)*DCM_y(theta0)*DCM_z(psi0);

VEB0 = DCM*VEE0;
uE0 = VEB0(1);
vE0 = VEB0(2);
wE0 = VEB0(3);
p0 = 0;
q0 = 0;
r0 = 0;
var0 = [xE0 yE0 zE0 phi0 theta0 psi0 uE0 vE0 wE0 p0 q0 r0];

[t, var] = ode45(@(t,var) QuadrotorEOM_LinearClosedLoop(t, var, g, m, I, d, km, nu, mu), tspan, var0);

for i = 1   :length(var)
    [Fc(i, :)] = InnerLoopFeedback(var(i, :));
end
Zc = Fc(:, 3);

control_input_array = repmat(Zc, 1, length(t));

fig = 1:6;
PlotAircraftSim(t', var', control_input_array, fig, 'b-')