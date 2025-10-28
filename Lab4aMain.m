clear
clc
close all

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
tols = odeset('RelTol', 1e-12, 'Abstol', 1e-12);
uEE0 = 0;
vEE0 = 5;
wEE0 = 0;
VEE0 = [uEE0 vEE0 wEE0]';

xE0 = 0;
yE0 = 0;
zE0 = 0;
phi0 = atan(nu*25/(m*g));
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

T = m*g/cos(phi0);
f1 = T/4; f2 = T/4; f3 = T/4; f4 = T/4;
motor_forces = [f1; f2; f3; f4];

[t, var] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), tspan, var0, tols);

Zc = -sum(motor_forces);
Lc = (d/sqrt(2))*(-motor_forces(1) - motor_forces(2) + motor_forces(3) + motor_forces(4));
Mc = (d/sqrt(2))*(motor_forces(1) - motor_forces(2) - motor_forces(3) + motor_forces(4));
Nc = km*(motor_forces(1) - motor_forces(2) + motor_forces(3) - motor_forces(4));
control_input_array = repmat([Zc; Lc; Mc; Nc], 1, length(t));

fig = 1:6;
PlotAircraftSim(t', var', control_input_array, fig, 'b-')
%sgtitle('Hover Simulation - Task 1')