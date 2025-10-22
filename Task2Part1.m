% Author: Naomi Ellis, Antonio Jimeno-Fernandez
% Date: 10/14/2025
% Lab 4 - Quadrotor Simulation and Control

% Housekeeping
clc;
clear;
close all;

% Declaration of Variables
t0 = 0; % Initial Time [s]
tf = 10; % Final Time [s]
tStep = 0.1; % Time Step [s]
tspan = [t0 tf]; % Time [s]
g = 9.81; % Gravitational Acceleration [m/s^2]
m = 0.068; % Mass [kg]
I = [5.8e-5; 7.2e-5; 1e-4;]; % Moment of Inertia [kg*m^2]
d = 0.06; % Radial Distance between CG and Propellor [m]
km = 0.0024; % Control moment coefficient k_m [Nm/N]
nu = 1e-3; % Aerodynamic force coefficient [N/(m/s)^2]
mu = 2e-6; % Aerodynamic moment coefficient [N*m/(rad/s)^2]
f1 = 0; % Motor Force 1 [N]
f2 = 0; % Motor Force 2 [N]
f3 = 0; % Motor Force 3 [N]
f4 = 0; % Motor Force 4 [N]
tols = odeset('RelTol', 1e-12,'AbsTol',1e-12);

% Calculations
% For trim conditions, f1=f2=f3=f4=mg/4
W = m*g;
f1 = W/4;
f2 = W/4;
f3 = W/4;
f4 = W/4;
motor_forces = [f1; f2; f3; f4;]; % Motor Force Vector [N]

% State Vector
% var = [xE yE zE phi theta psi uE vE wE p q r]'; % State vector 
var0 = zeros(12, 1);

% ode45 call -> I don't remember which of the variables need to be in the @
[t, var] = ode45(@(t,var) QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces), tspan, var0, tols);

Zc = -motor_forces(1) - motor_forces(2) - motor_forces(3) - motor_forces(4);
Lc = motor_forces(1)*-d/sqrt(2) + motor_forces(2)*-d/sqrt(2) + motor_forces(3)*d/sqrt(2) + motor_forces(4)*d/sqrt(2);
Mc = motor_forces(1)*d/sqrt(2) + motor_forces(2)*-d/sqrt(2) + motor_forces(3)*-d/sqrt(2) + motor_forces(4)*d/sqrt(2);
Nc = km*motor_forces(1) - km*motor_forces(2) + km*motor_forces(3) - km*motor_forces(4);
control_input_array = repmat([Zc; Lc; Mc; Nc], 1, length(t));

% plotting
fig = 1:6;
PlotAircraftSim(t', var', control_input_array, fig, 'b-')

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
    r = var(12);
    %r = 0.1;

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