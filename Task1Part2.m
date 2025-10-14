% Author: Naomi Ellis
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
f2 = 0; % Motor Force 1 [N]
f3 = 0; % Motor Force 1 [N]
f4 = 0; % Motor Force 1 [N]
motor_forces = [f1; f2; f3; f4;]; % Motor Force Vector [N]
tols = odeset('RelTol', 1e-12,'AbsTol',1e-12);

% State Vector
var = ; % State vector 

% Calculations
% For trim conditions, f1=f2=f3=f4=mg/4
W = m*g/2;
f1 = W/4;
f2 = W/4;
f3 = W/4;
f4 = W/4;

% ode45 call -> I don't remember which of the variables need to be in the @
[x t] = ode45(@(t,var,g,m,I,d,km,nu,mu,motor_forces), tspan, x0, tols);

% plotting


% ODE45 Function
function var_dot = OdeFunc(t, var, g, m, I, d, km, nu, mu, motor_forces)
    
% Unpack state vector
    

% Calculate derivatives


% Pack and output state vector


end