% Author: Naomi Ellis
% Date: 11/2/2025
% Purpose: Create a function to calculate motor thrust forces given the
% control forces and moments for a quadcopter of standard design.

function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% Definition of Variables
% Fc is the control thrust vector
% Gc is the control moment vector
% d is the distance from aircraft CG to rotor
% km is the control moment coefficient
% For computational efficiency, perform some calculations once here: 
root2 = sqrt(2);

% Unpack input vectors, ignoring X and Y which are zero for a quadcopter
zlmn = [Fc(3); Gc(1); Gc(2); Gc(3)];

% Calculate original A matrix for the [Controls] = A * [Thrust Force] eq.
A = [-1, -1, -1, -1;
    -d/root2, -d/root2, d/root2, d/root2;
    d/root2, -d/root2, -d/root2, d/root2;
    km, -km, km, -km];

% Perform vector equation calculation for [Thrust Force] = A^-1 *
% [Controls] equation. Note that division is more efficient than
% multiplication by the inverse for MATLAB.
motor_forces = A\zlmn;
  

end
