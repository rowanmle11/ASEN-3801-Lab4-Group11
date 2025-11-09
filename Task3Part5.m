clear
clc
close all

m = 0.068;
g = 9.81;

k1x = 6.96e-4; % kp
k2x = 1.16e-3; % kphi

k1y = 8.64e-4; % kq
k2y = 1.44e-3; % ktheta

Ix = 5.8e-5;
Iy = 7.2e-5;

A_lat = [0    g       0;
         0    0       1;
         0 -k2x/Ix -k1x/Ix];
B_lat = [0; 0; 1/Ix];
C_lat = [1 0 0];

A_long = [0   -g       0;
          0    0       1;
          0 -k2y/Iy -k1y/Iy];
B_long = [0; 0; 1/Iy];
C_long = [1 0 0];

k3_values = linspace(0,5,200);
eig_lat = zeros(3,length(k3_values));
eig_long = zeros(3,length(k3_values));

for i = 1:length(k3_values)
    k3 = k3_values(i);

    Acl_lat = A_lat - B_lat * k3 * C_lat;
    eig_lat(:,i) = eig(Acl_lat);

    Acl_long = A_long - B_long * -k3 * C_long;
    eig_long(:,i) = eig(Acl_long);
end

figure
plot(real(eig_lat), imag(eig_lat), 'b.')
xlabel('Real Axis'); ylabel('Imag Axis')
title('Lateral System Eigenvalue Locus')
hold on
xline(-1/1.25, '--r', 'Target Time Constant (1.25 s)')
legend('Eigenvalues', 'Location', 'northwest')
grid on

figure
plot(real(eig_long), imag(eig_long), 'b.')
xlabel('Real Axis'); ylabel('Imag Axis')
title('Longitudinal System Eigenvalue Locus')
hold on
xline(-1/1.25, '--r', 'Target Time Constant (1.25 s)')
legend('Eigenvalues', 'Location', 'northwest')
grid on

target_tau = 1.25;
target_sigma = -1/target_tau;

k3_lat_final = NaN;
k3_long_final = NaN;

for i = 1:length(k3_values)
    if all(real(eig_lat(:,i)) < target_sigma)
        k3_lat_final = k3_values(i);
        break
    end
end
if isnan(k3_lat_final)
    [~,idx] = min(max(real(eig_lat), [], 1));
    k3_lat_final = k3_values(idx);
end

for i = 1:length(k3_values)
    if all(real(eig_long(:,i)) < target_sigma)
        k3_long_final = k3_values(i);
        break
    end
end
if isnan(k3_long_final)
    [~,idx] = min(max(real(eig_long), [], 1));
    k3_long_final = k3_values(idx);
end

fprintf('Selected Outer Loop Gains:\n')
fprintf('Lateral K3 = %.3f\n', k3_lat_final)
fprintf('Longitudinal K3 = %.3f\n', k3_long_final)

Acl_lat_final = A_lat - B_lat * k3_lat_final * C_lat;
Acl_long_final = A_long - B_long * -k3_long_final * C_long;

disp('Eigenvalues (Lateral)')
disp(eig(Acl_lat_final))

disp('Eigenvalues (Longitudinal)')
disp(eig(Acl_long_final))