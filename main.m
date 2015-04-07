%% Finding the angle of the wheelchair on the stairs
clear all;
close all;

%% Initialize Variables
tic

fov_s = 260;        % Startpoint of FoV. Defined for scanner 1
fov_d = 300;        % Size of FoV

phi_1 = -2;            % Angle of sensors to stairdiagonal
phi_2 = 1;

scan_s = 200;          % Startpoint of beta calculation
scan_d = 100;            % Size of beta calculation

beta_v = zeros( 0,scan_s+scan_d);
dx_1 = zeros( 0,scan_s+scan_d);
dx_2 = zeros( 0,scan_s+scan_d);


v0 = [.10;.28;0.12];   % v0 = [heigth, depth, phase offset]

v_r_1 = v0;
v_r_2 = v0;

%% Run fminsearch over mutliple scans.

for scan_nr = scan_s:scan_s+scan_d;
% v0_1 = v_r_1;
% v0_2 = v_r_2;
[v_r_1,z_r_1] = matching('/cloud_1',scan_nr,phi_1,fov_s,fov_d,v_r_1);
[v_r_2,z_r_2] = matching('/cloud_2',scan_nr,phi_2,811-fov_s-fov_d,fov_d,v_r_2);

a = .63;            % Distance between sensors
beta = 180/pi*atan((v_r_1(3)-v_r_2(3))/a);

beta_v(scan_nr-(scan_s-1)) = beta;
dx_1(scan_nr-(scan_s-1)) = v_r_1(1);
dx_2(scan_nr-(scan_s-1)) = v_r_2(1);
end

figure
plot(beta_v);
hold on
beta_v_s = smooth(beta_v);
plot(beta_v_s);
xlabel('Scan Nummber'),ylabel('\beta');


figure
plot(dx_1);
hold on
plot(dx_2);
hold on
plot(dx_1 - dx_2);
legend('dx_1','dx_2','delta')
xlabel('Scan Nummber'),ylabel('Phasenverschiebung [m]');

toc

