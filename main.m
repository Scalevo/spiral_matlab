%% Finding the angle of the wheelchair on the stairs
clear all;
close all;

%% Initialize Variables
tic

fov_s = 260;           % Startpoint of FoV. Defined for scanner 1
fov_d = 300;           % Size of FoV

phi_1 = -3;            % Angle of sensors to stairdiagonal
phi_2 = 1;

scan_s = 100;          % Startpoint of beta calculation
scan_d = 0;           % Size of beta calculation

beta_v = zeros(0,scan_d);
v_v_1   = zeros(3,scan_d);
v_v_2   = zeros(3,scan_d);


v0_1 = [.16;.35;0.05];   % v0 = [heigth, depth, phase offset]
v0_2 = [.16;.35;0.12];

v_r_1 = v0_1;
v_r_2 = v0_2;

%% Run fminsearch over mutliple scans.

for scan_nr = scan_s:scan_s+scan_d;
[v_r_1,z_r_1,se_r_1] = matching('/cloud_1',scan_nr,phi_1,fov_s,fov_d,v_r_1);
[v_r_2,z_r_2,se_r_2] = matching('/cloud_2',scan_nr,phi_2,811-fov_s-fov_d,fov_d,v_r_2);

a = .63;            % Distance between sensors
beta = 180/pi*atan((v_r_1(3)-v_r_2(3))/a);

beta_v(scan_nr-(scan_s-1)) = beta;
v_v_1(:,scan_nr-(scan_s-1)) = v_r_1(:);
v_v_2(:,scan_nr-(scan_s-1)) = v_r_2(:);
end

%% Plot Results
% 
% figure
% plot(beta_v);
% hold on
% beta_v_s = smooth(beta_v);
% plot(beta_v_s);
% xlabel('Scan Nummber'),ylabel('\beta');
% 
% 
% figure
% plot(v_v_1(3,:));
% hold on
% plot(v_v_2(3,:));
% hold on
% plot(v_v_1(3,:) - v_v_2(3,:));
% legend('/cloud_1','/cloud_2','delta')
% xlabel('Scan Nummber'),ylabel('Phasenverschiebung [m]');
% 
% figure
% plot(v_v_1(1,:));
% hold on
% plot(v_v_2(1,:));
% hold on
% plot(v_v_1(1,:) - v_v_2(1,:));
% legend('/cloud_1','/cloud_2','delta')
% xlabel('Scan Nummber'),ylabel('Treppenhöhe [m]');
% 
% figure
% plot(v_v_1(2,:));
% hold on
% plot(v_v_2(2,:));
% hold on
% plot(v_v_1(2,:) - v_v_2(2,:));
% legend('/cloud_1','/cloud_2','delta')
% xlabel('Scan Nummber'),ylabel('Treppentiefe [m]');

toc

