    %% Finding the angle of the wheelchair on the stairs
clear all;
close all;

%% Initialize Variables
tic

fov_s = 811-280-220;                 % Startpoint of FoV. Defined for scanner 1
fov_e = 811-280;
fov_d = fov_e - fov_s;               % Size of FoV

phi_1 = -3;                          % Angle of sensors to stairdiagonal
phi_2 = 1;

scan_s = 1;                          % Startpoint of beta calculation
scan_d = 0;                        % Size of beta calculation

v0_1 = [.17;.3;0.05;0;1*pi/180];     % v0 = [heigth, depth, phase offset, sensor height, sensor rotation]
v0_2 = [.17;.3;0.12;0;1*pi/180];

v_r_1 = v0_1;
v_r_2 = v0_2;

beta_v  = zeros(0,scan_d);
v_v_1   = zeros(length(v0_1),scan_d);
v_v_2   = zeros(length(v0_2),scan_d);

%% Run fminsearch over mutliple scans.

for scan_nr = scan_s:scan_s+scan_d;
[v_r_1,z_r_1,se_r_1,xf_1,zf_1] = matching('/cloud_1',scan_nr,phi_1,fov_s,fov_d,v_r_1);
[v_r_2,z_r_2,se_r_2,xf_1,zf_1] = matching('/cloud_2',scan_nr,phi_2,811-fov_s-fov_d,fov_d,v_r_2);

a = .63;                            % Distance between sensors
beta = 180/pi*atan((v_r_1(3)-v_r_2(3))/a);

beta_v(scan_nr-(scan_s-1)) = beta;
v_v_1(:,scan_nr-(scan_s-1)) = v_r_1(:);
v_v_2(:,scan_nr-(scan_s-1)) = v_r_2(:);
end

beta_m = [beta_v;beta_v;beta_v;beta_v;beta_v];
v_v_r_1 = v_v_1.*cos(beta_m*pi/180);
v_v_r_2 = v_v_2.*cos(beta_m*pi/180);

%% Plot Results
if scan_d ~= 0;
figure
subplot(3,1,1)
plot(v_v_r_1(1,:));
xlabel('Scan Nummber'),ylabel('Treppenhöhe [m]');
subplot(3,1,2)
plot(v_v_r_1(2,:));
xlabel('Scan Nummber'),ylabel('Treppentiefe [m]');
subplot(3,1,3)
plot(v_v_r_1(3,:));
xlabel('Scan Nummber'),ylabel('Phasenverschiebung [m]');

figure
subplot(3,1,1)
plot(v_v_1(1,:));
hold on
plot(v_v_2(1,:));
hold on
plot(v_v_1(1,:) - v_v_2(1,:));
legend('/cloud_1','/cloud_2','delta')
xlabel('Scan Nummber'),ylabel('Treppenhöhe [m]');

subplot(3,1,2)
plot(v_v_1(2,:));
hold on
plot(v_v_2(2,:));
hold on
plot(v_v_1(2,:) - v_v_2(2,:));
legend('/cloud_1','/cloud_2','delta')
xlabel('Scan Nummber'),ylabel('Treppentiefe [m]');

subplot(3,1,3)
plot(v_v_1(3,:));
hold on
plot(v_v_2(3,:));
hold on
plot(v_v_1(3,:) - v_v_2(3,:));
legend('/cloud_1','/cloud_2','delta')
xlabel('Scan Nummber'),ylabel('Phasenverschiebung [m]');

figure
plot(beta_v);
hold on
% beta_v_s = smooth(beta_v);
% plot(beta_v_s);
xlabel('Scan Nummber'),ylabel('\beta [°] - Angle of Wheelchair on Stairs ');

end

toc

