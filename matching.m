function [v_r,z_r] = matching(topic,scan_nr,phi,fov_s,fov_d,v0)
clear rosbag_wrapper;
clear ros.Bag;

%% Load Rosbag
%bag = ros.Bag.load('spiral_matlab/2015-03-11-22-16-30.bag');
bag = ros.Bag.load('spiral_matlab/2015-03-09_Tracktest/track_testing.bag');
% bag.info()

%% Creat Transformed Pointcloud Vector
bag.resetView(topic);

for count = 0:scan_nr;
    msg = bag.read();
end


msg.points = msg.points(:,fov_s:fov_s+fov_d); % Reduce the field of view
if strcmp('/cloud_1',topic)
    M(1,:) = -msg.points(2,:);
    M(2,:) = -msg.points(1,:);
elseif strcmp('/cloud_2',topic)
    M(1,:) = msg.points(2,:);
    M(2,:) = -msg.points(1,:);
end

% figure
% plot(M(1,:),M(2,:),'x');
% axis equal

% Rotate pointcloud to match template.
phi = phi*pi/180;
MT = M;
MT(1,:) = cos(phi)*M(1,:) - sin(phi)*M(2,:);
MT(2,:) = cos(phi)*M(2,:) + sin(phi)*M(1,:);

xi = MT(1,:);
zi = MT(2,:);

%% Initialize Variables for Stairparam Creation
h0 = .17;
t0 = .28;
dx0 = 0.12;
dz0 = .600;

t = t0;
h = h0;
dx = dx0;
dz = dz0;

zi = zi + dz;
%% Find z for x values

[v_r,se_r,z_r] = stairparam(xi,zi,v0);
%disp(se_r)

% %  
% figure
% plot(xi,zi,'x');
% axis equal tight
% hold on
% plot(xi,z_r,'o')
% axis equal tight
% hold on

end