function [v_r,z_r,se_r, xf, zf] = matching(topic,scan_nr,phi,fov_s,fov_d,v0)
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

figure
subplot(2,1,1)
plot(M(1,:),M(2,:),'x');
axis equal tight


% Rotate pointcloud to match template.
phi = phi*pi/180;
MT = M;
MT(1,:) = cos(phi)*M(1,:) - sin(phi)*M(2,:);
MT(2,:) = cos(phi)*M(2,:) + sin(phi)*M(1,:);

xi = MT(1,:);
zi = MT(2,:);

%% Initialize Variables for Stairparam Creation

dz0 = 0.6;

zi = zi + dz0;
%% Find z for x values

%% [v_r,se_r,z_r,xf,zf] = stairparam(xi,zi,v0);
[v_r,se_r,z_r,xf,zf] = curvefit(xi,zi,v0);
disp(se_r)

subplot(2,1,2)
plot(xf,zf,'x');
axis equal tight
hold on
plot(xf,z_r,'o')
axis equal tight
hold on
xlabel(topic);

end