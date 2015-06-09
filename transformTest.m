close all
clear all
clear rosbag_wrapper;
clear ros.Bag;

%%
topic = '/cloud_2';
scan_nr = 1;
phi = -3;
fov_s = 1;
fov_d = 800;


%% Load Rosbag
%bag = ros.Bag.load('spiral_matlab/2015-03-11-22-16-30.bag');
bag = ros.Bag.load('spiral_matlab/2015-03-09_Tracktest/track_testing.bag');
% bag.info()

%% Creat Transformed Pointcloud Vector
bag.resetView(topic);

for count = 0:scan_nr;
    msg = bag.read();
end

    phi = phi*pi/180;

% msg.points = msg.points(:,fov_s:fov_s+fov_d); % Reduce the field of view

 
    xbla = + cos(phi)*msg.points(2,:) + sin(phi)*msg.points(1,:);
    zbla = - cos(phi)*msg.points(1,:) + sin(phi)*msg.points(2,:);


% 
% 
figure
plot(msg.points(1,:),msg.points(2,:),'x');
axis equal

% Rotate pointcloud to match template.
xi = -cos(phi)*msg.points(2,:) + sin(phi)*msg.points(1,:);
zi = -cos(phi)*msg.points(1,:) - sin(phi)*msg.points(2,:);

figure
plot(xbla,zbla,'x');
axis equal

figure
plot(xi,zi,'x');
axis equal

%% Initialize Variables for Stairparam Creation

% 
% dz0 = 0.6;
% 
% zbla = zbla + dz0;

