clear rosbag_wrapper;
clear ros.Bag;
clear all;
close all;

%% Load a bag and get information about it
% Using load() lets you auto-complete filepaths.
% bag = ros.Bag.load('spiral_matlab/2015-03-11_Stairtest/2015-03-11-22-16-30.bag');
bag = ros.Bag.load('spiral_matlab/2015-03-09_Tracktest/track_testing.bag');
bag.info();

%% Read all messages on a few topics
topic1 = '/cloud_1';
msgs = bag.readAll({topic1});

fprintf('Read %i messages\n', length(msgs));

%% Read messages incrementally
bag.resetView(topic1);
count = 0;
subplot(2,1,1)
for count = 0:200;
    msg = bag.read();
end
    msg.points = msg.points(:,260:560); %Reduce the field of view
    %rotate image
    M(1,:) = -msg.points(2,:);
    M(2,:) = -msg.points(1,:);
    plot(M(1,:),M(2,:),'x');
    axis equal
    % plot(-msg.points(2,:),-msg.points(1,:),'x');
hold on



%% Transform Pointcloud to binary Image

for i = 1:length(M)
   M(1,i) = round(100*M(1,i));                                 %Convert to [cm]
   M(2,i) = round(100*M(2,i));                                 %Convert to [cm]
end

M_min = min(transpose(M));
M_max = max(transpose(M));

w = M_max(1) - M_min(1);
h = M_max(2) - M_min(2);
BW = ones(h + 1,w + 1);                                        % Create Binary Image

for i = 1:length(M)
    BW(M_max(2) - M(2,i) + 1,M(1,i) - M_min(1) + 1) = 0;       % Fill Binary Image
end

subplot(2,1,2)
imshow(BW)

hold on

%% Create Custom Hough Transform

ntheta = 180;
nrho = round(2 * sqrt(w^2 + h^2));
accu = zeros(nrho,ntheta);

for i = 1:length(M)
    for t=1:ntheta
        r = round(nrho/2 + M(1,i) * cos(t*pi/180) + M(2,i) * sin(t*pi/180));
        
%         if r>1
%             accu(r-1,t) = accu(r-1,t) + 2;
%         end
%         if r<nrho
%             accu(r+1,t) = accu(r+1,t) + 2;
%         end
%         if t>1
%             accu(r,t-1) = accu(r,t-1) + 2;
%         end
%         if t<ntheta
%             accu(r,t+1) = accu(r,t+1) + 2;
%         end

        accu(r,t) = accu(r,t) + 1;
    end
end

rho = -nrho/2:nrho/2;
theta = 1:ntheta;

figure
subplot(2,1,1)
imshow(imadjust(mat2gray(accu)),'InitialMagnification','fit');
xlabel('\theta'), ylabel('\rho');
% imshow(accu,'InitialMagnification','fit');
axis on, axis normal, hold on;
colormap(hot);


max_val=0;
for i=1:nrho
    for j=1:ntheta
        if ( accu(i,j) > max_val)
            max_val = accu(i,j);
        end
    end
end
%% Find Peaks of Hough Transform
% 
% thresh = 23;
% npeaks = 0;
% for i=1:nrho
%     for j=1:ntheta
%         if ( accu(i,j) > thresh)
%             npeaks = npeaks + 1;
%             cpeaks(npeaks,1) = i;
%             cpeaks(npeaks,2) = j;
%         end
%     end
% end

cpeaks = houghpeaks(accu,20,'Threshold',20);

plot(cpeaks(:,2),cpeaks(:,1),'s','color','red');
    

subplot(2,1,2)
imshow(BW)
hold on;

subplot(2,1,2)
for i = 1:length(cpeaks);
    xT = -[0;(cpeaks(i,1) - nrho/2)*cos(cpeaks(i,2)*pi/180)];
    yT = -[0;(cpeaks(i,1) - nrho/2)*sin(cpeaks(i,2)*pi/180)];
    plot(yT,xT)
    axis equal
    hold on
end

%% Plot Lines

lines = houghlines(BW, theta, rho, cpeaks,'FillGap',70,'MinLength',1);
max_len = 0;
figure
imshow(BW,'Border','tight')
hold on;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
   hold on
   % Plot beginnings and ends of lines
    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
    hold on
    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red'); 
    hold on
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end


%% Hough Transform

% [H, theta, rho] = hough(BW);

%%
%     imshow(H,[],'XData',theta,'YData',rho,...
%             'InitialMagnification','fit');
%     xlabel('\theta'), ylabel('\rho');% %%
%     imshow(H,[],'XData',theta,'YData',rho,...
%             'InitialMagnification','fit');
%     xlabel('\theta'), ylabel('\rho');
%     axis on, axis normal, hold on;
% 
%     axis on, axis normal, hold on;
%%
% peaks = houghpeaks(H,8);
% 
%     x = theta(peaks(:,2)); 
%     y = rho(peaks(:,1));
%     %plot(x,y,'s','color','black');
%     for i = 1:length(peaks)
%     xT = [0;y(i)*cos(x(i)*pi/180)];
%     yT = [0;y(i)*sin(x(i)*pi/180)];
%     plot(xT,yT)
%     end
% 
% lines = houghlines(BW, theta, rho, peaks,'FillGap',5,'MinLength',7);
%% Plot Peaks
% 
% max_len = 0;
% for k = 1:length(lines)
%    xy = [lines(k).point1; lines(k).point2];
%    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
% 
%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
% % 
%    % Determine the endpoints of the longest line segment
%    len = norm(lines(k).point1 - lines(k).point2);
%    if ( len > max_len)
%       max_len = len;
%       xy_long = xy;
%    end
% end

% %highlight the longest line segment
% plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','blue');
