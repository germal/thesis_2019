% Group: CJ Jang, Minho, Eugene
clf;
clear all;
clc;
% Load bag for LIDAR
bagDir = 'SLAMBag2.bag';
if(~exist('bag'))
    bag = rosbag(bagDir);
end

% %Select only the lidar scans - for ICP
% lidarMsgs = select(bag, 'Topic', '/scan');
% scans = readMessages(lidarMsgs);

%Select scan and odometry to build an occupancy grid mapping
rosMsgs = select(bag, 'Topic', {'/scan','/odom','/raspicam_node/image/compressed'});
scan_odom_msg = readMessages(rosMsgs);

% Create a map for the scan
N = 1000;
M = 1000;
% this will decide the resolution of the map.
map = zeros(M,N);

% finding out the maximum range of each x and y

odomMsgs = select(bag, 'Topic', '/odom');
ts_odom = timeseries(odomMsgs, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');


max_x = max(ts_odom.Data(:,1))+3.5;
min_x = min(ts_odom.Data(:,1))-3.5;
max_y = max(ts_odom.Data(:,2))+3.5;
min_y = min(ts_odom.Data(:,2))-3.5;
% Since the maximum range of the laser scan is 3.5, the dimension of the
% scan grid is the range of x and y travelled + 3.5 * 2.

% go through the bag
for n = 1:size(scan_odom_msg,1)
    % There are less scans then odom info.
    % meaning that provided that we have at least one of messages from each
    % topic, we can update the grid mapping every time we access the scan.
    % check if the msg contains the scan or odometry
    if scan_odom_msg{n}.MessageType == "sensor_msgs/CompressedImage"
        
      

        % Check the loop closure and do the thing.
        
        
        
        
    elseif scan_odom_msg{n}.MessageType == "nav_msgs/Odometry"
        
        % if it is a odom, keep updating the current x and y and its heading
        % of the robot itself
        current_x = scan_odom_msg{n}.Pose.Pose.Position.X;
        current_y = scan_odom_msg{n}.Pose.Pose.Position.Y;
        current_heading = quat2eul([scan_odom_msg{n}.Pose.Pose.Orientation.W scan_odom_msg{n}.Pose.Pose.Orientation.X scan_odom_msg{n}.Pose.Pose.Orientation.Y scan_odom_msg{n}.Pose.Pose.Orientation.Z]); % orientation as quarternion. so you would need to convert this into a single yaw.
        current_heading = current_heading(1); % taking euler angle of rotation.
        headings(n) = current_heading(1);
        % Put the result with the kalman filter here.
        
        
        
        
        
    elseif scan_odom_msg{n}.MessageType == "sensor_msgs/LaserScan"
        
        % if it is a scan, convert scan message into cartesian space
        laser_scan = scan_odom_msg{n};
        
        %Convert scan message to a list of 360 points in cartesian space
        % x and y indicates the points in laser scan.
        
        % apply rotation to the world map relative to the base link of the robot.
        % Translate from the scan coordinate to grid coordinate.
        cart = readCartesian(laser_scan);
        x = cart(:,1);
        y = cart(:,2);
        v = [x y];
        x_centre = current_x;
        y_centre = current_y;
        centre = (repmat([x_centre; y_centre], 1, length(x)));
        theta = current_heading;
        % rotation matrix of the laser scan points
        R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
        s = transpose(v);
        so = R*s;
        % add the location of the centre ( which is the odometry info)
        vo = so + centre;
        x_rotated = vo(1,:);
        y_rotated = vo(2,:);
        
        figure(1)
        plot(x_rotated , y_rotated, x_centre, y_centre, 'bo');
        axis([min_x max_x min_y max_y]);      
        hold on;
        
        % Iterate through vo in order to count the occupancy grid.
        
        for k = 1:length(vo)
            
            % access each x and y value in vo
            x_vo = vo(1,k);
            y_vo = vo(2,k);
            
            % The following code will determine which nth and mth element
            % the point belongs to in a grid map (N x M)
            grid_x =  (max_x - min_x)/N;
            grid_y =  (max_y - min_y)/M;
            temp = x_vo;
            nth = 1;
            while temp > min_x
                temp = temp - grid_x;
                nth = nth +1;
            end
            temp = y_vo;
            mth = 1;
            while temp > min_y
                temp = temp - grid_y;
                mth = mth + 1;
            end
            
            % update the grid
            
            map(nth,mth) = map(nth,mth)+ 1;
            
            
        end
    end
end

pre_map = map;

% threshold for the grid
threshold = 10;
% Convert the matrix into a binary map

for c = 1:N
    for k = 1:M
        if map(c,k) < threshold
            map(c,k) = 0; % free space
        else
            map(c,k) = 1; % occupied
        end
    end
end

% creating a bitmap
hold on;
figure(2);

%legend('bitmap');
spy(map);
title(['Occupancy Grid for N=',num2str(N),', M=',num2str(M),' Threshold =',num2str(threshold)]);




