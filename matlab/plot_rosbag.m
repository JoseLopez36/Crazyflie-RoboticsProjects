clear all
close all
clc

% Parameters
bParsePose = true;
bParseSetpoint = true;

% Load the rosbag
% Update the path to your rosbag file
% Assuming script is run from 'matlab' folder and rosbag is in 'experiments/rosbag_success'
rosbag_path = fullfile('..', 'experiments', 'rosbag_success', 'test_rosbag_0.db3');

if ~isfile(rosbag_path)
    warning('Rosbag file not found at: %s', rosbag_path);
    % Try absolute path if relative fails (fallback logic or manual adjust needed)
end

try
    bag = ros2bagreader(rosbag_path);
catch ME
    error('Failed to load rosbag: %s. \nMake sure you have the ROS Toolbox installed.', ME.message);
end

% Topics
% /cf_0/control/setpoint (geometry_msgs/msg/PointStamped)
% /cf_1/control/setpoint (geometry_msgs/msg/PointStamped)
% /cf_0/pose (geometry_msgs/msg/PoseStamped)
% /cf_1/pose (geometry_msgs/msg/PoseStamped)

topics_pose = ["/cf_0/pose", "/cf_1/pose"];
topics_setpoint = ["/cf_0/control/setpoint", "/cf_1/control/setpoint"];

% Data containers
data_pose = containers.Map;
data_setpoint = containers.Map;

% Parse Pose Data
if bParsePose
    for i = 1:length(topics_pose)
        topic = topics_pose(i);
        fprintf('Parsing topic: %s\n', topic);
        try
            bag_sel = select(bag, "Topic", topic);
            if bag_sel.NumMessages > 0
                data_pose(topic) = ParsePose(bag_sel);
            else
                warning('No messages found for topic: %s', topic);
            end
        catch ME
            warning('Error parsing topic %s: %s', topic, ME.message);
        end
    end
end

% Parse Setpoint Data
if bParseSetpoint
    for i = 1:length(topics_setpoint)
        topic = topics_setpoint(i);
        fprintf('Parsing topic: %s\n', topic);
        try
            bag_sel = select(bag, "Topic", topic);
            if bag_sel.NumMessages > 0
                data_setpoint(topic) = ParsePoint(bag_sel);
            else
                warning('No messages found for topic: %s', topic);
            end
        catch ME
            warning('Error parsing topic %s: %s', topic, ME.message);
        end
    end
end

% Plotting
figure('Name', 'Crazyflie Trajectories', 'Color', 'w');
hold on;
grid on;
axis equal;
title('Crazyflie Trajectories');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');

colors = {'b', 'r', 'g', 'm'};
names = {'cf_0', 'cf_1'};

% Plot Poses (Solid lines)
for i = 1:length(topics_pose)
    topic = topics_pose(i);
    if isKey(data_pose, topic)
        d = data_pose(topic);
        color = colors{mod(i-1, length(colors)) + 1};
        plot3(d.x, d.y, d.z, '-', 'Color', color, 'LineWidth', 1.5, ...
            'DisplayName', [names{i} ' Pose']);
        % Mark start
        scatter3(d.x(1), d.y(1), d.z(1), 50, color, 'filled', ...
            'HandleVisibility', 'off', 'MarkerEdgeColor', 'k');
    end
end

% Plot Setpoints (Dashed lines)
for i = 1:length(topics_setpoint)
    topic = topics_setpoint(i);
    if isKey(data_setpoint, topic)
        d = data_setpoint(topic);
        color = colors{mod(i-1, length(colors)) + 1};
        plot3(d.x, d.y, d.z, '--', 'Color', color, 'LineWidth', 1.0, ...
            'DisplayName', [names{i} ' Setpoint']);
    end
end

legend('show', 'Location', 'best');
view(3);

%% Helper Functions

function out = ParsePose(bag_selection)
% Reads geometry_msgs/PoseStamped messages
msgs = readMessages(bag_selection);
N = length(msgs);

out.time = zeros(N, 1);
out.x = zeros(N, 1);
out.y = zeros(N, 1);
out.z = zeros(N, 1);

for k = 1:N
    msg = msgs{k};
    % Handle different message struct formats (struct vs object)
    if isstruct(msg)
        out.time(k) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        out.x(k) = msg.pose.position.x;
        out.y(k) = msg.pose.position.y;
        out.z(k) = msg.pose.position.z;
    else
        out.time(k) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        out.x(k) = msg.pose.position.x;
        out.y(k) = msg.pose.position.y;
        out.z(k) = msg.pose.position.z;
    end
end
end

function out = ParsePoint(bag_selection)
% Reads geometry_msgs/PointStamped messages
msgs = readMessages(bag_selection);
N = length(msgs);

out.time = zeros(N, 1);
out.x = zeros(N, 1);
out.y = zeros(N, 1);
out.z = zeros(N, 1);

for k = 1:N
    msg = msgs{k};
    if isstruct(msg)
        out.time(k) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        out.x(k) = msg.point.x;
        out.y(k) = msg.point.y;
        out.z(k) = msg.point.z;
    else
        out.time(k) = double(msg.header.stamp.sec) + double(msg.header.stamp.nanosec)*1e-9;
        out.x(k) = msg.point.x;
        out.y(k) = msg.point.y;
        out.z(k) = msg.point.z;
    end
end
end
